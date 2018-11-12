#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include "opt-A2.h"
#include <mips/trapframe.h>
#include <synch.h>
#include <test.h>
#include <vfs.h>
#include <kern/fcntl.h>
#include <limits.h>

void sys__exit(int exitcode) {

  struct addrspace *as;
  struct proc *p = curproc;

  lock_acquire(lk);

  for (size_t i = 0; i < array_num(procTable); i++) {
    struct process *p = array_get(procTable, i);
    if (p->pid == curproc->pid) {
      p->exited = 1;
      p->exitcode = _MKWAIT_EXIT(exitcode);
    }
  }

  cv_broadcast(cv, lk);

  lock_release(lk);

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);
  as_deactivate();
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
  as = curproc_setas(NULL);
  as_destroy(as);

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */
  proc_remthread(curthread);

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);
  
  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
  *retval = curproc->pid;
  return(0);
}

/* stub handler for waitpid() system call                */
int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{

  int exitstatus;
  int result;

  /* this is just a stub implementation that always reports an
     exit status of 0, regardless of the actual exit status of
     the specified process.   
     In fact, this will return 0 even if the specified process
     is still running, and even if it never existed in the first place.

     Fix this!
  */

  if (options != 0) {
    return(EINVAL);
  }

  lock_acquire(lk);

  bool found = false;
  for (size_t i = 0; i < array_num(procTable); i++) {
    struct process *p = array_get(procTable, i);
    if (p->pid == pid && p->parent == curproc) {
      if (p->exited) {
        exitstatus = p->exitcode;
        found = true;
        kfree(p);
        array_remove(procTable, i);
      } else {
        cv_wait(cv, lk);
        i = -1;
      }
    }
  }
  KASSERT(found);

  lock_release(lk);

  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;

  return(0);
}

#if OPT_A2
int sys_fork(struct trapframe *tf, pid_t *retval) {
  struct proc *child = proc_create_runprogram(curproc->p_name);
  if (child == NULL) {
    return(EMPROC);
  }
  struct addrspace *as;
  int ret = as_copy(curproc->p_addrspace, &as);
  if (ret != 0) return ret;

  spinlock_acquire(&child->p_lock);
  child->p_addrspace = as;
  spinlock_release(&child->p_lock);

  struct process *p = kmalloc(sizeof(struct process));
  p->pid = child->pid;
  p->exited = 0;
  p->parent = curproc;
  unsigned r;

  lock_acquire(lk);
  array_add(procTable, p, &r);
  lock_release(lk);

  struct trapframe *ctf = kmalloc(sizeof(struct trapframe));
  *ctf = *tf;

  thread_fork(child->p_name, child, enter_forked_process, ctf, 0);

  *retval = child->pid;

  return 0;
}

int sys_execv(userptr_t progname, userptr_t args) {
  // Copy program name into kernel
  char *progn = kmalloc(strlen((char*)progname)+1);
  strcpy(progn, (char*)progname);

  // Count number of arguments
  int nargs = 0;
  while (((char**)args)[nargs] != NULL) {
    nargs++;
  }

  // Copy arguments into kernel
  char **a = kmalloc((nargs+1)*sizeof(*a));
  for (int i = 0; i < nargs; i++) {
    char *s = kmalloc(strlen(((char**)args)[i])+1);
    strcpy(s, ((char**)args)[i]);
    a[i] = s;
  }
  a[nargs] = NULL;

  struct addrspace *as, *oldas;
	struct vnode *v;
	vaddr_t entrypoint, stackptr;
	int result;

	/* Open the file. */
	result = vfs_open(progn, O_RDONLY, 0, &v);
	if (result) {
		return result;
	}

  /* If execv was called, get old address space to delete later */
  oldas = curproc_getas();

	/* Create a new address space. */
	as = as_create();
	if (as == NULL) {
		vfs_close(v);
		return ENOMEM;
	}

	/* Switch to it and activate it. */
	curproc_setas(as);
	as_activate();

	/* Load the executable. */
	result = load_elf(v, &entrypoint);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		vfs_close(v);
		return result;
	}

	/* Done with the file now. */
	vfs_close(v);

	/* Define the user stack in the address space */
	result = as_define_stack(as, &stackptr);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		return result;
	}

  /* Copy arguments into new address space */
  int argoffset[nargs];
  int sizeargs = 0;
  for (int i = 0; i < nargs; i++) {
    sizeargs += ROUNDUP(strlen(a[i])+1, 8);
    copyoutstr(a[i], (userptr_t)stackptr - sizeargs, strlen(a[i])+1, NULL);
    argoffset[i] = sizeargs;
    kfree(a[i]);
  }
  if (sizeargs > ARG_MAX) {
    return E2BIG;
  }

  userptr_t argv = (userptr_t)stackptr;
  argv = argv - sizeargs - 4*(nargs+1);

  for (int i = 0; i < nargs; i++) {
    ((char**)argv)[i] = (char*)stackptr - argoffset[i];
  }
  ((char**)argv)[nargs] = NULL;

  /* Delete old address space */
  as_destroy(oldas);

	/* Warp to user mode. */
	enter_new_process(nargs /*argc*/, argv /*userspace addr of argv*/,
			  (vaddr_t) argv, entrypoint);

	/* enter_new_process does not return. */
	panic("enter_new_process returned\n");
	return EINVAL;
}

#endif /* OPT_A2 */
