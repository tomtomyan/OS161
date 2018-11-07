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

  for (size_t i = 0; i < array_num(procTable); i++) {
    struct process *p = array_get(procTable, i);
    if (p->pid == pid) {
      if (p->exited) {
        exitstatus = p->exitcode;
        kfree(p);
        array_remove(procTable, i);
      } else {
        cv_wait(cv, lk);
        i = 0;
      }
    }
  }

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
  array_add(procTable, p, &r);

  struct trapframe *ctf = kmalloc(sizeof(struct trapframe));
  *ctf = *tf;

  thread_fork(child->p_name, child, enter_forked_process, ctf, 0);

  *retval = child->pid;

  return 0;
}

int sys_execv(userptr_t progname, userptr_t args) {
  (void)progname;
  (void)args;
  return 0;
}

#endif /* OPT_A2 */
