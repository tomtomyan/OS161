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
  struct proc *parent = curproc->parent;

  KASSERT(p->children != NULL);
  KASSERT(p->cexitcodes != NULL);

  // remove it from children's parents
  for (size_t i = 0; i < array_num(p->children); i++) {
    struct proc *child = array_get(p->children, i);
    KASSERT(child != NULL);
    child->parent = NULL;
  }

  // check it's not NULL and not the kernel process
  if (parent && parent != (struct proc*) 0xdeadbeef) {
    KASSERT(parent->waitlock != NULL);
    lock_acquire(parent->waitlock);

    // remove it from parent
    for (size_t i = 0; i < array_num(parent->children); i++) {
      struct proc *child = array_get(parent->children, i);
      KASSERT(child != NULL);
      if (p->pid == child->pid) {
        array_remove(parent->children, i);
        break;
      }
    }

    struct cexitcodes *cec = kmalloc(sizeof(struct cexitcodes));
    cec->pid = p->pid;
    cec->exitcode = _MKWAIT_EXIT(exitcode);

    unsigned ret;
    array_add(parent->cexitcodes, cec, &ret);

    cv_broadcast(parent->waitcv, parent->waitlock);

    lock_release(parent->waitlock);
  }

  for (size_t i = 0; i < array_num(p->cexitcodes); i++) {
    struct cexitcodes *cec = array_get(p->cexitcodes, i);
    kfree(cec);
  }

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
  struct proc *p = curproc;

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

  // wait if child hasn't exited
  lock_acquire(p->waitlock);
  for (size_t i = 0; i < array_num(p->children); i++) {
    struct proc *child = array_get(p->children, i);
    if (pid == child->pid) {
      cv_wait(p->waitcv, p->waitlock);
      break;
    }
  }

  // get the exit status
  for (size_t i = 0; i < array_num(p->cexitcodes); i++) {
    struct cexitcodes *cec = array_get(p->cexitcodes, i);
    if (pid == cec->pid) {
      exitstatus = cec->exitcode;
      kfree(cec);
      array_remove(p->cexitcodes, i);
      break;
    }
  }
  lock_release(p->waitlock);

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

  child->parent = curproc;
  unsigned temp;
  array_add(curproc->children, child, &temp);

  struct trapframe *ctf = kmalloc(sizeof(struct trapframe));
  *ctf = *tf;

  thread_fork(child->p_name, child, enter_forked_process, ctf, 0);

  *retval = child->pid;

  return 0;
}
#endif /* OPT_A2 */
