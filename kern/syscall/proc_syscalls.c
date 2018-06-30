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
#include <syscall.h>
#include <mips/trapframe.h>
#include <array.h>
#include <vfs.h>
#include <kern/fcntl.h>

#include "opt-A2.h"

#if OPT_A2
// fork implementation 
int sys_fork(struct trapframe *tf, pid_t *retval) {
  // create child process
  struct proc *childproc = proc_create_runprogram(curproc->p_name);
  
  // if proc_create_program is NULL, then return non-zero exit code
  if (childproc == NULL) {
    return ENOMEM;  
  }
  
  int err = as_copy(curproc->p_addrspace, &childproc->p_addrspace);
  
  // if as_copy returns a non zero error code, return the error code 
  if (err != 0) {
    return err;
  }
  

 // childproc->pid = proc_getnewpid();

  struct trapframe *tfcopy = kmalloc(sizeof(struct trapframe));
  if (tfcopy == NULL) {
  	return ENOMEM;
  }  
  memcpy(tfcopy, tf, sizeof(struct trapframe));  

  err = thread_fork(curproc->p_name, childproc, &enter_forked_process, (void *)tfcopy, 1); 
  if (err != 0) {
    return err;    
  }
  
  array_add(curproc->children, childproc, NULL);
  childproc->ppid = curproc->pid;
  *retval = childproc->pid;
  return 0;
}
#endif /* OPT_A2 */

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {

  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */
  (void)exitcode;

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

#if OPT_A2
  //added code to pass exit code to parent
  proc_exitcode(p, exitcode);  
#endif /* OPT_A2 */

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
  
#if OPT_A2
  // destroy p's zombie children
  proc_destroyzombiechildren(p);  

  // if process has no active parent, destroy 
  if (!proc_hasparent(p)){
    proc_destroy(p);
  }

#endif /*OPT_A2*/

  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}

#if OPT_A2
int
sys_getpid(pid_t *retval)
{
  // simply return the pid of current process
  *retval = curproc->pid;
  return 0;
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
  // options not yet supported
  if (options != 0) {
   *retval = -1;
   return EINVAL;
  }
 
  // verify that waitpid is being called on child
  bool isChild = false;
  struct proc *childproc;
  for (unsigned int i = 0; i<array_num(curproc->children); i++) {
    struct proc *cproc = (struct proc *)array_get(curproc->children, i);
    if (cproc->pid == pid) {
      isChild = true;
      childproc = (struct proc*)array_get(curproc->children, i);
     
      // remove proc from children array, since we are done with it
      array_remove(curproc->children, i);
      break;
    }
  }   
  
  // verify that child is in child array
  if (!isChild) {
    *retval = -1;
    return ECHILD;
  }

  int exitstatus = proc_waitforchild(childproc);

  // destroy child after getting its exit status 
  proc_destroy(childproc);

  int result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    *retval = -1;
    return result;
  }
  
  
  *retval = pid;

  return 0;
}

int
sys_execv(const char *program, char **args) {  
  struct addrspace *as;
  struct vnode *v;
  vaddr_t entrypoint, stackptr;
  int result;


  // count number of arguments
  int argc = 0;
  while (args[argc] != NULL) {
    argc += 1;
  } 
 
  // Note that since argv is terminated by NULL, we use argc + 1 here
  char **progargs = (char **)kmalloc((argc + 1) * sizeof(char *));
  progargs[argc] = NULL;
  
  if (progargs == NULL) {
    return ENOMEM;
  }

  // copy arguments to kernel in variable progargs
  for (int i = 0; i < argc; i++){
    size_t argsize = (strlen(args[i]) + 1) * sizeof(char);
    progargs[i] = (char *)kmalloc(argsize);
    
    if (progargs[i] == NULL) {
      return ENOMEM;
    }
    
    result = copyinstr((const_userptr_t)args[i], progargs[i], argsize, NULL);

    if (result) {
      return result;
    }
  }
  
  // kprintf(progargs[0]);

  // copy program path into kernel
  size_t prognamesize = (strlen(program) + 1) * sizeof(char);
  char *progname = (char *)kmalloc(prognamesize);

  if (progname == NULL) {
    return ENOMEM;
  }
  
  result = copyinstr((const_userptr_t)program, progname, prognamesize, NULL);

  if (result) {
    return result;
  }
  // code from runprogram
  /* Open the file. */
  result = vfs_open(progname, O_RDONLY, 0, &v);
  if (result) {
     return result;
  }

  /* Create a new address space. */
  as = as_create();
  if (as ==NULL) {
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
  
  result = as_define_stack(as, &stackptr);
  if (result) {
    return result;
  }
  // end code from runprogram

  
  // pointers to each arg, used so we can store array
  vaddr_t *argptrs = (vaddr_t *)kmalloc((argc + 1) * sizeof(vaddr_t));
  if (argptrs == NULL) {
    return ENOMEM;
  }
  argptrs[argc] = (vaddr_t)NULL;
  
  // copy arguments onto userstack
  // since arguments are pushed onto stack from right to left, we start at argc - 1
  for (int i = argc - 1; i >= 0; i--) {
    size_t argsize = strlen(progargs[i]) + 1 * sizeof(char);
    argsize = ROUNDUP(argsize, 8);
    
    // allocate room for arg
    stackptr -= argsize;

    result = copyoutstr((const char *)progargs[i], (userptr_t)stackptr, argsize , NULL);
    
    if (result) {
      return result;
    }

    argptrs[i] = stackptr;
  }
  
  // copy array onto userstack
  for (int i = argc; i >= 0; i--) {
    size_t ptrsize = ROUNDUP(sizeof(vaddr_t), 4);
    stackptr -= ptrsize;
    
    result = copyout((const void *)&argptrs[i], (userptr_t)stackptr, ptrsize);
    
    if (result) {
      return result;
    }  
  }
  
  enter_new_process(argc, (userptr_t)stackptr, stackptr, entrypoint);

  panic("return from enter_new_process in sys_execv");
  return EINVAL;

}

#endif /* OPT_A2 */
