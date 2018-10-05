#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>
#include <array.h>


/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */
static struct cv *intersectionCv;
static struct lock *intersectionLock;

static struct array *origins;
static struct array *destinations;

static int intersectionNum;

void
printD(Direction o, Direction d) {
  switch (o)
    {
    case north:
      kprintf("North");
      break;
    case east:
      kprintf("East");
      break;
    case south:
      kprintf("South");
      break;
    case west:
      kprintf("West");
      break;
    }
  kprintf(" to ");
  switch (d)
    {
    case north:
      kprintf("North");
      break;
    case east:
      kprintf("East");
      break;
    case south:
      kprintf("South");
      break;
    case west:
      kprintf("West");
      break;
    }
}    

/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  intersectionCv = cv_create("intersectionCv");
  if (intersectionCv == NULL) {
    panic("could not create intersection condition variable");
  }

  intersectionLock = lock_create("intersectionLock");
  if (intersectionLock == NULL) {
    panic("could not create intersection lock");
  }

  origins = array_create();
  if (origins == NULL) {
    panic("could not create origins array");
  }

  destinations = array_create();
  if (destinations == NULL) {
    panic("could not create destinations array");
  }

  array_init(origins);
  array_init(destinations);

  array_setsize(origins, 10);
  array_setsize(destinations, 10);

  intersectionNum = 0;

  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  /* replace this default implementation with your own implementation */
  KASSERT(intersectionCv != NULL);
  KASSERT(intersectionLock != NULL);
  KASSERT(origins != NULL);
  KASSERT(destinations != NULL);

  cv_destroy(intersectionCv);
  lock_destroy(intersectionLock);

  array_cleanup(origins);
  array_cleanup(destinations);

  array_destroy(origins);
  array_destroy(destinations);
}

/*
 * Checks if it is a right turn
 */
bool isRightTurn(Direction origin, Direction destination) {
  if (origin == north && destination == west) return true;
  if (origin == west && destination == south) return true;
  if (origin == south && destination == east) return true;
  if (origin == east && destination == north) return true;
  return false;
}

/*
 * Checks if there's a conflict
 */
bool conflict(Direction o1, Direction d1, Direction o2, Direction d2) {
  if (o1 == o2) return false;
  if (o1 == d2 && o2 == d1) return false;
  if (d1 != d2 && (isRightTurn(o1, d1) || isRightTurn(o2, d2))) return false;
  return true;
}

/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{

  lock_acquire(intersectionLock);
  
  for (int i = 0; i < intersectionNum; i++) {
    //kprintf("Checking conflict\n");
    Direction o = *(Direction *) array_get(origins, i);
    Direction d = *(Direction *) array_get(destinations, i);

    if (conflict(origin, destination, o, d)) {
      //kprintf("Conflict found, waiting...\n");
      cv_wait(intersectionCv, intersectionLock);
      //kprintf("Woken up\n");
      i = -1;
    }
  }

  // NO CONFLICT -- SUCCESSFUL
  //kprintf("No conflict found\n");
  
  //kprintf("Entering intersection from ");
  //printD(origin, destination);
  //kprintf(", number of cars in intersection: %d\n", intersectionNum);

  //unsigned *ret;
  Direction *o = kmalloc(sizeof(*o));
  Direction *d = kmalloc(sizeof(*d));
  *o = origin;
  *d = destination;

  if ((int) array_num(origins) <= intersectionNum) {
    array_setsize(origins, 1 + array_num(origins) * 2);
    array_setsize(destinations, 1 + array_num(destinations) * 2);
  }
    
  array_set(origins, intersectionNum, o);
  array_set(destinations, intersectionNum, d);
  intersectionNum++;

  lock_release(intersectionLock);

}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{

  bool success = false;

  lock_acquire(intersectionLock);
  
  //kprintf("Exiting intersection from ");
  //printD(origin, destination);

  for (int i = 0; i < intersectionNum; i++) {
    Direction o = *(Direction *) array_get(origins, i);
    Direction d = *(Direction *) array_get(destinations, i);
    //printD(o, d);

    if (o == origin && d == destination) {
      intersectionNum--;
      kfree(array_get(origins, i));
      kfree(array_get(destinations, i));
      array_remove(origins, i);
      array_remove(destinations, i);
      //kprintf("Exit successful from ");
      //printD(origin, destination);
      //kprintf(", number of cars in intersection: %d\n", intersectionNum);
      success = true;
      break;
    }
  }
  KASSERT(success == true);

  cv_broadcast(intersectionCv, intersectionLock);
  lock_release(intersectionLock);

}
