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
static struct cv *intersectionEmpty;

static struct lock *intersectionLock;

static struct array *origins;
static struct array *destinations;

static Direction *N;
static Direction *W;
static Direction *S;
static Direction *E;

static int numWaiting;

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

  intersectionEmpty = cv_create("intersectionEmpty");
  if (intersectionEmpty == NULL) {
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

  N = kmalloc(sizeof(*N));
  W = kmalloc(sizeof(*W));
  S = kmalloc(sizeof(*S));
  E = kmalloc(sizeof(*E));

  *N = north;
  *W = west;
  *S = south;
  *E = east;

  numWaiting = 0;

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
  KASSERT(intersectionCv != NULL);
  KASSERT(intersectionEmpty != NULL);
  KASSERT(intersectionLock != NULL);
  KASSERT(origins != NULL);
  KASSERT(destinations != NULL);

  cv_destroy(intersectionCv);
  cv_destroy(intersectionEmpty);

  lock_destroy(intersectionLock);

  array_destroy(origins);
  array_destroy(destinations);

  kfree(N);
  kfree(W);
  kfree(S);
  kfree(E);
}

static Direction* dirPtr(Direction d) {
  if (d == north) return N;
  if (d == west) return W;
  if (d == south) return S;
  if (d == east) return E;
  return NULL;
}

/*
 * Checks if it is a right turn
 */
static bool isRightTurn(Direction origin, Direction destination) {
  if (origin == north && destination == west) return true;
  if (origin == west && destination == south) return true;
  if (origin == south && destination == east) return true;
  if (origin == east && destination == north) return true;
  return false;
}

/*
 * Checks if there's a conflict
 */
static bool conflict(Direction o1, Direction d1, Direction o2, Direction d2) {
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

  // Wait for waiting cars to finish first
  if (numWaiting > 0) {
    cv_wait(intersectionEmpty, intersectionLock);
  }

  for (size_t i = 0; i < array_num(origins); i++) {
    Direction o = *(Direction *) array_get(origins, i);
    Direction d = *(Direction *) array_get(destinations, i);

    if (conflict(origin, destination, o, d)) {
      numWaiting++;
      cv_wait(intersectionCv, intersectionLock);
      numWaiting--;
      i = -1;
    }
  }

  // NO CONFLICT -- SUCCESSFUL

  unsigned ret;
  array_add(origins, dirPtr(origin), &ret);
  array_add(destinations, dirPtr(destination), &ret);

  // If no cars are waiting, allow old cars to enter
  if (numWaiting == 0) {
    cv_broadcast(intersectionEmpty, intersectionLock);
  }

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

  for (size_t i = 0; i < array_num(origins); i++) {
    Direction o = *(Direction *) array_get(origins, i);
    Direction d = *(Direction *) array_get(destinations, i);

    if (o == origin && d == destination) {
      array_remove(origins, i);
      array_remove(destinations, i);
      success = true;
      break;
    }
  }
  KASSERT(success == true);

  cv_broadcast(intersectionCv, intersectionLock);
  lock_release(intersectionLock);
}
