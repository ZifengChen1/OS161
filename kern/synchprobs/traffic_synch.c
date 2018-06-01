#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>
#include <thread.h>
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
static struct lock *intersectionLock;
static struct cv *intersectionCV;

static struct array *intersectionQueue;
static struct array *waitQueue;

static struct array *indexQueue;

static int vehicleCount; // global vehicle counter which represents the number of vehicles encountered
static int index = 0;

static const int maxSize = 8; // maximum number of vehicles allowed in intersection before dealing with waiting vehiicles

struct directionPair {
  Direction orig;
  Direction dest;
};

// given an origin and destination, determine if vehicle is making a right turn
bool
right_turn(Direction origin, Direction destination) {
  if ((origin == west && destination == south) ||
      (origin == south && destination == east) ||
      (origin == east && destination == north) ||
      (origin == north && destination == west)) {
    return true;
  }
  return false;
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
  /* replace this default implementation with your own implementation */
  intersectionQueue = array_create();
  waitQueue = array_create();
  indexQueue = array_create();  

  intersectionLock = lock_create("intersectionLock");
  if (intersectionLock == NULL) {
    panic("could not create intersection lock");
  } 
  intersectionCV = cv_create("intersectionCV");
  if (intersectionCV == NULL) {
    panic("could not create intersection cv)");
  }

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
  KASSERT(intersectionLock != NULL);
  KASSERT(intersectionCV != NULL);
  
  lock_destroy(intersectionLock);
  cv_destroy(intersectionCV);
  
  array_destroy(intersectionQueue);
  array_destroy(waitQueue);
  array_destroy(indexQueue);
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
  lock_acquire(intersectionLock); // prevent race condition with queue reads and writes
  
  int vehicleIndex = vehicleCount;
  vehicleCount++;
  
  // create pair with both origin and destination to add to queue
  struct directionPair dpair;
  dpair.orig = origin;
  dpair.dest = destination;

  array_add(waitQueue, &dpair, NULL);
  
  if (array_num(indexQueue) < maxSize) {
    array_add(indexQueue, vehicleIndex, NULL);
  }

  while (true) {
    while (*array_get(indexQueue, index) != vehicleNumber) {
      cv_wait(intersectionCV, intersectionLock);
    }
    
    if (index < maxSize && index < array_num(indexQueue)) {
       index += 1;
    }

    bool valid = true;
    
    // check if vehicle can go in intersection
    for (int i=0; i<array_num(intersectionQueue); i++){
      vOrigin = array_get(intersectionQueue, i)->orig;
      vDest = array_get(intersectionQueue, i)->dest;
      
      if (origin == vOrigin) {
        continue;
      }
      else if (origin == vDest && destination == vOrigin) { 
        continue;
      }
      else if (dest != vDest && (right_turn(vOrigin, vDest) || right_turn(origin, destination))) {
        continue;
      }
      else {
        valid = false;
        break;
      }
    }
   
    if (valid) {
      break;
    }
  }

  array_add(indexQueue, array_get(array_num(indexQueue) - 1) + 1, NULL); 
  
  for (int i=0; i<array_num(indexQueue); i++) {
    if (array_get(indexQueue, i) == vehicleIndex) {
      array_remove(indexQueue, i);
      break;
    }
  }

  array_add(intersectionQueue, &dpair, NULL);
  cv_broadcast(intersectionCV, intersectionLock); 
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
  /* replace this default implementation with your own implementation */
  (void)origin;  /* avoid compiler complaint about unused parameter */
  (void)destination; /* avoid compiler complaint about unused parameter */
  KASSERT(array_num(intersectionQueue) > 0);
  array_remove(intersectionQueue, 0);
  index = 0;
  if (array_num(intersectionQueue) == 0){
    cv_broadcast(intersectionCV, intersectionLock);
  }
}
