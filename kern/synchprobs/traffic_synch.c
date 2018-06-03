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
static struct array *indexQueue; // queue of vehicles not yet in intersection sorted from longest to least waiting time

static unsigned int vehicleCount = 0; // global vehicle counter which represents the number of vehicles encountered

// grid representing vehicles currently in the intersection
// grid[d1][d2] is the number of vehicles in the intersection with origin d1 and destination d2
// allows vehicles with the same origin and destination in the wait queue to be unblocked 
static int grid[4][4] = 
{
  {0,0,0,0},
  {0,0,0,0},
  {0,0,0,0},
  {0,0,0,0}
};

// puts origin, destination pairs into one structure so that they can be more easily managed
struct directionPair {
  Direction orig;
  Direction dest;
};

// given an origin and destination, determine if vehicle is making a right turn
bool right_turn(Direction origin, Direction destination);
bool right_turn(Direction origin, Direction destination) {
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
  intersectionQueue = array_create();
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
  KASSERT(intersectionLock != NULL);
  KASSERT(intersectionCV != NULL);
  
  lock_destroy(intersectionLock);
  cv_destroy(intersectionCV);
  
  array_destroy(indexQueue);
  array_destroy(intersectionQueue);
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
  
  // vehicleIndex is equal to the position of the current vehicle; i.e. vehicleIndex = n-1 means current vehicle is the n-th vehicle 
  unsigned int *vehicleIndex = kmalloc(sizeof(unsigned int));
  *vehicleIndex = vehicleCount;
  array_add(indexQueue, vehicleIndex, NULL);

  vehicleCount++;
 
  // create pair with both origin and destination to add to queue
  struct directionPair *dpair = kmalloc(sizeof(struct directionPair));
  dpair->orig = origin;
  dpair->dest = destination;
  
  // loop runs until it is safe for current vehicle to enter the intersection
  while (true) {
    // stop/block vehicle (i.e. prevent from entering intersection) until either:
    //   1. there is another vehicle in the intersection with the same origin and destination
    //   2. current vehicle is the first in the wait queue 
    while (grid[origin][destination] == 0 && *(unsigned int *)array_get(indexQueue, 0) != *vehicleIndex) {
      cv_wait(intersectionCV, intersectionLock);
    }

    // if there is a vehicle in intersection with the same origin and destination, it is safe to enter the intersection  
    if (grid[origin][destination] != 0) {
      break;
    }   
    
    // otherwise, we check if the vehicle may collide with any vehicle in the intersection 
    bool valid = true;
    
    for (unsigned int i=0; i<array_num(intersectionQueue); i++){
      struct directionPair *vehiclePair = (struct directionPair*)array_get(intersectionQueue, i);
      Direction vOrigin = vehiclePair->orig;
      Direction vDest = vehiclePair->dest;
      
      if (origin == vOrigin) {
        continue;
      }
      else if (origin == vDest && destination == vOrigin) { 
        continue;
      }
      else if (destination != vDest && (right_turn(vOrigin, vDest) || right_turn(origin, destination))) {
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
  
  // remove current vehicle from wait queue
  for (unsigned int i=0; i<array_num(indexQueue); i++){
    if (vehicleIndex == (unsigned int *)array_get(indexQueue, i)) {
      array_remove(indexQueue, i);
      break;
    }
  }
  
  // add current vehicle to intersection queue, and change grid accordingly
  array_add(intersectionQueue, dpair, NULL);
  grid[origin][destination]++;

  // alert all other waiting vehicles that another vehicle has entered the queue
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
  lock_acquire(intersectionLock);

  // vehicles enter and leave in the same order, so we pop off the first vehicle in intersection queue
  array_remove(intersectionQueue, 0);
  grid[origin][destination]--;

  // broadcast to all waiting vehicles if intersection is empty
  if (array_num(intersectionQueue) == 0){
    cv_broadcast(intersectionCV, intersectionLock);
  }
  lock_release(intersectionLock);
}
