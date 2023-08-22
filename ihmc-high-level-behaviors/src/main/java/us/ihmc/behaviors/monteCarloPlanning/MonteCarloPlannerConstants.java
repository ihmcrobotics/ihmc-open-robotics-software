package us.ihmc.behaviors.monteCarloPlanning;

public class MonteCarloPlannerConstants
{
   static int REWARD_GOAL = 20000000;
   static int REWARD_COVERAGE = 500;
   static int REWARD_SAFE_DISTANCE = 50;

   static int PENALTY_COLLISION_OBSTACLE = 400;
   static int PENALTY_COLLISION_BOUNDARY = 1000;
   static int PENALTY_PROXIMITY_OBSTACLE = 200;

   static byte OCCUPANCY_UNKNOWN = 0;
   static byte OCCUPANCY_FREE = 50;
   static byte OCCUPIED = 100;

   static float OCCUPANCY_MIN_THRESHOLD_HEIGHT_IN_METERS = 0.5f;
}
