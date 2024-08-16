package us.ihmc.footstepPlanning.monteCarloPlanning;

public class MonteCarloPlannerConstants
{
   static int REWARD_GOAL = 20000000;
   static int REWARD_COVERAGE = 500;
   static int REWARD_SAFE_DISTANCE = 100;
   static int REWARD_DISTANCE_FROM_AVERAGE_POSITION = 4;

   static int PENALTY_COLLISION_OBSTACLE = 500;
   static int PENALTY_COLLISION_BOUNDARY = 5000000;
   static int PENALTY_PROXIMITY_OBSTACLE = 300;

   static byte OCCUPANCY_UNKNOWN = 0;
   static byte OCCUPANCY_FREE = 50;
   static byte OCCUPIED = 100;

   static float OCCUPANCY_MIN_THRESHOLD_HEIGHT_IN_METERS = 0.5f;
}
