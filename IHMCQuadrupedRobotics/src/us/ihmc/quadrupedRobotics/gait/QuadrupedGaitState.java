package us.ihmc.quadrupedRobotics.gait;

import static us.ihmc.quadrupedRobotics.gait.QuadrupedGaitQuadrantState.SUPPORT;
import static us.ihmc.quadrupedRobotics.gait.QuadrupedGaitQuadrantState.SWING;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public enum QuadrupedGaitState
{
   FLIGHT(SWING, SWING, SWING, SWING),
   FRONT_LEFT_ONLY(SUPPORT, SWING, SWING, SWING),
   FRONT_RIGHT_ONLY(SWING, SUPPORT, SWING, SWING),
   HIND_RIGHT_ONLY(SWING, SWING, SUPPORT, SWING),
   HIND_LEFT_ONLY(SWING, SWING, SWING, SUPPORT),
   BOUND_FRONT(SUPPORT, SUPPORT, SWING, SWING),
   BOUND_HIND(SWING, SWING, SUPPORT, SUPPORT),
   PACE_RIGHT(SWING, SUPPORT, SUPPORT, SWING),
   PACE_LEFT(SUPPORT, SWING, SWING, SUPPORT),
   TROT_RIGHT(SWING, SUPPORT, SWING, SUPPORT),
   TROT_LEFT(SUPPORT, SWING, SUPPORT, SWING),
   WALK_FRONT_LEFT(SWING, SUPPORT, SUPPORT, SUPPORT),
   WALK_FRONT_RIGHT(SUPPORT, SWING, SUPPORT, SUPPORT),
   WALK_HIND_RIGHT(SUPPORT, SUPPORT, SWING, SUPPORT),
   WALK_HIND_LEFT(SUPPORT, SUPPORT, SUPPORT, SWING),
   ALL_FOURS(SUPPORT, SUPPORT, SUPPORT, SUPPORT),
   ;
   
   private final RobotQuadrant[] swingQuadrants;
   private final RobotQuadrant[] supportQuadrants;
   
   QuadrupedGaitState(QuadrupedGaitQuadrantState frontLeftFootState, QuadrupedGaitQuadrantState frontRightFootState, QuadrupedGaitQuadrantState hindRightFootState, QuadrupedGaitQuadrantState hindLeftFootState)
   {
      int numSupport = 0;
      
      if (frontLeftFootState == SUPPORT)
      {
         ++numSupport;
      }
      if (frontRightFootState == SUPPORT)
      {
         ++numSupport;
      }
      if (hindRightFootState == SUPPORT)
      {
         ++numSupport;
      }
      if (hindLeftFootState == SUPPORT)
      {
         ++numSupport;
      }
      
      int numSwing = 4 - numSupport;
      
      swingQuadrants = new RobotQuadrant[numSwing];
      supportQuadrants = new RobotQuadrant[numSupport];
      
      int i = 0;
      if (frontLeftFootState == SWING)
      {
         swingQuadrants[i++] = RobotQuadrant.FRONT_LEFT;
      }
      if (frontRightFootState == SWING)
      {
         swingQuadrants[i++] = RobotQuadrant.FRONT_RIGHT;
      }
      if (hindRightFootState == SWING)
      {
         swingQuadrants[i++] = RobotQuadrant.HIND_RIGHT;
      }
      if (hindLeftFootState == SWING)
      {
         swingQuadrants[i++] = RobotQuadrant.HIND_LEFT;
      }
      
      i = 0;
      if (frontLeftFootState == SUPPORT)
      {
         supportQuadrants[i++] = RobotQuadrant.FRONT_LEFT;
      }
      if (frontRightFootState == SUPPORT)
      {
         supportQuadrants[i++] = RobotQuadrant.FRONT_RIGHT;
      }
      if (hindRightFootState == SUPPORT)
      {
         supportQuadrants[i++] = RobotQuadrant.HIND_RIGHT;
      }
      if (hindLeftFootState == SUPPORT)
      {
         supportQuadrants[i++] = RobotQuadrant.HIND_LEFT;
      }
   }
   
   public RobotQuadrant[] swingQuadrants()
   {
      return swingQuadrants;
   }
   
   public RobotQuadrant[] supportQuadrants()
   {
      return supportQuadrants;
   }
}
