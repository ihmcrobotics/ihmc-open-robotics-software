package us.ihmc.quadrupedRobotics.planning.gait;

import static us.ihmc.quadrupedRobotics.planning.gait.QuadrupedFootContactState.SUPPORT;
import static us.ihmc.quadrupedRobotics.planning.gait.QuadrupedFootContactState.SWING;

import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public enum QuadrupedSupportConfiguration
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
   
   private final QuadrantDependentList<QuadrupedFootContactState> swingQuadrants = new QuadrantDependentList<>();
   private final QuadrantDependentList<QuadrupedFootContactState> supportQuadrants = new QuadrantDependentList<>();
   
   QuadrupedSupportConfiguration(QuadrupedFootContactState frontLeftFootState, QuadrupedFootContactState frontRightFootState, QuadrupedFootContactState hindRightFootState, QuadrupedFootContactState hindLeftFootState)
   {
      if (frontLeftFootState == SUPPORT)
      {
         supportQuadrants.set(RobotQuadrant.FRONT_LEFT, SUPPORT);
      }
      else
      {
         swingQuadrants.set(RobotQuadrant.FRONT_LEFT, SWING);
      }
      if (frontRightFootState == SUPPORT)
      {
         supportQuadrants.set(RobotQuadrant.FRONT_RIGHT, SUPPORT);
      }
      else
      {
         swingQuadrants.set(RobotQuadrant.FRONT_RIGHT, SWING);
      }
      if (hindRightFootState == SUPPORT)
      {
         supportQuadrants.set(RobotQuadrant.HIND_RIGHT, SUPPORT);
      }
      else
      {
         swingQuadrants.set(RobotQuadrant.HIND_RIGHT, SWING);
      }
      if (hindLeftFootState == SUPPORT)
      {
         supportQuadrants.set(RobotQuadrant.HIND_LEFT, SUPPORT);
      }
      else
      {
         swingQuadrants.set(RobotQuadrant.HIND_LEFT, SWING);
      }
   }
   
   public QuadrupedFootContactState getQuadrantState(RobotQuadrant robotQuadrant)
   {
      return isSupportQuadrant(robotQuadrant) ? SUPPORT : SWING;
   }
   
   public boolean isSupportQuadrant(RobotQuadrant robotQuadrant)
   {
      return supportQuadrants.containsQuadrant(robotQuadrant);
   }
   
   public boolean isSwingQuadrant(RobotQuadrant robotQuadrant)
   {
      return swingQuadrants.containsQuadrant(robotQuadrant);
   }
   
   public RobotQuadrant[] swingQuadrants()
   {
      return swingQuadrants.quadrants();
   }
   
   public RobotQuadrant[] supportQuadrants()
   {
      return supportQuadrants.quadrants();
   }
}
