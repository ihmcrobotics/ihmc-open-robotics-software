package us.ihmc.quadrupedRobotics.planning.gait;

import static us.ihmc.quadrupedRobotics.planning.ContactState.IN_CONTACT;
import static us.ihmc.quadrupedRobotics.planning.ContactState.NO_CONTACT;

import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public enum QuadrupedSupportConfiguration
{
   FLIGHT(NO_CONTACT, NO_CONTACT, NO_CONTACT, NO_CONTACT),
   FRONT_LEFT_ONLY(IN_CONTACT, NO_CONTACT, NO_CONTACT, NO_CONTACT),
   FRONT_RIGHT_ONLY(NO_CONTACT, IN_CONTACT, NO_CONTACT, NO_CONTACT),
   HIND_RIGHT_ONLY(NO_CONTACT, NO_CONTACT, IN_CONTACT, NO_CONTACT),
   HIND_LEFT_ONLY(NO_CONTACT, NO_CONTACT, NO_CONTACT, IN_CONTACT),
   BOUND_FRONT(IN_CONTACT, IN_CONTACT, NO_CONTACT, NO_CONTACT),
   BOUND_HIND(NO_CONTACT, NO_CONTACT, IN_CONTACT, IN_CONTACT),
   PACE_RIGHT(NO_CONTACT, IN_CONTACT, IN_CONTACT, NO_CONTACT),
   PACE_LEFT(IN_CONTACT, NO_CONTACT, NO_CONTACT, IN_CONTACT),
   TROT_RIGHT(NO_CONTACT, IN_CONTACT, NO_CONTACT, IN_CONTACT),
   TROT_LEFT(IN_CONTACT, NO_CONTACT, IN_CONTACT, NO_CONTACT),
   WALK_FRONT_LEFT(NO_CONTACT, IN_CONTACT, IN_CONTACT, IN_CONTACT),
   WALK_FRONT_RIGHT(IN_CONTACT, NO_CONTACT, IN_CONTACT, IN_CONTACT),
   WALK_HIND_RIGHT(IN_CONTACT, IN_CONTACT, NO_CONTACT, IN_CONTACT),
   WALK_HIND_LEFT(IN_CONTACT, IN_CONTACT, IN_CONTACT, NO_CONTACT),
   ALL_FOURS(IN_CONTACT, IN_CONTACT, IN_CONTACT, IN_CONTACT),
   ;
   
   private final QuadrantDependentList<ContactState> swingQuadrants = new QuadrantDependentList<>();
   private final QuadrantDependentList<ContactState> supportQuadrants = new QuadrantDependentList<>();
   
   QuadrupedSupportConfiguration(ContactState frontLeftFootState, ContactState frontRightFootState, ContactState hindRightFootState, ContactState hindLeftFootState)
   {
      if (frontLeftFootState == IN_CONTACT)
      {
         supportQuadrants.set(RobotQuadrant.FRONT_LEFT, IN_CONTACT);
      }
      else
      {
         swingQuadrants.set(RobotQuadrant.FRONT_LEFT, NO_CONTACT);
      }
      if (frontRightFootState == IN_CONTACT)
      {
         supportQuadrants.set(RobotQuadrant.FRONT_RIGHT, IN_CONTACT);
      }
      else
      {
         swingQuadrants.set(RobotQuadrant.FRONT_RIGHT, NO_CONTACT);
      }
      if (hindRightFootState == IN_CONTACT)
      {
         supportQuadrants.set(RobotQuadrant.HIND_RIGHT, IN_CONTACT);
      }
      else
      {
         swingQuadrants.set(RobotQuadrant.HIND_RIGHT, NO_CONTACT);
      }
      if (hindLeftFootState == IN_CONTACT)
      {
         supportQuadrants.set(RobotQuadrant.HIND_LEFT, IN_CONTACT);
      }
      else
      {
         swingQuadrants.set(RobotQuadrant.HIND_LEFT, NO_CONTACT);
      }
   }
   
   public ContactState getQuadrantState(RobotQuadrant robotQuadrant)
   {
      return isSupportQuadrant(robotQuadrant) ? IN_CONTACT : NO_CONTACT;
   }
   
   public boolean isSupportQuadrant(RobotQuadrant robotQuadrant)
   {
      return supportQuadrants.containsKey(robotQuadrant);
   }
   
   public boolean isSwingQuadrant(RobotQuadrant robotQuadrant)
   {
      return swingQuadrants.containsKey(robotQuadrant);
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
