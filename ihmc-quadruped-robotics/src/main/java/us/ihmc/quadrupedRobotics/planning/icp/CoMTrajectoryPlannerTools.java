package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.quadrupedRobotics.planning.ContactState;

public class CoMTrajectoryPlannerTools
{
   static double getFirstCoefficientPositionMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return Math.exp(omega * timeInPhase);
      }
      else
      {
         return timeInPhase;
      }
   }

   static double getSecondCoefficientPositionMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return Math.exp(-omega * timeInPhase);
      }
      else
      {
         return 1.0;
      }
   }

   static double getFirstCoefficientVelocityMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return omega * Math.exp(omega * timeInPhase);
      }
      else
      {
         return 1.0;
      }
   }

   static double getSecondCoefficientVelocityMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return -omega * Math.exp(-omega * timeInPhase);
      }
      else
      {
         return 0.0;
      }
   }

   static double getGravityPositionEffect(ContactState contactState, double timeInPhase, double gravityZ)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return 0.0;
      }
      else
      {
         return -gravityZ * timeInPhase * timeInPhase;
      }
   }

   static double getGravityVelocityEffect(ContactState contactState, double timeInPhase, double gravityZ)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return 0.0;
      }
      else
      {
         return -2.0 * gravityZ * timeInPhase;
      }
   }

   static int getFirstCoefficientIndex(int sequenceId)
   {
      return 2 * sequenceId;
   }

   static int getSecondCoefficientIndex(int sequenceId)
   {
      return 2 * sequenceId + 1;
   }
}
