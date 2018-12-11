package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commons.MathTools;
import us.ihmc.quadrupedRobotics.planning.ContactState;

public class CoMTrajectoryPlannerTools
{
   static double getFirstCoefficientPositionMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return Math.min(Double.MAX_VALUE, Math.exp(omega * timeInPhase));
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
         return Math.min(Double.MAX_VALUE, omega * Math.exp(omega * timeInPhase));
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
         return -omega * getSecondCoefficientPositionMultiplier(contactState, timeInPhase, omega);
      }
      else
      {
         return 0.0;
      }
   }

   static double getFirstCoefficientAccelerationMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return Math.min(Double.MAX_VALUE, MathTools.square(omega) * Math.exp(omega * timeInPhase));
      }
      else
      {
         return 1.0;
      }
   }

   static double getSecondCoefficientAccelerationMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return MathTools.square(omega) * getSecondCoefficientPositionMultiplier(contactState, timeInPhase, omega);
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
         return -0.5 * gravityZ * MathTools.square(timeInPhase);
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
         return -gravityZ * timeInPhase;
      }
   }

   static double getGravityAccelerationEffect(ContactState contactState, double gravityZ)
   {
      if (contactState == ContactState.IN_CONTACT)
      {
         return 0.0;
      }
      else
      {
         return -gravityZ;
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
