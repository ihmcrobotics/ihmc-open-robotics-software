package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

public class LinearCoMTrajectoryPlannerTools
{

   static double getFirstCoefficientPositionMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState.isLoadBearing())
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
      if (contactState.isLoadBearing())
      {
         return Math.exp(-omega * timeInPhase);
      }
      else
      {
         return 1.0;
      }
   }

   static double getThirdCoefficientPositionMultiplier(ContactState contactState, double timeInPhase)
   {
      if (contactState.isLoadBearing())
      {
         return timeInPhase;
      }
      else
      {
         throw new IllegalArgumentException("Flight only has two coefficients.");
      }
   }

   static double getFourthCoefficientPositionMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 1.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight only has two coefficients.");
      }
   }

   static double getFirstCoefficientVelocityMultiplier(ContactState contactState, double timeInPhase, double omega)
   {
      if (contactState.isLoadBearing())
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
      if (contactState.isLoadBearing())
      {
         return -omega * getSecondCoefficientPositionMultiplier(contactState, timeInPhase, omega);
      }
      else
      {
         return 0.0;
      }
   }

   static double getThirdCoefficientVelocityMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 1.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight only has two coefficients.");
      }
   }

   static double getFourthCoefficientVelocityMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 0.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight only has two coefficients.");
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

   static double getThirdCoefficientAccelerationMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 0.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight only has two coefficients.");
      }
   }

   static double getFourthCoefficientAccelerationMultiplier(ContactState contactState)
   {
      if (contactState.isLoadBearing())
      {
         return 0.0;
      }
      else
      {
         throw new IllegalArgumentException("Flight only has two coefficients.");
      }
   }

   static double getGravityPositionEffect(ContactState contactState, double timeInPhase, double gravityZ)
   {
      if (contactState.isLoadBearing())
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
      if (contactState.isLoadBearing())
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
      if (contactState.isLoadBearing())
      {
         return 0.0;
      }
      else
      {
         return -gravityZ;
      }
   }
}
