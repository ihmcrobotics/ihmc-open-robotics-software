package us.ihmc.llaQuadruped;

import us.ihmc.SdfLoader.partNames.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionStandPrepControllerParameters;

public class LLAQuadrupedStandPrepParameters implements QuadrupedPositionStandPrepControllerParameters
{
   private static final double INITIAL_HEIGHT = 0.38;
   
   @Override
   public double getInitialHeight()
   {
      return INITIAL_HEIGHT;
   }

   @Override
   public double getInitialPosition(QuadrupedJointName joint)
   {
      switch (joint)
      {
      case FRONT_LEFT_HIP_ROLL:
      case FRONT_RIGHT_HIP_ROLL:
      case HIND_LEFT_HIP_ROLL:
      case HIND_RIGHT_HIP_ROLL:
         return 0.0;
      case FRONT_LEFT_HIP_PITCH:
      case FRONT_RIGHT_HIP_PITCH:
         return 0.4;
      case HIND_LEFT_HIP_PITCH:
      case HIND_RIGHT_HIP_PITCH:
         return -0.6;
      case FRONT_LEFT_KNEE_PITCH:
      case FRONT_RIGHT_KNEE_PITCH:
         return -1.000;
      case HIND_LEFT_KNEE_PITCH:
      case HIND_RIGHT_KNEE_PITCH:
         return 1.00;
      case PROXIMAL_NECK_YAW:
      case PROXIMAL_NECK_PITCH:
      case DISTAL_NECK_YAW:
      case DISTAL_NECK_PITCH:
      case DISTAL_NECK_ROLL:
         return 0.0;
      default:
         throw new IllegalArgumentException();
      }
   }
}
