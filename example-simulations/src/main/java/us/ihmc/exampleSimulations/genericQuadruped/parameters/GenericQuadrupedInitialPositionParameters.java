package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.quadrupedRobotics.model.QuadrupedSimulationInitialPositionParameters;
import us.ihmc.robotics.partNames.QuadrupedJointName;

public abstract class GenericQuadrupedInitialPositionParameters implements QuadrupedSimulationInitialPositionParameters
{
   private static final boolean INVERT_REAR_LEGS = true;

   @Override
   public double getInitialJointPosition(QuadrupedJointName joint)
   {
      double directionalMultiplier;
      if (INVERT_REAR_LEGS)
         directionalMultiplier = -1.0;
      else
         directionalMultiplier = 1.0;

      switch (joint)
      {
      case FRONT_LEFT_HIP_ROLL:
      case HIND_LEFT_HIP_ROLL:
         return getHipRollAngle();
      case FRONT_RIGHT_HIP_ROLL:
      case HIND_RIGHT_HIP_ROLL:
         return - getHipRollAngle();
      case FRONT_LEFT_HIP_PITCH:
      case FRONT_RIGHT_HIP_PITCH:
         return getHipPitchAngle();
      case HIND_LEFT_HIP_PITCH:
      case HIND_RIGHT_HIP_PITCH:
         return directionalMultiplier * getHipPitchAngle();
      case FRONT_LEFT_KNEE_PITCH:
      case FRONT_RIGHT_KNEE_PITCH:
         return getKneePitchAngle();
      case HIND_LEFT_KNEE_PITCH:
      case HIND_RIGHT_KNEE_PITCH:
         return directionalMultiplier * getKneePitchAngle();
      }

      throw new RuntimeException(joint + " not defined!");
   }

   abstract double getHipRollAngle();
   abstract double getHipPitchAngle();
   abstract double getKneePitchAngle();
}
