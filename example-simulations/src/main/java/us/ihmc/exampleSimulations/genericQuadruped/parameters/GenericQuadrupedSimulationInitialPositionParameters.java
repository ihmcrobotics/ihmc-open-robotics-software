package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.model.QuadrupedSimulationInitialPositionParameters;
import us.ihmc.robotics.partNames.QuadrupedJointName;

public class GenericQuadrupedSimulationInitialPositionParameters implements QuadrupedSimulationInitialPositionParameters
{
   private static final Point3D INITIAL_BODY_POSITION = new Point3D(0.0, 0.0, 0.32);
   private static final double HIP_ROLL_ANGLE = 0.3;
   private static final double HIP_PITCH_ANGLE = 1.0;
   private static final double KNEE_PITCH_ANGLE = -1.7;

    private static final boolean INVERT_REAR_LEGS = true;
   
   @Override
   public Point3D getInitialBodyPosition()
   {
      return INITIAL_BODY_POSITION;
   }

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
         return HIP_ROLL_ANGLE;
      case FRONT_RIGHT_HIP_ROLL:
      case HIND_RIGHT_HIP_ROLL:
         return -HIP_ROLL_ANGLE;
      case FRONT_LEFT_HIP_PITCH:
      case FRONT_RIGHT_HIP_PITCH:
         return HIP_PITCH_ANGLE;
      case HIND_LEFT_HIP_PITCH:
      case HIND_RIGHT_HIP_PITCH:
         return directionalMultiplier * HIP_PITCH_ANGLE;
      case FRONT_LEFT_KNEE_PITCH:
      case FRONT_RIGHT_KNEE_PITCH:
         return KNEE_PITCH_ANGLE;
      case HIND_LEFT_KNEE_PITCH:
      case HIND_RIGHT_KNEE_PITCH:
         return directionalMultiplier * KNEE_PITCH_ANGLE;
      }

      throw new RuntimeException(joint + " not defined!");
   }
}
