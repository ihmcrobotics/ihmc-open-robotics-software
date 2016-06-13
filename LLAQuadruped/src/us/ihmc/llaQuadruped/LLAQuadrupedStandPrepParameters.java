package us.ihmc.llaQuadruped;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionStandPrepControllerParameters;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class LLAQuadrupedStandPrepParameters implements QuadrupedPositionStandPrepControllerParameters
{
   private static final Vector3d INITIAL_POSITION = new Vector3d(-0.17853, -0.05263, 0.37);
   private static final double HIP_ROLL_ANGLE = 0.0;
   private static final double HIP_PITCH_ANGLE = 0.3;
   private static final double KNEE_PITCH_ANGLE = 0.6;
   
   public double getInitialJointPosition(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      if (robotQuadrant == RobotQuadrant.FRONT_LEFT && legJointName == LegJointName.HIP_ROLL)
         return HIP_ROLL_ANGLE;
      if (robotQuadrant == RobotQuadrant.FRONT_LEFT && legJointName == LegJointName.HIP_PITCH)
         return HIP_PITCH_ANGLE;
      if (robotQuadrant == RobotQuadrant.FRONT_LEFT && legJointName == LegJointName.KNEE)
         return -KNEE_PITCH_ANGLE;
      if (robotQuadrant == RobotQuadrant.FRONT_RIGHT && legJointName == LegJointName.HIP_ROLL)
         return HIP_ROLL_ANGLE;
      if (robotQuadrant == RobotQuadrant.FRONT_RIGHT && legJointName == LegJointName.HIP_PITCH)
         return -HIP_PITCH_ANGLE;
      if (robotQuadrant == RobotQuadrant.FRONT_RIGHT && legJointName == LegJointName.KNEE)
         return KNEE_PITCH_ANGLE;
      if (robotQuadrant == RobotQuadrant.HIND_RIGHT && legJointName == LegJointName.HIP_ROLL)
         return HIP_ROLL_ANGLE;
      if (robotQuadrant == RobotQuadrant.HIND_RIGHT && legJointName == LegJointName.HIP_PITCH)
         return HIP_PITCH_ANGLE;
      if (robotQuadrant == RobotQuadrant.HIND_RIGHT && legJointName == LegJointName.KNEE)
         return -KNEE_PITCH_ANGLE;
      if (robotQuadrant == RobotQuadrant.HIND_LEFT && legJointName == LegJointName.HIP_ROLL)
         return HIP_ROLL_ANGLE;
      if (robotQuadrant == RobotQuadrant.HIND_LEFT && legJointName == LegJointName.HIP_PITCH)
         return -HIP_PITCH_ANGLE;
      if (robotQuadrant == RobotQuadrant.HIND_LEFT && legJointName == LegJointName.KNEE)
         return KNEE_PITCH_ANGLE;

      throw new RuntimeException();
   }
   
   public Vector3d getInitialPosition()
   {
      return INITIAL_POSITION;
   }
   
   @Override
   public double getInitialHeight()
   {
      return INITIAL_POSITION.getZ();
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
