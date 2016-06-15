package us.ihmc.llaQuadruped;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.controller.position.states.QuadrupedPositionStandPrepControllerParameters;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class LLAQuadrupedStandPrepParameters implements QuadrupedPositionStandPrepControllerParameters
{
   private static final Vector3d INITIAL_POSITION = new Vector3d(0.0, 0.0, 0.37);
   private static final double HIP_ROLL_ANGLE = 0.0;
   private static final double HIP_PITCH_ANGLE = 0.3;
   private static final double KNEE_PITCH_ANGLE = 0.6;
   
   public double getInitialJointPosition(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      if (robotQuadrant == RobotQuadrant.FRONT_LEFT && legJointName == LegJointName.HIP_ROLL)
         return HIP_ROLL_ANGLE;
      if (robotQuadrant == RobotQuadrant.FRONT_LEFT && legJointName == LegJointName.HIP_PITCH)
         return -HIP_PITCH_ANGLE;
      if (robotQuadrant == RobotQuadrant.FRONT_LEFT && legJointName == LegJointName.KNEE)
         return KNEE_PITCH_ANGLE;
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
         return HIP_PITCH_ANGLE;
      if (robotQuadrant == RobotQuadrant.HIND_LEFT && legJointName == LegJointName.KNEE)
         return -KNEE_PITCH_ANGLE;

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
      if (joint.name().toLowerCase().contains("hip_roll"))
      {
         return getInitialJointPosition(joint.getQuadrant(), LegJointName.HIP_ROLL);
      }
      else if (joint.name().toLowerCase().contains("hip_pitch"))
      {
         return getInitialJointPosition(joint.getQuadrant(), LegJointName.HIP_PITCH);
      }
      else
      {
         return getInitialJointPosition(joint.getQuadrant(), LegJointName.KNEE);
      }
   }
}
