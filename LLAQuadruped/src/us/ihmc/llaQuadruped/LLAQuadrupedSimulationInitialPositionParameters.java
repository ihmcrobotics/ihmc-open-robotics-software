package us.ihmc.llaQuadruped;

import javax.vecmath.Point3d;

import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.model.QuadrupedSimulationInitialPositionParameters;

public class LLAQuadrupedSimulationInitialPositionParameters implements QuadrupedSimulationInitialPositionParameters
{
   private static final Point3d INITIAL_BODY_POSITION = new Point3d(0.0, 0.0, 0.37);
   private static final double HIP_ROLL_ANGLE = 0.0;
   private static final double HIP_PITCH_ANGLE = 0.3;
   private static final double KNEE_PITCH_ANGLE = 0.6;
   
   @Override
   public Point3d getInitialBodyPosition()
   {
      return INITIAL_BODY_POSITION;
   }

   @Override
   public double getInitialJointPosition(QuadrupedJointName joint)
   {
      if (joint == QuadrupedJointName.FRONT_LEFT_HIP_ROLL)
         return HIP_ROLL_ANGLE;
      if (joint == QuadrupedJointName.FRONT_LEFT_HIP_PITCH)
         return -HIP_PITCH_ANGLE;
      if (joint == QuadrupedJointName.FRONT_LEFT_KNEE_PITCH)
         return KNEE_PITCH_ANGLE;
      if (joint == QuadrupedJointName.FRONT_RIGHT_HIP_ROLL)
         return HIP_ROLL_ANGLE;
      if (joint == QuadrupedJointName.FRONT_RIGHT_HIP_PITCH)
         return -HIP_PITCH_ANGLE;
      if (joint == QuadrupedJointName.FRONT_RIGHT_KNEE_PITCH)
         return KNEE_PITCH_ANGLE;
      if (joint == QuadrupedJointName.HIND_RIGHT_HIP_ROLL)
         return HIP_ROLL_ANGLE;
      if (joint == QuadrupedJointName.HIND_RIGHT_HIP_PITCH)
         return HIP_PITCH_ANGLE;
      if (joint == QuadrupedJointName.HIND_RIGHT_KNEE_PITCH)
         return -KNEE_PITCH_ANGLE;
      if (joint == QuadrupedJointName.HIND_LEFT_HIP_ROLL)
         return HIP_ROLL_ANGLE;
      if (joint == QuadrupedJointName.HIND_LEFT_HIP_PITCH)
         return HIP_PITCH_ANGLE;
      if (joint == QuadrupedJointName.HIND_LEFT_KNEE_PITCH)
         return -KNEE_PITCH_ANGLE;

      throw new RuntimeException(joint + " not defined!");
   }
}
