package us.ihmc.atlas.parameters;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

public class AtlasDefaultArmConfigurations implements DefaultArmConfigurations
{
   @Override
   public double[] getArmDefaultConfigurationJointAngles(ArmConfigurations armConfiguration, RobotSide robotSide)
   {
      //     SHOULDER_YAW, SHOULDER_ROLL, ELBOW_PITCH, ELBOW_ROLL, FIRST_WRIST_PITCH, WRIST_ROLL
      double[] armJointAngles;
      switch (armConfiguration)
      {
      case HOME:
         armJointAngles = fillArmJointAnglesArray(robotSide.negateIfRightSide(0.1), robotSide.negateIfRightSide(-1.3), 1.94, robotSide.negateIfRightSide(1.18),
               0.0, robotSide.negateIfRightSide(-0.07), 0.0);
         return armJointAngles;
      case WIDER_HOME:
         //TODO put the same angles as those in the UI
         armJointAngles = fillArmJointAnglesArray(robotSide.negateIfRightSide(0.1), robotSide.negateIfRightSide(-1.3), 1.94, robotSide.negateIfRightSide(1.18),
               0.0, robotSide.negateIfRightSide(-0.07), 0.0);
         return armJointAngles;
      case COMPACT_HOME:
         armJointAngles = fillArmJointAnglesArray(robotSide.negateIfRightSide(0.1), robotSide.negateIfRightSide(-1.6), 1.9, robotSide.negateIfRightSide(2.1),
               0.0, robotSide.negateIfRightSide(-0.55), 0.0);
         return armJointAngles;
      default:
         return null;
      }
   }

   private double[] fillArmJointAnglesArray(double shoulderYaw, double shoulderRoll, double elbowPitch, double elbowRoll, double wristPitch, double wristRoll, double wristSecondPitch)
   {
      double[] armJointAngles = new double[7];
      armJointAngles[0] = shoulderYaw;
      armJointAngles[1] = shoulderRoll;
      armJointAngles[2] = elbowPitch;
      armJointAngles[3] = elbowRoll;
      armJointAngles[4] = wristPitch;
      armJointAngles[5] = wristRoll;
      armJointAngles[6] = wristSecondPitch;

      return armJointAngles;

   }

}
