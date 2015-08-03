package us.ihmc.wholeBodyController.parameters;

import us.ihmc.robotics.robotSide.RobotSide;

public interface DefaultArmConfigurations
{
   public enum ArmConfigurations
   {
      HOME, WIDER_HOME, COMPACT_HOME
   }
   public abstract double[] getArmDefaultConfigurationJointAngles(ArmConfigurations armConfiguration, RobotSide robotSide);
}
