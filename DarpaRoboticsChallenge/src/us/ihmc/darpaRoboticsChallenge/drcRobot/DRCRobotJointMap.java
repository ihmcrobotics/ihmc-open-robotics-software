package us.ihmc.darpaRoboticsChallenge.drcRobot;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

public abstract class DRCRobotJointMap implements SDFJointNameMap
{
   // TODO Crap
   public abstract double getAnkleHeight();

   // TODO SensorMap
   public abstract String[] getIMUSensorsToUse();

   public abstract String getLeftCameraName();

   public abstract String getRightCameraName();

   public abstract String getLidarSensorName();

   public abstract String getLidarJointName();

   public abstract String[] getForceSensorNames();

   public abstract SideDependentList<String> getFeetForceSensorNames();

   // TODO "real" JointMap
   public abstract String getNameOfJointBeforeChest();

   public abstract String getNameOfJointBeforeThigh(RobotSide robotSide);

   public abstract String getNameOfJointBeforeHand(RobotSide robotSide);

   public abstract SideDependentList<String> getJointBeforeThighNames();

   public abstract String[] getOrderedJointNames();

   public abstract String getHighestNeckPitchJointName();
}