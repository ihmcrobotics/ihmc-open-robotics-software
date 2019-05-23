package us.ihmc.sensorProcessing.parameters;

import us.ihmc.robotics.robotSide.SideDependentList;

public interface HumanoidRobotSensorInformation extends AvatarRobotVisionSensorInformation, AvatarRobotRosIntegratedSensorInformation
{
   public String[] getIMUSensorsToUseInStateEstimator();

   public String[] getForceSensorNames();

   public SideDependentList<String> getFeetForceSensorNames();

   public SideDependentList<String> getFeetContactSensorNames();

   public SideDependentList<String> getWristForceSensorNames();

   public String getPrimaryBodyImu();
}
