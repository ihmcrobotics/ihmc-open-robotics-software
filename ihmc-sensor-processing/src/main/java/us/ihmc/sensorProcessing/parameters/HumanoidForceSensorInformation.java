package us.ihmc.sensorProcessing.parameters;

import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public interface HumanoidForceSensorInformation
{
   public String[] getForceSensorNames();

   public SideDependentList<String> getFeetForceSensorNames();

   public SideDependentList<String> getFeetContactSensorNames();

   public SideDependentList<String> getWristForceSensorNames();
}
