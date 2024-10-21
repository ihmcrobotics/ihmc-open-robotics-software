package us.ihmc.sensorProcessing.parameters;

import us.ihmc.commons.robotics.robotSide.SideDependentList;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public interface HumanoidForceSensorInformation
{
   public String[] getForceSensorNames();

   public SideDependentList<String> getFeetForceSensorNames();

   public SideDependentList<String> getWristForceSensorNames();
}
