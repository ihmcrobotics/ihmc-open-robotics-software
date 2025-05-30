package us.ihmc.alexander;

import us.ihmc.alexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.alexander.parameters.model.HumanoidURDFParameterInterface;
import us.ihmc.avatar.drcRobot.RobotVersion;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Collection;

public interface AlexanderVersionInterface extends RobotVersion
{
   Collection<String> getModelPath();

   Collection<String> getHardwareMapResources();

   boolean hasCycloidForearms();

   AlexanderJointMap getJointMap();

   boolean hasNubHands(RobotSide side);

   AlexanderSensorInformation getSensorInformation();

   AlexanderPhysicalProperties getPhysicalProperties();

   HumanoidURDFParameterInterface getURDFParameters();
}
