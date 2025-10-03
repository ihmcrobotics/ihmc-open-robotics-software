package us.ihmc.openAlexander;

import us.ihmc.openAlexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.openAlexander.parameters.model.HumanoidURDFParameterInterface;
import us.ihmc.avatar.drcRobot.RobotVersion;
import us.ihmc.openAlexander.parameters.model.OpenAlexanderURDFParameters;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Collection;

public interface AlexanderVersionInterface extends RobotVersion
{
   Collection<String> getModelPath();

   Collection<String> getHardwareMapResources();

   boolean hasCycloidForearms();

   AlexanderJointMap getJointMap();

   boolean hasNubHands(RobotSide side);
   
   boolean hasCycloidHands(RobotSide side);

   AlexanderSensorInformation getSensorInformation();

   AlexanderPhysicalProperties getPhysicalProperties();

   OpenAlexanderURDFParameters getURDFParameters();
}
