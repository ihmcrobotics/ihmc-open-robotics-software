package us.ihmc.openAlexander;

import us.ihmc.openAlexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.avatar.drcRobot.RobotVersion;
import us.ihmc.openAlexander.parameters.model.OpenAlexanderURDFParameters;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Collection;

public interface ZuluVersionInterface extends RobotVersion
{
   public String getRobotModelResourceDirectory();

   Collection<String> getURDFDescriptionResources();

   Collection<String> getXMLDescriptionResources();

   boolean hasCycloidForearms();

   boolean hasCycloidForearm(RobotSide robotSide);

   ZuluJointMap getJointMap();

   boolean hasNubHands(RobotSide side);

   ZuluSensorInformation getSensorInformation();

   AlexanderPhysicalProperties getPhysicalProperties();

   OpenAlexanderURDFParameters getURDFParameters();
}
