package us.ihmc.openAlexander;

import us.ihmc.openAlexander.parameters.model.AlexanderPhysicalProperties;
import us.ihmc.avatar.drcRobot.RobotVersion;
import us.ihmc.openAlexander.parameters.model.OpenAlexanderURDFParameters;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Collection;

public interface AlexanderVersionInterface extends RobotVersion
{
   String getRobotModelResourceDirectory();

   Collection<String> getURDFDescriptionResources();

   Collection<String> getXMLDescriptionResources();

   default boolean hasCycloidForearms()
   {
      return hasCycloidForearm(RobotSide.LEFT) || hasCycloidForearm(RobotSide.RIGHT);
   }

   boolean hasCycloidForearm(RobotSide robotSide);

   AlexanderJointMap getJointMap();

   boolean hasNubHands(RobotSide side);

   AlexanderSensorInformation getSensorInformation();

   AlexanderPhysicalProperties getPhysicalProperties();

   OpenAlexanderURDFParameters getURDFParameters();
}
