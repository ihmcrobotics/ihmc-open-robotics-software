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

   /** Has at least one cycloid forearm. */
   default boolean hasCycloidForearm()
   {
      return hasCycloidForearm(RobotSide.LEFT) || hasCycloidForearm(RobotSide.RIGHT);
   }

   boolean hasCycloidForearm(RobotSide robotSide);

   AlexanderJointMap getJointMap();

   boolean hasNubForearms(RobotSide side);

   AlexanderSensorInformation getSensorInformation();

   AlexanderPhysicalProperties getPhysicalProperties();

   OpenAlexanderURDFParameters getURDFParameters();
}
