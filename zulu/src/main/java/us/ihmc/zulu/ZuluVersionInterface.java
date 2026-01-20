package us.ihmc.zulu;

import us.ihmc.zulu.parameters.model.ZuluPhysicalProperties;
import us.ihmc.avatar.drcRobot.RobotVersion;
import us.ihmc.zulu.parameters.model.HumanoidURDFParameterInterface;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Collection;

public interface ZuluVersionInterface extends RobotVersion
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

   ZuluJointMap getJointMap();

   boolean hasNubForearms(RobotSide side);

   ZuluSensorInformation getSensorInformation();

   ZuluPhysicalProperties getPhysicalProperties();

   HumanoidURDFParameterInterface getURDFParameters();
}
