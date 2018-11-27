package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.Collection;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class KinematicsBasedFootSwitchFactory implements FootSwitchFactory
{
   private double defaultContactThresholdHeight;

   public KinematicsBasedFootSwitchFactory()
   {
   }

   public void setDefaultContactThresholdHeight(double defaultContactThresholdHeight)
   {
      this.defaultContactThresholdHeight = defaultContactThresholdHeight;
   }

   @Override
   public FootSwitchInterface newFootSwitch(ContactablePlaneBody foot, Collection<? extends ContactablePlaneBody> otherFeet,
                                            ForceSensorDataReadOnly footForceSensor, ContactSensorHolder contactSensorHolder, double totalRobotWeight,
                                            YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      DoubleProvider contactThresholdHeight = new DoubleParameter("ContactThresholdHeight", registry, defaultContactThresholdHeight);
      return new KinematicsBasedFootSwitch(foot.getName(), foot, otherFeet, contactThresholdHeight, totalRobotWeight, registry);
   }
}
