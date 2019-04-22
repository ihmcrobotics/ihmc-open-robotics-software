package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.Collection;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class KinematicsBasedFootSwitchFactory implements FootSwitchFactory
{
   private double defaultContactThresholdHeight = 0.05;
   private DoubleProvider contactThresholdHeight;

   public KinematicsBasedFootSwitchFactory()
   {
   }

   /**
    * When determining whether a foot has hit the ground the controller can use the height difference
    * between the swing foot and the lowest of the feet of the robot. If the difference falls below
    * this threshold foot-ground contact is assumed.
    */
   public void setDefaultContactThresholdHeight(double defaultContactThresholdHeight)
   {
      this.defaultContactThresholdHeight = defaultContactThresholdHeight;
   }

   @Override
   public FootSwitchInterface newFootSwitch(String namePrefix, ContactablePlaneBody foot, Collection<? extends ContactablePlaneBody> otherFeet,
                                            ForceSensorDataReadOnly footForceSensor, double totalRobotWeight, YoGraphicsListRegistry yoGraphicsListRegistry,
                                            YoVariableRegistry registry)
   {
      if (contactThresholdHeight == null)
         contactThresholdHeight = new DoubleParameter("ContactThresholdHeight", registry, defaultContactThresholdHeight);

      return new KinematicsBasedFootSwitch(namePrefix, foot, otherFeet, contactThresholdHeight, totalRobotWeight, registry);
   }
}
