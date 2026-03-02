package us.ihmc.robotics.sensors;

import java.util.Collection;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface FootSwitchFactory
{
   default FootSwitchInterface newFootSwitch(String namePrefix,
                                             ContactablePlaneBody foot,
                                             Collection<? extends ContactablePlaneBody> otherFeet,
                                             ForceSensorDataReadOnly footForceSensor,
                                             double totalRobotWeight,
                                             DoubleProvider switchDT,
                                             YoRegistry registry)
   {
      return newFootSwitch(namePrefix, foot, otherFeet, null, footForceSensor, totalRobotWeight, switchDT, registry);
   }

   FootSwitchInterface newFootSwitch(String namePrefix,
                                     ContactablePlaneBody foot,
                                     Collection<? extends ContactablePlaneBody> otherFeet,
                                     RigidBodyBasics rootBody,
                                     ForceSensorDataReadOnly footForceSensor,
                                     double totalRobotWeight,
                                     DoubleProvider switchDT,
                                     YoRegistry registry);
}
