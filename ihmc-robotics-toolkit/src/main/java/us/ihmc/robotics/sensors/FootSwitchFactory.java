package us.ihmc.robotics.sensors;

import java.util.Collection;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface FootSwitchFactory
{
   FootSwitchInterface newFootSwitch(String namePrefix, ContactablePlaneBody foot, Collection<? extends ContactablePlaneBody> otherFeet,
                                     ForceSensorDataReadOnly footForceSensor, double totalRobotWeight, YoGraphicsListRegistry yoGraphicsListRegistry,
                                     YoRegistry registry);
}
