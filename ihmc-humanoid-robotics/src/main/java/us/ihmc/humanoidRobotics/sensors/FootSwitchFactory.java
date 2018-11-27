package us.ihmc.humanoidRobotics.sensors;

import java.util.Collection;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface FootSwitchFactory
{
   FootSwitchInterface newFootSwitch(ContactablePlaneBody foot, Collection<? extends ContactablePlaneBody> otherFeet, ForceSensorDataReadOnly footForceSensor,
                                     ContactSensorHolder contactSensorHolder, double totalRobotWeight, YoGraphicsListRegistry yoGraphicsListRegistry,
                                     YoVariableRegistry registry);
}
