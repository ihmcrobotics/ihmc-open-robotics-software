package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.HipDamperControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.DoubleYoVariable;

public class SimpleHipYawDamperControlModule implements HipDamperControlModule
{
   private final ProcessedSensorsInterface processedSensors;
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleHipDamperControlModule");
   private final DoubleYoVariable hipYawDamping = new DoubleYoVariable("hipYawDamping", registry);

   public SimpleHipYawDamperControlModule(ProcessedSensorsInterface processedSensors, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      parentRegistry.addChild(registry);
   }

   public void addHipDamping(LegTorques legTorques)
   {
      RobotSide robotSide = legTorques.getRobotSide();
      LegJointName hipYaw = LegJointName.HIP_YAW;
      double dampingTorque = -hipYawDamping.getDoubleValue() * processedSensors.getLegJointVelocity(robotSide, hipYaw);
      legTorques.addTorque(hipYaw, dampingTorque);
   }
   
   public void setParametersForR2()
   {
      hipYawDamping.set(10.0);
   }
   
   public void setParametersForM2V2()
   {
      hipYawDamping.set(5.0);
   }

}
