package us.ihmc.commonWalkingControlModules.controlModules;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.HipDamperControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;

public class SimpleHipDamperControlModule implements HipDamperControlModule
{
   private final ProcessedSensorsInterface processedSensors;
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleHipDamperControlModule");
   private final DoubleYoVariable hipYawDamping = new DoubleYoVariable("hipYawDamping", registry);

   public SimpleHipDamperControlModule(ProcessedSensorsInterface processedSensors, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
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
      hipYawDamping.set(10.0);
   }

}
