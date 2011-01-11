package us.ihmc.commonWalkingControlModules.controlModules.arm;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class FrozenArmControlModule extends PDArmControlModule
{
   public FrozenArmControlModule(ProcessedSensorsInterface processedSensors, double controlDT, YoVariableRegistry parentRegistry)
   {
      super(processedSensors, controlDT, parentRegistry);
   }

   protected void computeDesireds()
   {
      // do nothing, keep at zero.
   }

   protected void setGains()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_PITCH).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_ROLL).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_YAW).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.ELBOW).setProportionalGain(100.0);

         armControllers.get(robotSide).get(ArmJointName.SHOULDER_PITCH).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_ROLL).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_YAW).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.ELBOW).setDerivativeGain(10.0);
      }
   }
}

