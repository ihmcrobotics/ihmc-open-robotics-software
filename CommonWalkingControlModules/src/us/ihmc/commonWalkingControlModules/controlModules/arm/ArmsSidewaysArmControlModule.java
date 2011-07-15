package us.ihmc.commonWalkingControlModules.controlModules.arm;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ArmsSidewaysArmControlModule extends PDArmControlModule
{
   public ArmsSidewaysArmControlModule(ProcessedSensorsInterface processedSensors, double controlDT, YoVariableRegistry parentRegistry)
   {
      super(processedSensors, controlDT, parentRegistry);
   }

   protected void computeDesireds()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         desiredArmPositions.get(robotSide).get(ArmJointName.SHOULDER_ROLL).set(robotSide.negateIfRightSide(Math.PI / 2.0));
         desiredArmPositions.get(robotSide).get(ArmJointName.SHOULDER_YAW).set(robotSide.negateIfRightSide(0.7));
      }
   }

   protected void setGains()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_PITCH).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_ROLL).setProportionalGain(200.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_YAW).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.ELBOW).setProportionalGain(100.0);

         armControllers.get(robotSide).get(ArmJointName.SHOULDER_PITCH).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_ROLL).setDerivativeGain(20.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_YAW).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.ELBOW).setDerivativeGain(10.0);
      }
   }
}

