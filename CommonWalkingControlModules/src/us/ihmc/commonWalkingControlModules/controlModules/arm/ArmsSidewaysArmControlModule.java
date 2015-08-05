package us.ihmc.commonWalkingControlModules.controlModules.arm;

import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class ArmsSidewaysArmControlModule extends PDArmControlModule
{
   public ArmsSidewaysArmControlModule(ProcessedSensorsInterface processedSensors, double controlDT, YoVariableRegistry parentRegistry)
   {
      super(processedSensors, controlDT, parentRegistry);
   }

   protected void computeDesireds()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         desiredArmJointPositions.get(robotSide).get(ArmJointName.SHOULDER_ROLL).set(robotSide.negateIfRightSide(Math.PI / 2.1));
         desiredArmJointPositions.get(robotSide).get(ArmJointName.SHOULDER_YAW).set(robotSide.negateIfRightSide(0.7));
      }
      
//      RobotSide robotSide = RobotSide.LEFT;
//      desiredArmJointPositions.get(robotSide).get(ArmJointName.SHOULDER_ROLL).set(robotSide.negateIfRightSide(0.0));

   }

   protected void setGains()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_PITCH).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_ROLL).setProportionalGain(200.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_YAW).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.ELBOW_PITCH).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.WRIST_ROLL).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH).setProportionalGain(100.0);

         armControllers.get(robotSide).get(ArmJointName.SHOULDER_PITCH).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_ROLL).setDerivativeGain(20.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_YAW).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.ELBOW_PITCH).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.WRIST_ROLL).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH).setDerivativeGain(10.0);
      }
   }
}

