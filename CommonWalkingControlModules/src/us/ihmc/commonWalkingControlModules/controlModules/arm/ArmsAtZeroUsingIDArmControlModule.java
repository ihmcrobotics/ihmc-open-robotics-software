package us.ihmc.commonWalkingControlModules.controlModules.arm;

import java.util.EnumMap;

import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class ArmsAtZeroUsingIDArmControlModule extends IDArmControlModule
{
   public ArmsAtZeroUsingIDArmControlModule(ProcessedSensorsInterface processedSensors, double controlDT, YoVariableRegistry parentRegistry, InverseDynamicsCalculator armsIDCalculator, SideDependentList<EnumMap<ArmJointName, RevoluteJoint>> armJointArrayLists)
    {
      super(processedSensors, controlDT, parentRegistry, armsIDCalculator, armJointArrayLists);
   }

   protected void setDesiredJointPositionsAndVelocities()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
//         desiredArmJointPositions.get(robotSide).get(ArmJointName.SHOULDER_ROLL).set(robotSide.negateIfRightSide(Math.PI / 2.0));
//         desiredArmJointPositions.get(robotSide).get(ArmJointName.SHOULDER_YAW).set(robotSide.negateIfRightSide(0.7));
      }
   }

   protected void setGains()
   {
      
      double gainScaling = 1.0;
      for (RobotSide robotSide : RobotSide.values)
      {
         armDesiredQddControllers.get(robotSide).get(ArmJointName.SHOULDER_PITCH).setProportionalGain(gainScaling * 100.0);
         armDesiredQddControllers.get(robotSide).get(ArmJointName.SHOULDER_ROLL).setProportionalGain(gainScaling * 200.0);
         armDesiredQddControllers.get(robotSide).get(ArmJointName.SHOULDER_YAW).setProportionalGain(gainScaling * 100.0);
         armDesiredQddControllers.get(robotSide).get(ArmJointName.ELBOW_PITCH).setProportionalGain(gainScaling * 100.0);
         armDesiredQddControllers.get(robotSide).get(ArmJointName.WRIST_ROLL).setProportionalGain(gainScaling * 100.0);
         armDesiredQddControllers.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH).setProportionalGain(gainScaling * 100.0);

         armDesiredQddControllers.get(robotSide).get(ArmJointName.SHOULDER_PITCH).setDerivativeGain(gainScaling * 10.0);
         armDesiredQddControllers.get(robotSide).get(ArmJointName.SHOULDER_ROLL).setDerivativeGain(gainScaling * 20.0);
         armDesiredQddControllers.get(robotSide).get(ArmJointName.SHOULDER_YAW).setDerivativeGain(gainScaling * 10.0);
         armDesiredQddControllers.get(robotSide).get(ArmJointName.ELBOW_PITCH).setDerivativeGain(gainScaling * 10.0);
         armDesiredQddControllers.get(robotSide).get(ArmJointName.WRIST_ROLL).setDerivativeGain(gainScaling * 10.0);
         armDesiredQddControllers.get(robotSide).get(ArmJointName.FIRST_WRIST_PITCH).setDerivativeGain(gainScaling * 10.0);
      }
   }
}

