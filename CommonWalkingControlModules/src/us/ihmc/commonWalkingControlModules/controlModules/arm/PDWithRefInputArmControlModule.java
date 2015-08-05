package us.ihmc.commonWalkingControlModules.controlModules.arm;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmTorques;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public abstract class PDWithRefInputArmControlModule implements RefInputArmControlModule
{
   protected final ProcessedSensorsInterface processedSensors;
   protected final YoVariableRegistry registry = new YoVariableRegistry("PDControlModule");
   protected final SideDependentList<EnumMap<ArmJointName, PIDController>> armControllers = SideDependentList.createListOfEnumMaps(ArmJointName.class);

   protected final SideDependentList<EnumMap<ArmJointName, DoubleYoVariable>> desiredArmJointPositions = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   protected final SideDependentList<EnumMap<ArmJointName, DoubleYoVariable>> desiredArmJointVelocities = SideDependentList.createListOfEnumMaps(ArmJointName.class);

   private final double controlDT;
   
   public PDWithRefInputArmControlModule(ProcessedSensorsInterface processedSensors, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      this.controlDT = controlDT;
      
      populateYoVariables();
      populateControllers();
      setGains();
      parentRegistry.addChild(registry);
   }

   protected abstract void computeDesireds();

   protected abstract void setGains();

   public void doArmControl(ArmTorques[] armTorquesToPack)
   {
      computeDesireds();

      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTorques armTorques = armTorquesToPack[robotSide.ordinal()];
         EnumMap<ArmJointName, PIDController> armController = armControllers.get(robotSide);

         for (ArmJointName armJointName : ArmJointName.values())
         {
            PIDController pidController = armController.get(armJointName);
            double desiredPosition = desiredArmJointPositions.get(robotSide).get(armJointName).getDoubleValue();
            double desiredVelocity = desiredArmJointVelocities.get(robotSide).get(armJointName).getDoubleValue();
            double torque = pidController.compute(processedSensors.getArmJointPosition(robotSide, armJointName), desiredPosition,
                  processedSensors.getArmJointVelocity(robotSide, armJointName), desiredVelocity, controlDT);
            armTorques.setTorque(armJointName, torque);
         }
      }
   }

   private void populateYoVariables()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName armJointName : ArmJointName.values())
         {
            String name = "desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + armJointName.getCamelCaseNameForMiddleOfExpression();

            DoubleYoVariable positionVariable = new DoubleYoVariable(name + "Position", registry);
            desiredArmJointPositions.get(robotSide).put(armJointName, positionVariable);

            DoubleYoVariable velocityVariable = new DoubleYoVariable(name + "Velocity", registry);
            desiredArmJointVelocities.get(robotSide).put(armJointName, velocityVariable);
         }
      }
   }

   private void populateControllers()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName armJointName : ArmJointName.values())
         {
            String name = robotSide.getCamelCaseNameForStartOfExpression() + armJointName.getCamelCaseNameForMiddleOfExpression();
            armControllers.get(robotSide).put(armJointName, new PIDController(name, registry));
         }
      }
   }

   public void setDesireds(SideDependentList<EnumMap<ArmJointName, Double>> desiredJointPositions)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName armJointName : ArmJointName.values())
         {
            desiredArmJointPositions.get(robotSide).get(armJointName).set(desiredJointPositions.get(robotSide).get(armJointName));
         }
      }
   }
}

