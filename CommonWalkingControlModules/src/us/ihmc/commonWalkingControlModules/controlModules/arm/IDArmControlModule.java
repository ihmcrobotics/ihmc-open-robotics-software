package us.ihmc.commonWalkingControlModules.controlModules.arm;

import java.util.EnumMap;

import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.ArmControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmTorques;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.yoUtilities.controllers.PIDController;


public abstract class IDArmControlModule implements ArmControlModule
{
   protected final ProcessedSensorsInterface processedSensors;
   protected final YoVariableRegistry registry = new YoVariableRegistry("IDControlModule");

   protected final InverseDynamicsCalculator armsIDCalculator;
   protected final SideDependentList<EnumMap<ArmJointName, RevoluteJoint>> armJoints;

   protected final SideDependentList<EnumMap<ArmJointName, PIDController>> armDesiredQddControllers = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   protected final SideDependentList<EnumMap<ArmJointName, DoubleYoVariable>> desiredArmJointPositions = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   protected final SideDependentList<EnumMap<ArmJointName, DoubleYoVariable>> desiredArmJointVelocities = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   private final SideDependentList<EnumMap<ArmJointName, DoubleYoVariable>> desiredArmJointAcceleration = SideDependentList.createListOfEnumMaps(ArmJointName.class);

   private final double controlDT;
   
   public IDArmControlModule(ProcessedSensorsInterface processedSensors, double controlDT, YoVariableRegistry parentRegistry,
         InverseDynamicsCalculator armsIDCalculator, SideDependentList<EnumMap<ArmJointName, RevoluteJoint>> armJoints)
   {
      this.armJoints = armJoints;
      this.armsIDCalculator = armsIDCalculator;
      this.processedSensors = processedSensors;
      this.controlDT = controlDT;

      populateYoVariables();
      populateDesiredAccelerationControllers();
      setGains();
      setDesiredJointPositionsAndVelocities();
      parentRegistry.addChild(registry);
   }

   protected abstract void setDesiredJointPositionsAndVelocities();

   protected abstract void setGains();

   public void doArmControl(ArmTorques[] armTorquesToPack)
   {
//      mirrorDesiredPositionRightBasedOnLeft();
      
      setDesiredQdd();

      armsIDCalculator.compute();

      setArmTorques(armTorquesToPack);
   }

   private void mirrorDesiredPositionRightBasedOnLeft()
   {
      for (ArmJointName armJointName : ArmJointName.values())
      {
         double armJointQ = desiredArmJointPositions.get(RobotSide.LEFT).get(armJointName).getDoubleValue();
         desiredArmJointPositions.get(RobotSide.RIGHT).get(armJointName).set( - armJointQ);
      }
   }

   private void setDesiredQdd()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName armJointName : ArmJointName.values())
         {
            setDesiredQddForJoint(robotSide, armJointName);
            armJoints.get(robotSide).get(armJointName).setQddDesired(desiredArmJointAcceleration.get(robotSide).get(armJointName).getDoubleValue());
         }
      }
   }

   private void setDesiredQddForJoint(RobotSide robotSide, ArmJointName armJointName)
   {
      PIDController pidController = this.armDesiredQddControllers.get(robotSide).get(armJointName);
      double desiredPosition = desiredArmJointPositions.get(robotSide).get(armJointName).getDoubleValue();
      double desiredVelocity = desiredArmJointVelocities.get(robotSide).get(armJointName).getDoubleValue();

      double currentPosition = processedSensors.getArmJointPosition(robotSide, armJointName);
      double currentVelocity = processedSensors.getArmJointVelocity(robotSide, armJointName);

      double desiredAcceleration = pidController.compute(currentPosition, desiredPosition, currentVelocity, desiredVelocity, controlDT);

      desiredArmJointAcceleration.get(robotSide).get(armJointName).set(desiredAcceleration);
   }

   private void setArmTorques(ArmTorques[] armTorquesToPack)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTorques armTorques = armTorquesToPack[robotSide.ordinal()];
         for (ArmJointName armJointName : ArmJointName.values())
         {
            double torque = armJoints.get(robotSide).get(armJointName).getTau();
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

            DoubleYoVariable positionVariable = new DoubleYoVariable("q_" + name, registry);
            desiredArmJointPositions.get(robotSide).put(armJointName, positionVariable);

            DoubleYoVariable velocityVariable = new DoubleYoVariable("qd_" + name, registry);
            desiredArmJointVelocities.get(robotSide).put(armJointName, velocityVariable);
            
            DoubleYoVariable accelerationVariable = new DoubleYoVariable("qdd_" + name, registry);
            desiredArmJointAcceleration.get(robotSide).put(armJointName, accelerationVariable);
         }
      }
   }

   private void populateDesiredAccelerationControllers()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName armJointName : ArmJointName.values())
         {
            String name = "desired" + robotSide.getCamelCaseNameForStartOfExpression() + armJointName.getCamelCaseNameForMiddleOfExpression() + "Ctrl";
            armDesiredQddControllers.get(robotSide).put(armJointName, new PIDController(name, registry));
         }
      }
   }
}
