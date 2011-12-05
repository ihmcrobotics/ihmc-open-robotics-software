package us.ihmc.commonWalkingControlModules.controlModules.arm;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.ArmControlModule;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.RevoluteJoint;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PIDController;

public abstract class IDArmControlModule implements ArmControlModule
{
   protected final ProcessedSensorsInterface processedSensors;
   protected final YoVariableRegistry registry = new YoVariableRegistry("IDControlModule");
   
   protected final InverseDynamicsCalculator armsIDCalculator;
   protected final SideDependentList<EnumMap<ArmJointName, RevoluteJoint>> armJointArrayLists;
   
   protected final SideDependentList<EnumMap<ArmJointName, PIDController>> armQddReferenceControllers = SideDependentList.createListOfEnumMaps(ArmJointName.class);

   protected final SideDependentList<EnumMap<ArmJointName, DoubleYoVariable>> desiredArmJointPositions = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   protected final SideDependentList<EnumMap<ArmJointName, DoubleYoVariable>> desiredArmJointVelocities = SideDependentList.createListOfEnumMaps(ArmJointName.class);

//   protected final SideDependentList<EnumMap<ArmJointName, Double>> desiredArmJointAccelerations = SideDependentList.createListOfEnumMaps(ArmJointName.class);

   private final double controlDT;
   
   public IDArmControlModule(ProcessedSensorsInterface processedSensors, double controlDT, YoVariableRegistry parentRegistry, InverseDynamicsCalculator armsIDCalculator, SideDependentList<EnumMap<ArmJointName, RevoluteJoint>> armJointArrayLists)
   {
      this.armJointArrayLists = armJointArrayLists;
      this.armsIDCalculator = armsIDCalculator;
      this.processedSensors = processedSensors;
      this.controlDT = controlDT;
      
      populateYoVariables();
      populateQddReferenceControllers();
      setGains();
      setDesiredJointPositionsAndVelocities();
      parentRegistry.addChild(registry);
   }

   protected abstract void setDesiredJointPositionsAndVelocities();
   
//   protected abstract void computeDesireds();

   protected abstract void setGains();

   public void doArmControl(ArmTorques[] armTorquesToPack)
   {

      for (RobotSide robotSide : RobotSide.values())
      {
         
         EnumMap<ArmJointName, PIDController> armQddReferenceControllers = this.armQddReferenceControllers.get(robotSide);

         for (ArmJointName armJointName : ArmJointName.values())
         {
            PIDController pidController = armQddReferenceControllers.get(armJointName);
            double desiredPosition = desiredArmJointPositions.get(robotSide).get(armJointName).getDoubleValue();
            double desiredVelocity = desiredArmJointVelocities.get(robotSide).get(armJointName).getDoubleValue();
            
            double desiredAcceleration = pidController.compute(processedSensors.getArmJointPosition(robotSide, armJointName), desiredPosition,
                  processedSensors.getArmJointVelocity(robotSide, armJointName), desiredVelocity, controlDT);
            
//            desiredArmJointAccelerations.get(robotSide).put(armJointName, desiredAcceleration);
            armJointArrayLists.get(robotSide).get(armJointName).setQddDesired(desiredAcceleration);
         }
      }
      
      armsIDCalculator.compute();
      
      for (RobotSide robotSide : RobotSide.values())
      {
         ArmTorques armTorques = armTorquesToPack[robotSide.ordinal()];
         for (ArmJointName armJointName : ArmJointName.values())
         {
            double torque = armJointArrayLists.get(robotSide).get(armJointName).getTau();
            armTorques.setTorque(armJointName, torque);
         }
      }
      

   }

   private void populateYoVariables()
   {
      for (RobotSide robotSide : RobotSide.values())
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

   
   private void populateQddReferenceControllers()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         for (ArmJointName armJointName : ArmJointName.values())
         {
            String name = robotSide.getCamelCaseNameForStartOfExpression() + armJointName.getCamelCaseNameForMiddleOfExpression();
            armQddReferenceControllers.get(robotSide).put(armJointName, new PIDController(name, registry));
         }
      }
   }
}
