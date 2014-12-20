package us.ihmc.steppr.hardware.controllers;

import java.util.EnumMap;

import us.ihmc.simulationconstructionset.robotController.OutputProcessor;
import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.StepprUtil;
import us.ihmc.steppr.hardware.command.StepprCommand;
import us.ihmc.steppr.hardware.command.StepprJointCommand;
import us.ihmc.steppr.hardware.state.StepprAnkleInterpolator;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class StepprOutputProcessor implements OutputProcessor
{

   private final YoVariableRegistry registry = new YoVariableRegistry("StepprOutputProcessor");
   private final EnumMap<StepprActuator, DoubleYoVariable> outputActuators = new EnumMap<StepprActuator, DoubleYoVariable>(StepprActuator.class);
   private final DoubleYoVariable totalDesiredRobotPower = new DoubleYoVariable("totalDesiredMotorPowoer", registry);
   private final StepprAnkleInterpolator ankleInterpolator = new StepprAnkleInterpolator();
   private EnumMap<StepprJoint, OneDoFJoint> wholeBodyControlJoints;
   private final StepprCommand command = new StepprCommand(registry);

   public StepprOutputProcessor(FullRobotModel controllerFullRobotModel)
   {
      wholeBodyControlJoints = StepprUtil.createJointMap(controllerFullRobotModel.getOneDoFJoints());

      for (StepprActuator actuator : StepprActuator.values)
      {
         outputActuators.put(actuator, new DoubleYoVariable(actuator.getName() + "MotorPower", registry));
      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void update()
   {
      for (StepprJoint joint : StepprJoint.values)
      {
         OneDoFJoint oneDoFJoint = wholeBodyControlJoints.get(joint);
         StepprJointCommand jointCommand = command.getStepprJointCommand(joint);
         jointCommand.setTauDesired(oneDoFJoint.getTau(), oneDoFJoint.getQ(), oneDoFJoint.getQd(), Double.NaN, Double.NaN);
      }
      
      updateAnkle(StepprJoint.LEFT_ANKLE_X, StepprJoint.LEFT_ANKLE_Y);
      updateAnkle(StepprJoint.RIGHT_ANKLE_X, StepprJoint.RIGHT_ANKLE_Y);
      command.updateActuatorCommandsFromJointCommands();

      double accTotalRobotPower =0;
      for (StepprActuator actuator : StepprActuator.values)
      {         
         double actuatorTau=command.getAcutatorTau(actuator);
         double actuatorPower = square( actuatorTau/ actuator.getKm());
         outputActuators.get(actuator).set(actuatorPower);
         accTotalRobotPower += actuatorPower;
      }
      totalDesiredRobotPower.set(accTotalRobotPower);
   }

   private double square(double v)
   {
      return v * v;
   }

   private void updateAnkle(StepprJoint ankleX, StepprJoint ankleY)
   {

      OneDoFJoint oneDofAnkleX = wholeBodyControlJoints.get(ankleX);
      OneDoFJoint oneDofAnkleY = wholeBodyControlJoints.get(ankleY);
      double motor1 = ankleInterpolator.calculateMotor1Angle(oneDofAnkleX.getQ(), oneDofAnkleY.getQ());
      double motor2 = ankleInterpolator.calculateMotor2Angle(oneDofAnkleX.getQ(), oneDofAnkleY.getQ());
      command.getStepprJointCommand(ankleX).setTauDesired(oneDofAnkleX.getTau(), oneDofAnkleX.getQ(), oneDofAnkleX.getQd(), motor1, motor2);
      command.getStepprJointCommand(ankleY).setTauDesired(oneDofAnkleY.getTau(), oneDofAnkleY.getQ(), oneDofAnkleY.getQd(), motor1, motor2);
   }

}
