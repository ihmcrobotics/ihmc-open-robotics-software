package us.ihmc.steppr.hardware.controllers;

import java.util.EnumMap;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.acsell.hardware.state.AcsellAnkleAngleCalculator;
import us.ihmc.acsell.hardware.state.AcsellAnkleFullComputation;
import us.ihmc.acsell.hardware.state.AcsellFourbarCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.robotics.robotController.OutputProcessor;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.StepprUtil;
import us.ihmc.steppr.hardware.configuration.StepprAnkleKinematicParameters;
import us.ihmc.steppr.hardware.configuration.StepprFourbarProperties;

public class StepprOutputProcessor implements OutputProcessor
{

   private final YoVariableRegistry registry = new YoVariableRegistry("StepprOutputProcessor");
   private final EnumMap<StepprActuator, DoubleYoVariable> predictedMotorPower = new EnumMap<StepprActuator, DoubleYoVariable>(StepprActuator.class);
   private final DoubleYoVariable totalPredictedRobotPower = new DoubleYoVariable("totalPredictedMotorPower", registry);
   private final SimpleMovingAverageFilteredYoVariable totalPredictedRobotPowerAverage = new SimpleMovingAverageFilteredYoVariable(
         totalPredictedRobotPower.getName() + "MovingAverage", 1000, totalPredictedRobotPower, registry);

   //private final AcsellAnkleAngleCalculator ankleCalculator = new AcsellAnkleInterpolator(new StepprAnkleKinematicParameters());
   private final AcsellAnkleAngleCalculator rightAnkleCalculator = new AcsellAnkleFullComputation(new StepprAnkleKinematicParameters(),RobotSide.RIGHT);
   private final AcsellAnkleAngleCalculator leftAnkleCalculator = new AcsellAnkleFullComputation(new StepprAnkleKinematicParameters(),RobotSide.LEFT);
   private final AcsellFourbarCalculator leftFourbar;
   private final AcsellFourbarCalculator rightFourbar;
   private EnumMap<StepprJoint, OneDoFJoint> wholeBodyControlJoints;

   public StepprOutputProcessor(FullRobotModel controllerFullRobotModel)
   {
      wholeBodyControlJoints = StepprUtil.createJointMap(controllerFullRobotModel.getOneDoFJoints());

      for (StepprActuator actuator : StepprActuator.values)
      {
         predictedMotorPower.put(actuator, new DoubleYoVariable(actuator.getName() + "PredictedMotorPower", registry));
      }
      leftFourbar = new AcsellFourbarCalculator(new StepprFourbarProperties(), "leftOutputProcessor", RobotSide.LEFT, registry);
      rightFourbar = new AcsellFourbarCalculator(new StepprFourbarProperties(), "rightOutputProcessor", RobotSide.RIGHT, registry);
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
         double actuatorTau = 0;
         OneDoFJoint oneDoFJoint = wholeBodyControlJoints.get(joint);
         if (joint.isLinear())
         {
            actuatorTau = oneDoFJoint.getTau() / joint.getRatio();
            StepprActuator actuator = joint.getActuators()[0];
            predictedMotorPower.get(actuator).set(calcPower(actuatorTau, actuator.getKm()));
         }
      }

      updateKnees(StepprJoint.LEFT_KNEE_Y, StepprActuator.LEFT_KNEE, StepprJoint.RIGHT_KNEE_Y, StepprActuator.RIGHT_KNEE);
      updateAnkle(rightAnkleCalculator, StepprJoint.RIGHT_ANKLE_X, StepprJoint.RIGHT_ANKLE_Y, StepprActuator.RIGHT_ANKLE_LEFT, StepprActuator.RIGHT_ANKLE_RIGHT);
      updateAnkle(leftAnkleCalculator, StepprJoint.LEFT_ANKLE_X, StepprJoint.LEFT_ANKLE_Y, StepprActuator.LEFT_ANKLE_LEFT, StepprActuator.LEFT_ANKLE_RIGHT);      

      double sumTotalMotorPower = 0;
      for (StepprActuator actuator : StepprActuator.values)
      {
         sumTotalMotorPower += predictedMotorPower.get(actuator).getDoubleValue();
      }
      totalPredictedRobotPower.set(sumTotalMotorPower);
      totalPredictedRobotPowerAverage.update();
   }
   
   private void updateKnees(StepprJoint leftJoint, StepprActuator leftActuator,StepprJoint rightJoint, StepprActuator rightActuator)
   {

      OneDoFJoint oneDofLeftKnee = wholeBodyControlJoints.get(leftJoint);
      OneDoFJoint oneDofRightKnee = wholeBodyControlJoints.get(rightJoint);
      
      leftFourbar.update(oneDofLeftKnee);
      rightFourbar.update(oneDofRightKnee);

      predictedMotorPower.get(leftActuator).set(
            calcPower(oneDofLeftKnee.getTau() / leftFourbar.getFourbarRatioBasedOnCalculatedInputAngle() / leftJoint.getRatio(), leftActuator.getKm()));

      predictedMotorPower.get(rightActuator).set(
              calcPower(oneDofRightKnee.getTau() / rightFourbar.getFourbarRatioBasedOnCalculatedInputAngle() / rightJoint.getRatio(), rightActuator.getKm()));

   }
   
   private void updateAnkle(AcsellAnkleAngleCalculator ankleCalculator, StepprJoint ankleX, StepprJoint ankleY, StepprActuator ankleLeftActuator, StepprActuator ankleRightActuator)
   {

      OneDoFJoint oneDofAnkleX = wholeBodyControlJoints.get(ankleX);
      OneDoFJoint oneDofAnkleY = wholeBodyControlJoints.get(ankleY);
      
      ankleCalculator.updateAnkleState(oneDofAnkleX, oneDofAnkleY);

      predictedMotorPower.get(ankleRightActuator).set(
              calcPower(ankleCalculator.getComputedTauRightActuator(), ankleRightActuator.getKm()));

      predictedMotorPower.get(ankleLeftActuator).set(
              calcPower(ankleCalculator.getComputedTauLeftActuator(), ankleLeftActuator.getKm()));

   }

   
   private double calcPower(double tau, double km)
   {
      return square(tau / km);
   }

   private static double square(double v)
   {
      return v * v;
   }
}
