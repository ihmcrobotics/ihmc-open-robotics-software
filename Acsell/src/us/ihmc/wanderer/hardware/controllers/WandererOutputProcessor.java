package us.ihmc.wanderer.hardware.controllers;

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
import us.ihmc.wanderer.hardware.WandererActuator;
import us.ihmc.wanderer.hardware.WandererJoint;
import us.ihmc.wanderer.hardware.WandererUtil;
import us.ihmc.wanderer.hardware.configuration.WandererAnkleKinematicParameters;
import us.ihmc.wanderer.hardware.configuration.WandererFourbarProperties;

public class WandererOutputProcessor implements OutputProcessor
{

   private final YoVariableRegistry registry = new YoVariableRegistry("WandererOutputProcessor");
   private final EnumMap<WandererActuator, DoubleYoVariable> predictedMotorPower = new EnumMap<WandererActuator, DoubleYoVariable>(WandererActuator.class);
   private final DoubleYoVariable totalPredictedRobotPower = new DoubleYoVariable("totalPredictedMotorPower", registry);
   private final SimpleMovingAverageFilteredYoVariable totalPredictedRobotPowerAverage = new SimpleMovingAverageFilteredYoVariable(
         totalPredictedRobotPower.getName() + "MovingAverage", 1000, totalPredictedRobotPower, registry);

   //private final AcsellAnkleAngleCalculator ankleInterpolator = new AcsellAnkleInterpolator(new WandererAnkleKinematicParameters());
   private final AcsellAnkleAngleCalculator rightAnkleCalculator = new AcsellAnkleFullComputation(new WandererAnkleKinematicParameters(),RobotSide.RIGHT);
   private final AcsellAnkleAngleCalculator leftAnkleCalculator = new AcsellAnkleFullComputation(new WandererAnkleKinematicParameters(),RobotSide.LEFT);
   private final AcsellFourbarCalculator leftFourbar;
   private final AcsellFourbarCalculator rightFourbar;
   private EnumMap<WandererJoint, OneDoFJoint> wholeBodyControlJoints;

   public WandererOutputProcessor(FullRobotModel controllerFullRobotModel)
   {
      wholeBodyControlJoints = WandererUtil.createJointMap(controllerFullRobotModel.getOneDoFJoints());

      for (WandererActuator actuator : WandererActuator.values)
      {
         predictedMotorPower.put(actuator, new DoubleYoVariable(actuator.getName() + "PredictedMotorPower", registry));
      }
      leftFourbar = new AcsellFourbarCalculator(new WandererFourbarProperties(), "leftOutputProcessor", RobotSide.LEFT, registry);
      rightFourbar = new AcsellFourbarCalculator(new WandererFourbarProperties(), "rightOutputProcessor", RobotSide.RIGHT, registry);
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
      for (WandererJoint joint : WandererJoint.values)
      {
         double actuatorTau = 0;
         OneDoFJoint oneDoFJoint = wholeBodyControlJoints.get(joint);
         if (joint.isLinear())
         {
            actuatorTau = oneDoFJoint.getTau() / joint.getRatio();
            WandererActuator actuator = joint.getActuators()[0];
            predictedMotorPower.get(actuator).set(calcPower(actuatorTau, actuator.getKm()));
         }
      }

      updateKnees(WandererJoint.LEFT_KNEE_Y, WandererActuator.LEFT_KNEE, WandererJoint.RIGHT_KNEE_Y, WandererActuator.RIGHT_KNEE);
      updateAnkle(rightAnkleCalculator, WandererJoint.RIGHT_ANKLE_X, WandererJoint.RIGHT_ANKLE_Y, WandererActuator.RIGHT_ANKLE_LEFT, WandererActuator.RIGHT_ANKLE_RIGHT);
      updateAnkle(leftAnkleCalculator, WandererJoint.LEFT_ANKLE_X, WandererJoint.LEFT_ANKLE_Y, WandererActuator.LEFT_ANKLE_LEFT, WandererActuator.LEFT_ANKLE_RIGHT);
      

      double sumTotalMotorPower = 0;
      for (WandererActuator actuator : WandererActuator.values)
      {
         sumTotalMotorPower += predictedMotorPower.get(actuator).getDoubleValue();
      }
      totalPredictedRobotPower.set(sumTotalMotorPower);
      totalPredictedRobotPowerAverage.update();
   }
   
   private void updateKnees(WandererJoint leftJoint, WandererActuator leftActuator,WandererJoint rightJoint, WandererActuator rightActuator)
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
   
   private void updateAnkle(AcsellAnkleAngleCalculator ankleCalculator, WandererJoint ankleX, WandererJoint ankleY, WandererActuator ankleLeftActuator, WandererActuator ankleRightActuator)
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
