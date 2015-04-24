package us.ihmc.steppr.hardware.controllers;

import java.util.EnumMap;

import us.ihmc.acsell.fourbar.StepprFourbarProperties;
import us.ihmc.acsell.hardware.state.AcsellAnkleAngleCalculator;
import us.ihmc.acsell.hardware.state.AcsellAnkleInterpolator;
import us.ihmc.acsell.hardware.state.AcsellFourbarCalculator;
import us.ihmc.simulationconstructionset.robotController.OutputProcessor;
import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.StepprUtil;
import us.ihmc.steppr.hardware.configuration.StepprAnkleKinematicParameters;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.filters.SimpleMovingAverageFilteredYoVariable;

public class StepprOutputProcessor implements OutputProcessor
{

   private final YoVariableRegistry registry = new YoVariableRegistry("StepprOutputProcessor");
   private final EnumMap<StepprActuator, DoubleYoVariable> predictedMotorPower = new EnumMap<StepprActuator, DoubleYoVariable>(StepprActuator.class);
   private final DoubleYoVariable totalPredictedRobotPower = new DoubleYoVariable("totalPredictedMotorPower", registry);
   private final SimpleMovingAverageFilteredYoVariable totalPredictedRobotPowerAverage = new SimpleMovingAverageFilteredYoVariable(
         totalPredictedRobotPower.getName() + "MovingAverage", 1000, totalPredictedRobotPower, registry);

   private final AcsellAnkleAngleCalculator ankleInterpolator = new AcsellAnkleInterpolator(new StepprAnkleKinematicParameters());
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
         double actuatorTau = 0;//, actuatorQd = 0;
         OneDoFJoint oneDoFJoint = wholeBodyControlJoints.get(joint);
         if (joint.isLinear())
         {
            actuatorTau = oneDoFJoint.getTau() / joint.getRatio();
            //actuatorQd = oneDoFJoint.getQd() * joint.getRatio();
            StepprActuator actuator = joint.getActuators()[0];
            //predictedMotorPower.get(actuator).set(calcPower(actuatorTau, actuatorQd, actuator.getKm()));
            predictedMotorPower.get(actuator).set(calcPower(actuatorTau, actuator.getKm()));
         }
      }

      updateKnees(StepprJoint.LEFT_KNEE_Y, StepprActuator.LEFT_KNEE, StepprJoint.RIGHT_KNEE_Y, StepprActuator.RIGHT_KNEE);
      updateAnkle(StepprJoint.LEFT_ANKLE_X, StepprJoint.LEFT_ANKLE_Y, StepprActuator.LEFT_ANKLE_LEFT, StepprActuator.LEFT_ANKLE_RIGHT);
      updateAnkle(StepprJoint.RIGHT_ANKLE_X, StepprJoint.RIGHT_ANKLE_Y, StepprActuator.RIGHT_ANKLE_LEFT, StepprActuator.RIGHT_ANKLE_RIGHT);
      

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
            calcPower(oneDofLeftKnee.getTau() / leftFourbar.getFourbarRatio() / leftJoint.getRatio(), leftActuator.getKm()));

      predictedMotorPower.get(rightActuator).set(
              calcPower(oneDofRightKnee.getTau() / rightFourbar.getFourbarRatio() / rightJoint.getRatio(), rightActuator.getKm()));

   }
   
   private void updateAnkle(StepprJoint ankleX, StepprJoint ankleY, StepprActuator ankleLeftActuator, StepprActuator ankleRightActuator)
   {

      OneDoFJoint oneDofAnkleX = wholeBodyControlJoints.get(ankleX);
      OneDoFJoint oneDofAnkleY = wholeBodyControlJoints.get(ankleY);
      
      ankleInterpolator.updateAnkleState(oneDofAnkleX, oneDofAnkleY);
      //double qRightMotor = ankleInterpolator.calculateRightMotorAngle(oneDofAnkleX.getQ(), oneDofAnkleY.getQ());
      //double qLeftMotor = ankleInterpolator.calculateLeftMotorAngle(oneDofAnkleX.getQ(), oneDofAnkleY.getQ());

      //ankleInterpolator.calculateDesiredTau(qRightMotor, qLeftMotor, oneDofAnkleX.getTau(), oneDofAnkleY.getTau());
      //ankleInterpolator.calculateActuatordQd(qRightMotor, qLeftMotor, oneDofAnkleX.getQd(), oneDofAnkleY.getQd());

/*      predictedMotorPower.get(ankleRightActuator).set(
            calcPower(ankleInterpolator.getComputedTauRightActuator(), ankleInterpolator.getComputedMotorVelocityRight(), ankleRightActuator.getKm()));

      predictedMotorPower.get(ankleLeftActuator).set(
            calcPower(ankleInterpolator.getComputedTauLeftActuator(), ankleInterpolator.getComputedMotorVelocityLeft(), ankleLeftActuator.getKm()));
*/
      predictedMotorPower.get(ankleRightActuator).set(
              calcPower(ankleInterpolator.getComputedTauRightActuator(), ankleRightActuator.getKm()));

        predictedMotorPower.get(ankleLeftActuator).set(
              calcPower(ankleInterpolator.getComputedTauLeftActuator(), ankleLeftActuator.getKm()));

   }

/*   @Deprecated //qd is unnecessary
   private double calcPower(double tau, double qd, double km)
   {
      return square(tau / km);
   }
*/
   
   private double calcPower(double tau, double km)
   {
      return square(tau / km);
   }

   private static double square(double v)
   {
      return v * v;
   }
}
