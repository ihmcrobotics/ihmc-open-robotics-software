package us.ihmc.steppr.hardware.controllers;

import java.util.EnumMap;

import us.ihmc.simulationconstructionset.robotController.OutputProcessor;
import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.steppr.hardware.StepprUtil;
import us.ihmc.steppr.hardware.state.StepprAnkleAngleCalculator;
import us.ihmc.steppr.hardware.state.StepprAnkleInterpolator;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
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

   private final StepprAnkleAngleCalculator ankleInterpolator = new StepprAnkleInterpolator();
   private EnumMap<StepprJoint, OneDoFJoint> wholeBodyControlJoints;

   public StepprOutputProcessor(FullRobotModel controllerFullRobotModel)
   {
      wholeBodyControlJoints = StepprUtil.createJointMap(controllerFullRobotModel.getOneDoFJoints());

      for (StepprActuator actuator : StepprActuator.values)
      {
         predictedMotorPower.put(actuator, new DoubleYoVariable(actuator.getName() + "PredictedMotorPower", registry));
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
         double actuatorTau = 0, actuatorQd = 0;
         OneDoFJoint oneDoFJoint = wholeBodyControlJoints.get(joint);
         if (joint.isLinear() || joint == StepprJoint.LEFT_KNEE_Y || joint == StepprJoint.RIGHT_KNEE_Y)
         {
            actuatorTau = oneDoFJoint.getTau() / joint.getRatio();
            actuatorQd = oneDoFJoint.getQd() * joint.getRatio();
            StepprActuator actuator = joint.getActuators()[0];
            predictedMotorPower.get(actuator).set(calcPower(actuatorTau, actuatorQd, actuator.getKm()));
         }
      }

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

   private void updateAnkle(StepprJoint ankleX, StepprJoint ankleY, StepprActuator ankleLeftActuator, StepprActuator ankleRightActuator)
   {

      OneDoFJoint oneDofAnkleX = wholeBodyControlJoints.get(ankleX);
      OneDoFJoint oneDofAnkleY = wholeBodyControlJoints.get(ankleY);
      
      ankleInterpolator.updateAnkleState(oneDofAnkleX, oneDofAnkleY);
      //double qRightMotor = ankleInterpolator.calculateRightMotorAngle(oneDofAnkleX.getQ(), oneDofAnkleY.getQ());
      //double qLeftMotor = ankleInterpolator.calculateLeftMotorAngle(oneDofAnkleX.getQ(), oneDofAnkleY.getQ());

      //ankleInterpolator.calculateDesiredTau(qRightMotor, qLeftMotor, oneDofAnkleX.getTau(), oneDofAnkleY.getTau());
      //ankleInterpolator.calculateActuatordQd(qRightMotor, qLeftMotor, oneDofAnkleX.getQd(), oneDofAnkleY.getQd());

      predictedMotorPower.get(ankleRightActuator).set(
            calcPower(ankleInterpolator.getComputedTauRightActuator(), ankleInterpolator.getComputedMotorVelocityRight(), ankleRightActuator.getKm()));

      predictedMotorPower.get(ankleLeftActuator).set(
            calcPower(ankleInterpolator.getComputedTauLeftActuator(), ankleInterpolator.getComputedMotorVelocityLeft(), ankleLeftActuator.getKm()));

   }

   private double calcPower(double tau, double qd, double km)
   {
      return square(tau / km);
   }

   private static double square(double v)
   {
      return v * v;
   }
}
