package us.ihmc.simulationToolkit.controllers;

import us.ihmc.commons.MathTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointLowLevelJointControlSimulator implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry;
   private final PIDController jointPositionController;
   private final PIDController jointVelocityController;

   private final double controlDT;
   private final OneDegreeOfFreedomJoint simulatedJoint;
   private final OneDoFJointBasics controllerJoint;
   private final JointDesiredOutputReadOnly jointDesiredOutput;

   public JointLowLevelJointControlSimulator(OneDegreeOfFreedomJoint simulatedJoint, OneDoFJointBasics highLevelControllerOutputJoint,
                                             JointDesiredOutputReadOnly jointDesiredOutput, boolean isUpperBodyJoint,
                                             boolean isBackJoint, boolean isExoJoint, double totalMass, double controlDT)
   {
      registry = new YoRegistry(simulatedJoint.getName() + name);
      this.controlDT = controlDT;
      jointPositionController = new PIDController(simulatedJoint.getName() + "PositionControllerSimulator", registry);
      jointVelocityController = new PIDController(simulatedJoint.getName() + "VelocityControllerSimulator", registry);

      this.simulatedJoint = simulatedJoint;
      this.controllerJoint = highLevelControllerOutputJoint;
      this.jointDesiredOutput = jointDesiredOutput;

      double integralLeakRatio = 1.0; // 0.996;

      if (isUpperBodyJoint)
      {
         double subtreeMass = TotalMassCalculator.computeSubTreeMass(highLevelControllerOutputJoint.getSuccessor());
         jointPositionController.setProportionalGain(15.0 * subtreeMass);
         jointPositionController.setIntegralGain(35.0 * subtreeMass);
         jointPositionController.setMaxIntegralError(0.3);
         jointPositionController.setIntegralLeakRatio(integralLeakRatio);
         jointPositionController.setDerivativeGain(2.0 * subtreeMass);
         jointPositionController.setMaximumOutputLimit(40.0 * subtreeMass);

         jointVelocityController.setProportionalGain(2.0 * subtreeMass);
         jointVelocityController.setIntegralGain(35.0 * subtreeMass);
         jointVelocityController.setMaxIntegralError(0.3);
         jointVelocityController.setIntegralLeakRatio(integralLeakRatio);
         jointVelocityController.setMaximumOutputLimit(40.0 * subtreeMass);
      }
      else if (isBackJoint)
      {
         double subtreeMass = TotalMassCalculator.computeSubTreeMass(highLevelControllerOutputJoint.getSuccessor());
         jointPositionController.setProportionalGain(145.0 * subtreeMass);
         jointPositionController.setIntegralGain(9.0 * 50.0 * subtreeMass);
         jointPositionController.setMaxIntegralError(0.2);
         jointPositionController.setIntegralLeakRatio(integralLeakRatio);
         jointPositionController.setDerivativeGain(4.5 * subtreeMass);
         jointPositionController.setMaximumOutputLimit(3.5 * subtreeMass);

         jointVelocityController.setProportionalGain(4.5 * subtreeMass);
         jointVelocityController.setIntegralGain(9.0 * 50.0 * subtreeMass);
         jointVelocityController.setMaxIntegralError(0.2);
         jointVelocityController.setIntegralLeakRatio(integralLeakRatio);
         jointVelocityController.setMaximumOutputLimit(3.5 * subtreeMass);
      }
      else if (isExoJoint)
      {
         jointPositionController.setProportionalGain(8000.0);
         jointPositionController.setIntegralGain(1000.0 * 50.0);
         jointPositionController.setMaxIntegralError(0.2);
         jointPositionController.setIntegralLeakRatio(integralLeakRatio);
         jointPositionController.setDerivativeGain(200.0);
         jointPositionController.setMaximumOutputLimit(400.0);

         jointVelocityController.setProportionalGain(200.0);
         jointVelocityController.setIntegralGain(1000.0 * 50.0);
         jointVelocityController.setMaxIntegralError(0.2);
         jointVelocityController.setIntegralLeakRatio(integralLeakRatio);
         jointVelocityController.setMaximumOutputLimit(400.0);
      }
      else
      {
         jointPositionController.setProportionalGain(63.0 * totalMass);
         jointPositionController.setIntegralGain(6.0 * 50.0 * totalMass);
         jointPositionController.setMaxIntegralError(0.2);
         jointPositionController.setIntegralLeakRatio(integralLeakRatio);
         //jointPositionController.setDerivativeGain(5.0 * totalMass);
         jointPositionController.setDerivativeGain(0.1 * totalMass);
         jointPositionController.setMaximumOutputLimit(2.5 * totalMass);

         jointVelocityController.setProportionalGain(3.2 * totalMass);
         jointVelocityController.setIntegralGain(6.4 * 50.0 * totalMass);
         jointVelocityController.setMaxIntegralError(0.2);
         jointVelocityController.setIntegralLeakRatio(integralLeakRatio);
         jointVelocityController.setMaximumOutputLimit(2.6 * totalMass);
      }
   }

   @Override
   public void doControl()
   {
      if (jointDesiredOutput != null && jointDesiredOutput.getControlMode() == JointDesiredControlMode.POSITION)
      {
         double currentPosition = simulatedJoint.getQYoVariable().getDoubleValue();
         double desiredPosition = jointDesiredOutput.hasDesiredPosition() ? jointDesiredOutput.getDesiredPosition() : currentPosition;
         double currentRate = simulatedJoint.getQDYoVariable().getDoubleValue();
         double desiredRate = jointDesiredOutput.hasDesiredVelocity() ? jointDesiredOutput.getDesiredVelocity() : 0.0;

         if (jointDesiredOutput.hasStiffness())
            jointPositionController.setProportionalGain(jointDesiredOutput.getStiffness());
         if (jointDesiredOutput.hasDamping())
            jointPositionController.setDerivativeGain(jointDesiredOutput.getDamping());

         if (jointDesiredOutput.hasPositionFeedbackMaxError())
         {
            double error = desiredPosition - currentPosition;

            if (Math.abs(error) > jointDesiredOutput.getPositionFeedbackMaxError())
            {
               currentPosition = desiredPosition - MathTools.clamp(error, jointDesiredOutput.getPositionFeedbackMaxError());
            }
         }

         if (jointDesiredOutput.hasVelocityFeedbackMaxError())
         {
            double error = desiredRate - currentRate;

            if (Math.abs(error) > jointDesiredOutput.getVelocityFeedbackMaxError())
            {
               currentRate = desiredRate - MathTools.clamp(error, jointDesiredOutput.getVelocityFeedbackMaxError());
            }
         }

         if (jointDesiredOutput.hasStiffness() || jointDesiredOutput.hasDamping())
         {
            jointPositionController.setIntegralGain(0.0);
            jointPositionController.setMaximumOutputLimit(Double.POSITIVE_INFINITY);
         }

         double desiredTau = jointPositionController.compute(currentPosition, desiredPosition, currentRate, desiredRate, controlDT);
         simulatedJoint.setTau(desiredTau);
      }

      /*
      if (lowLevelJointData.getControlMode() == LowLevelJointControlMode.VELOCITY_CONTROL)
      {
         double currentRate = simulatedJoint.getQDYoVariable().getDoubleValue();
         double desiredRate = lowLevelJointData.getDesiredVelocity();
         double desiredTau = jointPositionController.compute(currentRate, desiredRate, 0.0, 0.0, controlDT);
         simulatedJoint.setTau(desiredTau);
      }
      */
   }

   public void resetIntegrator()
   {
      jointPositionController.resetIntegrator();
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
}
