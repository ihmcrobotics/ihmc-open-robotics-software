package us.ihmc.simulationToolkit.controllers;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointLowLevelControlSimulator implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry;
   private final PIDController jointPositionController;

   private final double controlDT;
   private final OneDegreeOfFreedomJoint simulatedJoint;
   private final JointDesiredOutputReadOnly jointDesiredOutput;

   public JointLowLevelControlSimulator(OneDegreeOfFreedomJoint simulatedJoint, JointDesiredOutputReadOnly jointDesiredOutput, double controlDT)
   {
      registry = new YoRegistry(simulatedJoint.getName() + name);
      this.controlDT = controlDT;
      jointPositionController = new PIDController(simulatedJoint.getName() + "PositionControllerSimulator", registry);
      this.simulatedJoint = simulatedJoint;
      this.jointDesiredOutput = jointDesiredOutput;
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
         desiredTau = EuclidCoreTools.clamp(desiredTau, simulatedJoint.getTorqueLimit());

         simulatedJoint.setTau(desiredTau);
      }
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

   public OneDegreeOfFreedomJoint getSimulatedJoint()
   {
      return simulatedJoint;
   }

   public String getJointName()
   {
      return simulatedJoint.getName();
   }
}
