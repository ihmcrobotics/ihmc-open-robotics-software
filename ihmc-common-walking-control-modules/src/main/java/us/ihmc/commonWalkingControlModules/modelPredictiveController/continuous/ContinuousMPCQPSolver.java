package us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommandList;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.LinearMPCQPSolver;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ContinuousMPCQPSolver extends LinearMPCQPSolver
{
   private final YoDouble orientationCoefficientRegularization = new YoDouble("orientationCoefficientRegularization", registry);

   private final YoDouble orientationRateCoefficientRegularization = new YoDouble("orientationRateCoefficientRegularization", registry);

   private final ContinuousMPCIndexHandler indexHandler;
   private final ContinuousMPCQPInputCalculator inputCalculator;

   private final double dt;

   public ContinuousMPCQPSolver(ContinuousMPCIndexHandler indexHandler, double dt, double mass, double gravityZ, YoRegistry parentRegistry)
   {
      super(indexHandler, dt, gravityZ, parentRegistry);
      this.indexHandler = indexHandler;
      this.dt = dt;

      orientationCoefficientRegularization.set(1e-5);

      orientationRateCoefficientRegularization.set(1e-6);

      inputCalculator = new ContinuousMPCQPInputCalculator(indexHandler, mass, gravityZ);

   }

   public void setOrientationCoefficientRegularization(double weight)
   {
      this.orientationCoefficientRegularization.set(weight);
   }

   public void setOrientationRateCoefficientRegularization(double weight)
   {
      this.orientationRateCoefficientRegularization.set(weight);
   }

   @Override
   protected void addValueRegularization()
   {
      super.addValueRegularization();
      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         int  start = indexHandler.getYawCoefficientsStartIndex(segmentId);
            int end = start + ContinuousMPCIndexHandler.orientationCoefficientsPerSegment;
            for (int i = start; i < end; i++)
               solverInput_H.add(i, i, orientationCoefficientRegularization.getDoubleValue());
            start = indexHandler.getPitchCoefficientsStartIndex(segmentId);
            end = start + ContinuousMPCIndexHandler.orientationCoefficientsPerSegment;
            for (int i = start; i < end; i++)
               solverInput_H.add(i, i, orientationCoefficientRegularization.getDoubleValue());
            start = indexHandler.getRollCoefficientsStartIndex(segmentId);
            end = start + ContinuousMPCIndexHandler.orientationCoefficientsPerSegment;
            for (int i = start; i < end; i++)
               solverInput_H.add(i, i, orientationCoefficientRegularization.getDoubleValue());
      }
   }

   @Override
   protected void addRateRegularization()
   {
      super.addRateRegularization();
   }

   @Override
   public void submitMPCCommand(MPCCommand<?> command)
   {
      switch (command.getCommandType())
      {
         case VALUE:
            submitMPCValueObjective((MPCValueCommand) command);
            break;
         case CONTINUITY:
            submitContinuityObjective((MPCContinuityCommand) command);
            break;
         case ORIENTATION_TRACKING:
            submitOrientationTrackingCommand((OrientationTrackingCommand) command);
            break;
         case ORIENTATION_DYNAMICS:
            submitOrientationDynamicsCommand((OrientationDynamicsCommand) command);
            break;
         default:
            super.submitMPCCommand(command);
      }
   }

   @Override
   public void submitMPCValueObjective(MPCValueCommand command)
   {
      boolean success = inputCalculator.calculateValueObjective(qpInputTypeA, command);
      if (success)
         addInput(qpInputTypeA);
   }

   @Override
   public void submitContinuityObjective(MPCContinuityCommand command)
   {
      boolean success = inputCalculator.calculateContinuityObjective(qpInputTypeA, command);
      if (success)
         addInput(qpInputTypeA);
   }

   public void submitOrientationTrackingCommand(OrientationTrackingCommand command)
   {
      boolean success = inputCalculator.calculateOrientationTrackingObjective(qpInputTypeC, command);
      if (success)
         addInput(qpInputTypeC);
   }

   public void submitCubicTrackingCommand(AngleTrackingCommand command)
   {
      boolean success = inputCalculator.calculateAngleTrackingCommand(qpInputTypeC, command);
      if (success)
         addInput(qpInputTypeC);
   }

   public void submitOrientationDynamicsCommand(OrientationDynamicsCommand command)
   {
      boolean success = inputCalculator.calculateOrientationDynamicsObjective(qpInputTypeA, command);
      if (success)
         addInput(qpInputTypeA);
   }
}
