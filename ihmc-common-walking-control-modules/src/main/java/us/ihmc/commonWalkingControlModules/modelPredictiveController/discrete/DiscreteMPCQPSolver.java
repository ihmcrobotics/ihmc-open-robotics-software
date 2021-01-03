package us.ihmc.commonWalkingControlModules.modelPredictiveController.discrete;

import sun.reflect.generics.reflectiveObjects.NotImplementedException;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.LinearMPCQPSolver;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommandList;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.ContinuousMPCIndexHandler;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.ContinuousMPCQPInputCalculator;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DiscreteMPCQPSolver extends LinearMPCQPSolver
{
   private final YoDouble orientationCoefficientRegularization = new YoDouble("orientationCoefficientRegularization", registry);

   private final YoDouble orientationRateCoefficientRegularization = new YoDouble("orientationRateCoefficientRegularization", registry);

   private final DiscreteMPCIndexHandler indexHandler;
   private final DiscreteMPCQPInputCalculator inputCalculator;

   private final double dt;

   public DiscreteMPCQPSolver(DiscreteMPCIndexHandler indexHandler, double dt, double mass, double gravityZ, YoRegistry parentRegistry)
   {
      super(indexHandler, dt, gravityZ, parentRegistry);
      this.indexHandler = indexHandler;
      this.dt = dt;

      orientationCoefficientRegularization.set(1e-5);

      orientationRateCoefficientRegularization.set(1e-6);

      inputCalculator = new DiscreteMPCQPInputCalculator(indexHandler, mass, gravityZ);
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
         int vars = DiscreteMPCIndexHandler.orientationVariablesPerTick * indexHandler.getOrientationTicksInSegment(segmentId);
         int start = indexHandler.getOrientationStart(segmentId);
         int end = start + vars;
         for (int i = start; i < end; i++)
            solverInput_H.add(i, i, orientationCoefficientRegularization.getDoubleValue());
      }
   }

   @Override
   protected void addRateRegularization()
   {
      super.addRateRegularization();
   }

   public void submitMPCCommand(MPCCommand<?> command)
   {
      switch (command.getCommandType())
      {
         case ORIENTATION_TRACKING:
            submitOrientationTrackingCommand((OrientationTrackingCommand) command);
            break;
         case ORIENTATION_DYNAMICS:
            submitOrientationDynamicsCommand((DiscreteOrientationDynamicsCommand) command);
            break;
         default:
            super.submitMPCCommand(command);
      }
   }

   public void submitOrientationTrackingCommand(OrientationTrackingCommand command)
   {
      throw new NotImplementedException();
   }

   public void submitOrientationDynamicsCommand(DiscreteOrientationDynamicsCommand command)
   {
      boolean success = inputCalculator.calculateOrientationDynamicsObjective(qpInputTypeA, command);
      if (success)
         addInput(qpInputTypeA);
   }
}
