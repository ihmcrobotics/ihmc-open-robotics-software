package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.convexOptimization.quadraticProgram.InverseMatrixCalculator;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This is the wrapper class for the quadratic program solver. It receives all the MPC Commands, converting them to quadratic costs or linear constraints.
 * It then submits these costs and constraints to a quadratic program solver, and computes the optimal solution. It then provides these solution coefficients
 * as an output.
 */
public class SE3MPCQPSolver extends LinearMPCQPSolver
{
   private final SE3MPCIndexHandler indexHandler;

   private final YoDouble firstOrientationVariableRegularization = new YoDouble("firstOrientationVariableRegularization", registry);
   private final YoDouble secondOrientationVariableRegularization = new YoDouble("secondOrientationVariableRegularization", registry);
   private final YoDouble firstOrientationRateVariableRegularization = new YoDouble("firstOrientationRateVariableRegularization", registry);
   private final YoDouble secondOrientationRateVariableRegularization = new YoDouble("secondOrientationRateVariableRegularization", registry);

   private final OrientationTrajectoryInputCalculator orientationInputCalculator;

   public SE3MPCQPSolver(SE3MPCIndexHandler indexHandler, double dt, double gravityZ, YoRegistry parentRegistry)
   {
      this(indexHandler,
           dt,
           gravityZ,
           new BlockInverseCalculator(indexHandler, indexHandler::getOrientationStartIndex, indexHandler::getVariablesInSegment),
           parentRegistry);
   }

   public SE3MPCQPSolver(SE3MPCIndexHandler indexHandler, double dt, double gravityZ, InverseMatrixCalculator<NativeMatrix> inverseCalculator, YoRegistry parentRegistry)
   {
      super(indexHandler, dt, gravityZ, inverseCalculator, parentRegistry);

      this.indexHandler = indexHandler;

      orientationInputCalculator = new OrientationTrajectoryInputCalculator(indexHandler);

      firstOrientationVariableRegularization.set(1e-8);
      secondOrientationVariableRegularization.set(1e-8);
      firstOrientationRateVariableRegularization.set(1e-6);
      secondOrientationRateVariableRegularization.set(1e-6);
   }

   public void setFirstOrientationVariableRegularization(double value)
   {
      firstOrientationVariableRegularization.set(value);
   }

   public void setSecondOrientationVariableRegularization(double value)
   {
      secondOrientationVariableRegularization.set(value);
   }

   public void setFirstOrientationRateVariableRegularization(double value)
   {
      firstOrientationRateVariableRegularization.set(value);
   }

   public void setSecondOrientationRateVariableRegularization(double value)
   {
      secondOrientationRateVariableRegularization.set(value);
   }

   public void addValueRegularization()
   {
      super.addValueRegularization();

      double firstOrientationCoefficientFactor = firstOrientationVariableRegularization.getDoubleValue();
      double secondOrientationCoefficientFactor = secondOrientationVariableRegularization.getDoubleValue();

      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         int startVar = indexHandler.getOrientationStartIndex(segmentId);

         for (int var = startVar; var < startVar + 3; var++)
         {
            solverInput_H.add(var, var, firstOrientationCoefficientFactor);
            solverInput_H.add(var + 3, var + 3, secondOrientationCoefficientFactor);
         }
      }
   }

   public void addRateRegularization()
   {
      super.addRateRegularization();

      double firstOrientationCoefficientFactor = firstOrientationRateVariableRegularization.getDoubleValue() / dt2;
      double secondOrientationCoefficientFactor = secondOrientationRateVariableRegularization.getDoubleValue() / dt2;

      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         int startVar = indexHandler.getOrientationStartIndex(segmentId);

         for (int var = startVar; var < startVar + 3; var++)
         {
            if (!Double.isNaN(previousSolution.get(var, 0)))
            {
               solverInput_H.add(var, var, firstOrientationCoefficientFactor);
               solverInput_f.add(var, 0, -previousSolution.get(var, 0) * firstOrientationCoefficientFactor);
            }

            if (!Double.isNaN(previousSolution.get(var + 3, 0)))
            {
               solverInput_H.add(var + 3, var + 3, secondOrientationCoefficientFactor);
               solverInput_f.add(var + 3, 0, -previousSolution.get(var + 3, 0) * secondOrientationCoefficientFactor);
            }
         }
      }
   }

   public void submitMPCCommand(MPCCommand<?> command)
   {
      switch (command.getCommandType())
      {
         case ORIENTATION_TRAJECTORY:
            submitOrientationTrajectoryCommand((OrientationTrajectoryCommand) command);
            return;
         case ORIENTATION_CONTINUITY:
            submitOrientationContinuityCommand((OrientationContinuityCommand) command);
            return;
         case ORIENTATION_VALUE:
            submitOrientationValueCommand((OrientationValueCommand) command);
            return;
         case DIRECT_ORIENTATION_VALUE:
            submitDirectOrientationValueCommand((DirectOrientationValueCommand) command);
            return;
      }

      super.submitMPCCommand(command);
   }

   public void submitOrientationValueCommand(OrientationValueCommand command)
   {
      boolean success = orientationInputCalculator.compute(qpInputTypeA, command);
      if (success)
         addInput(qpInputTypeA);
   }

   public void submitDirectOrientationValueCommand(DirectOrientationValueCommand command)
   {
      boolean success = orientationInputCalculator.compute(qpInputTypeA, command);
      if (success)
         addInput(qpInputTypeA);
   }

   public void submitOrientationContinuityCommand(OrientationContinuityCommand command)
   {
      boolean success = orientationInputCalculator.compute(qpInputTypeA, command);
      if (success)
         addInput(qpInputTypeA);
   }

   public void submitOrientationTrajectoryCommand(OrientationTrajectoryCommand command)
   {
      for (int tick = 0; tick < command.getNumberOfTicksInSegment(); tick++)
      {
         boolean success = orientationInputCalculator.compute(tick, qpInputTypeA, command);
         if (success)
            addInput(qpInputTypeA);
      }
   }
}
