package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommandType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationTrajectoryCommand;
import us.ihmc.convexOptimization.quadraticProgram.InverseMatrixCalculator;
import us.ihmc.matrixlib.NativeMatrix;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This is the wrapper class for the quadratic program solver. It receives all the MPC Commands, converting them to quadratic costs or linear constraints.
 * It then submits these costs and constraints to a quadratic program solver, and computes the optimal solution. It then provides these solution coefficients
 * as an output.
 */
public class ImplicitSE3MPCQPSolver extends LinearMPCQPSolver
{
   private final ImplicitSE3MPCIndexHandler indexHandler;

   private final YoDouble firstOrientationVariableRegularization = new YoDouble("firstOrientationVariableRegularization", registry);
   private final YoDouble secondOrientationVariableRegularization = new YoDouble("secondOrientationVariableRegularization", registry);
   private final YoDouble firstOrientationRateVariableRegularization = new YoDouble("firstOrientationRateVariableRegularization", registry);
   private final YoDouble secondOrientationRateVariableRegularization = new YoDouble("secondOrientationRateVariableRegularization", registry);

   private final OrientationTrajectoryInputCalculator orientationInputCalculator;

   public ImplicitSE3MPCQPSolver(ImplicitSE3MPCIndexHandler indexHandler, double dt, double gravityZ, double mass, YoRegistry parentRegistry)
   {
      this(indexHandler,
           dt,
           gravityZ,
           mass,
           new BlockInverseCalculator(indexHandler,
                                      i ->
                                      {
                                         int variables = indexHandler.getRhoCoefficientsInSegment(i) + LinearMPCIndexHandler.comCoefficientsPerSegment;
                                         if (i > 0)
                                            variables += ImplicitSE3MPCIndexHandler.variablesPerOrientationTick;
                                         return variables;
                                      }),
           parentRegistry);
   }

   public ImplicitSE3MPCQPSolver(ImplicitSE3MPCIndexHandler indexHandler, double dt, double gravityZ, double mass, InverseMatrixCalculator<NativeMatrix> inverseCalculator, YoRegistry parentRegistry)
   {
      super(indexHandler, dt, gravityZ, inverseCalculator, parentRegistry);

      this.indexHandler = indexHandler;

      orientationInputCalculator = new OrientationTrajectoryInputCalculator(indexHandler, mass, gravityZ);

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

      for (int segmentId = 1; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         int startVar = indexHandler.getOrientationStartIndices(segmentId);


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

      double firstOrientationCoefficientFactor = dt * dt / firstOrientationRateVariableRegularization.getDoubleValue();
      double secondOrientationCoefficientFactor = dt * dt / secondOrientationRateVariableRegularization.getDoubleValue();

      for (int segmentId = 1; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         int startVar = indexHandler.getOrientationStartIndices(segmentId);

         for (int var = startVar; var < startVar + 3; var++)
         {
            solverInput_H.add(var, var, firstOrientationCoefficientFactor);
            solverInput_f.add(var, 0, -solverOutput.get(var, 0) / firstOrientationCoefficientFactor);

            solverInput_H.add(var + 3, var + 3, secondOrientationCoefficientFactor);
            solverInput_f.add(var + 3, 0, -solverOutput.get(var + 3, 0) / secondOrientationCoefficientFactor);
         }
      }
   }

   public void submitMPCCommand(MPCCommand<?> command)
   {
       if (command.getCommandType() == MPCCommandType.ORIENTATION_TRAJECTORY)
      {
         submitOrientationTrajectoryCommand((OrientationTrajectoryCommand) command);
         return;
      }

      super.submitMPCCommand(command);
   }


   public void submitOrientationTrajectoryCommand(OrientationTrajectoryCommand command)
   {
      boolean success = orientationInputCalculator.computeConstraintForNextSegmentStart(qpInputTypeA, command);
      if (success)
         addInput(qpInputTypeA);
      for (int tick = 0; tick < command.getNumberOfTicksInSegment(); tick++)
      {
         success = orientationInputCalculator.computeErrorMinimizationCommand(tick, qpInputTypeA, command);
         if (success)
            addInput(qpInputTypeA);
      }

   }
}
