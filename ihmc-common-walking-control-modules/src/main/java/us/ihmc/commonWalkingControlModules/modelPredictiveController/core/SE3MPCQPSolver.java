package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeC;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * This is the wrapper class for the quadratic program solver. It receives all the MPC Commands, converting them to quadratic costs or linear constraints.
 * It then submits these costs and constraints to a quadratic program solver, and computes the optimal solution. It then provides these solution coefficients
 * as an output.
 */
public class SE3MPCQPSolver extends LinearMPCQPSolver
{
   private final SE3MPCIndexHandler indexHandler;

   private final YoDouble orientationVariableRegularization = new YoDouble("orientationVariableRegularization", registry);
   private final YoDouble orientationRateVariableRegularization = new YoDouble("orientationRateVariableRegularization", registry);

   private final OrientationInputCalculator orientationInputCalculator;

   public SE3MPCQPSolver(SE3MPCIndexHandler indexHandler, double dt, double gravityZ, double mass, YoRegistry parentRegistry)
   {
      this(indexHandler, dt, gravityZ, mass, true, parentRegistry);
   }

   public SE3MPCQPSolver(SE3MPCIndexHandler indexHandler, double dt, double gravityZ, double mass, boolean useBlockInverse, YoRegistry parentRegistry)
   {
      super(indexHandler, dt, gravityZ, useBlockInverse, parentRegistry);
      this.indexHandler = indexHandler;

      orientationInputCalculator = new OrientationInputCalculator(indexHandler, mass, gravityZ);

      orientationVariableRegularization.set(1e-5);
      orientationRateVariableRegularization.set(1e-6);

   }

   public void addValueRegularization()
   {
      super.addValueRegularization();

      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         int start = indexHandler.getOrientationStartIndices(segmentId);
         int end = start + indexHandler.getOrientationTicksInSegment(segmentId) * SE3MPCIndexHandler.variablesPerOrientationTick;
         for (int i = start; i < end; i++)
            solverInput_H.add(i, i, orientationVariableRegularization.getDoubleValue());
      }
   }

   public void addRateRegularization()
   {
      super.addRateRegularization();

      double orientationCoefficientFactor = dt * dt / orientationRateVariableRegularization.getDoubleValue();

      for (int segmentId = 0; segmentId < indexHandler.getNumberOfSegments(); segmentId++)
      {
         int start = indexHandler.getOrientationStartIndices(segmentId);
         int end = start + indexHandler.getOrientationTicksInSegment(segmentId) * SE3MPCIndexHandler.variablesPerOrientationTick;
         for (int i = start; i < end; i++)
         {
            solverInput_H.add(i, i, 1.0 / orientationCoefficientFactor);
            solverInput_f.add(i, 0, -solverOutput.get(start + i, 0) / orientationCoefficientFactor);
         }
      }
   }

   public void submitMPCCommand(MPCCommand<?> command)
   {
      if (command.getCommandType() == MPCCommandType.ORIENTATION_DYNAMICS)
      {
         submitOrientationDynamicsCommand((DiscreteOrientationCommand) command);
         return;
      }

      super.submitMPCCommand(command);
   }

   public void submitOrientationDynamicsCommand(DiscreteOrientationCommand command)
   {
      boolean success = orientationInputCalculator.compute(qpInputTypeA, command);
      if (success)
         addInput(qpInputTypeA);
   }
}
