package us.ihmc.commonWalkingControlModules.dynamicReachability;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.convexOptimization.quadraticProgram.SimpleActiveSetQPSolverInterface;
import us.ihmc.convexOptimization.quadraticProgram.SimpleEfficientActiveSetQPSolver;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class TimeAdjustmentSolver
{
   private static final int currentInitialTransferIndex = 0;
   private static final int currentEndTransferIndex = 1;
   private static final int currentInitialSwingIndex = 2;
   private static final int currentEndSwingIndex = 3;
   private static final int nextInitialTransferIndex = 4;
   private static final int nextEndTransferIndex = 5;

   private static final double minimumInitialTransferDuration = 0.01;
   private static final double minimumEndTransferDuration = 0.05;
   private static final double minimumInitialSwingDuration = 0.15;
   private static final double minimumEndSwingDuration = 0.15;

   private static final double minimumTransferDuration = 0.15;
   private static final double maximumTransferDuration = 5.0;
   private static final double minimumSwingDuration = 0.4;
   private static final double maximumSwingDuration = 10.0;

   private final SimpleActiveSetQPSolverInterface activeSetSolver = new SimpleEfficientActiveSetQPSolver();

   private static final double perpendicularWeight = 0.0;
   private static final double swingAdjustmentWeight = 10.0;
   private static final double transferAdjustmentWeight = 1.0;
   private static final double constraintWeight = 1000.0;

   private final DoubleYoVariable yoMinimumInitialTransferDuration;
   private final DoubleYoVariable yoMinimumEndTransferDuration;
   private final DoubleYoVariable yoMinimumInitialSwingDuration;
   private final DoubleYoVariable yoMinimumEndSwingDuration;
   private final DoubleYoVariable yoMinimumTransferDuration;
   private final DoubleYoVariable yoMaximumTransferDuration;
   private final DoubleYoVariable yoMinimumSwingDuration;
   private final DoubleYoVariable yoMaximumSwingDuration;

   private double desiredParallelAdjustment;

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_h;

   private final DenseMatrix64F segmentAdjustmentObjective_H;
   private final DenseMatrix64F stepAdjustmentObjective_H;
   private final DenseMatrix64F perpendicularObjective_H;
   private final DenseMatrix64F parallelObjective_H;
   private final DenseMatrix64F parallelObjective_h;

   private final DenseMatrix64F parallel_J;
   private final DenseMatrix64F perpendicular_J;

   private final DenseMatrix64F solverInput_Lb;
   private final DenseMatrix64F solverInput_Ub;

   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;

   private final DenseMatrix64F solution;

   private int numberOfFootstepsToConsider;
   private int numberOfFootstepsRegistered;

   public TimeAdjustmentSolver(int maxNumberOfFootstepsToConsider, YoVariableRegistry registry)
   {
      yoMinimumInitialTransferDuration = new DoubleYoVariable("minimumInitialTransferDuration", registry);
      yoMinimumEndTransferDuration = new DoubleYoVariable("minimumEndTransferDuration", registry);
      yoMinimumInitialSwingDuration = new DoubleYoVariable("minimumInitialSwingDuration", registry);
      yoMinimumEndSwingDuration = new DoubleYoVariable("minimumEndSwingDuration", registry);
      yoMinimumTransferDuration = new DoubleYoVariable("minimumTransferDuration", registry);
      yoMaximumTransferDuration = new DoubleYoVariable("maximumTransferDuration", registry);
      yoMinimumSwingDuration = new DoubleYoVariable("minimumSwingDuration", registry);
      yoMaximumSwingDuration = new DoubleYoVariable("maximumSwingDuration", registry);

      yoMinimumInitialTransferDuration.set(minimumInitialTransferDuration);
      yoMinimumEndTransferDuration.set(minimumEndTransferDuration);
      yoMinimumInitialSwingDuration.set(minimumInitialSwingDuration);
      yoMinimumEndSwingDuration.set(minimumEndSwingDuration);

      yoMinimumTransferDuration.set(minimumTransferDuration);
      yoMaximumTransferDuration.set(maximumTransferDuration);
      yoMinimumSwingDuration.set(minimumSwingDuration);
      yoMaximumSwingDuration.set(maximumSwingDuration);

      int problemSize = 6 + 2 * (maxNumberOfFootstepsToConsider - 3);

      solverInput_H = new DenseMatrix64F(problemSize, problemSize);
      solverInput_h = new DenseMatrix64F(problemSize, 1);

      solverInput_Lb = new DenseMatrix64F(problemSize, 1);
      solverInput_Ub = new DenseMatrix64F(problemSize, 1);

      solverInput_Ain = new DenseMatrix64F(6, problemSize);
      solverInput_bin = new DenseMatrix64F(6, 1);

      solution = new DenseMatrix64F(problemSize, 1);

      parallel_J = new DenseMatrix64F(1, problemSize);
      perpendicular_J = new DenseMatrix64F(1, problemSize);

      segmentAdjustmentObjective_H = new DenseMatrix64F(problemSize, problemSize);
      stepAdjustmentObjective_H = new DenseMatrix64F(problemSize, problemSize);

      perpendicularObjective_H = new DenseMatrix64F(problemSize, problemSize);

      parallelObjective_H = new DenseMatrix64F(problemSize, problemSize);
      parallelObjective_h = new DenseMatrix64F(problemSize, 1);
   }

   public void setNumberOfFootstepsToConsider(int numberOfFootstepsToConsider)
   {
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
   }

   public void setNumberOfFootstepsRegistered(int numberOfFootstepsRegistered)
   {
      this.numberOfFootstepsRegistered = numberOfFootstepsRegistered;
   }

   public void reshape()
   {
      int problemSize = 6;

      //// TODO: 3/24/17
      /*
      if (numberOfFootstepsToConsider > 3 & numberOfFootstepsRegistered > 2)
         problemSize += 2 * Math.min(numberOfFootstepsToConsider - 3, numberOfFootstepsRegistered - 1);
         */

      solution.reshape(problemSize, problemSize);

      segmentAdjustmentObjective_H.reshape(problemSize, problemSize);
      stepAdjustmentObjective_H.reshape(problemSize, problemSize);
      perpendicularObjective_H.reshape(problemSize, problemSize);
      parallelObjective_H.reshape(problemSize, problemSize);
      parallelObjective_h.reshape(problemSize, 1);

      parallel_J.reshape(1, problemSize);
      perpendicular_J.reshape(1, problemSize);

      solverInput_H.reshape(problemSize, problemSize);
      solverInput_h.reshape(problemSize, 1);

      solverInput_Lb.reshape(problemSize, 1);
      solverInput_Ub.reshape(problemSize, 1);

      solverInput_Ain.reshape(6, problemSize);
      solverInput_bin.reshape(6, 1);

      solution.zero();

      segmentAdjustmentObjective_H.zero();
      stepAdjustmentObjective_H.zero();
      perpendicularObjective_H.zero();
      parallelObjective_H.zero();
      parallelObjective_h.zero();

      parallel_J.zero();
      perpendicular_J.zero();

      solverInput_H.zero();
      solverInput_h.zero();
      solverInput_Lb.zero();
      solverInput_Ub.zero();

      solverInput_Ain.zero();
      solverInput_bin.zero();
   }

   public void setCurrentInitialTransferGradient(FrameVector gradient)
   {
      setGradient(currentInitialTransferIndex, gradient);
   }

   public void setCurrentEndTransferGradient(FrameVector gradient)
   {
      setGradient(currentEndTransferIndex, gradient);
   }

   public void setCurrentInitialSwingGradient(FrameVector gradient)
   {
      setGradient(currentInitialSwingIndex, gradient);
   }

   public void setCurrentEndSwingGradient(FrameVector gradient)
   {
      setGradient(currentEndSwingIndex, gradient);
   }

   public void setNextInitialTransferGradient(FrameVector gradient)
   {
      setGradient(nextInitialTransferIndex, gradient);
   }

   public void setNextEndTransferGradient(FrameVector gradient)
   {
      setGradient(nextEndTransferIndex, gradient);
   }

   private void setGradient(int colIndex, FrameVector gradient)
   {
      double parallel = gradient.getX();
      double perpendicular = gradient.getY();

      parallel_J.set(0, colIndex, parallel);
      perpendicular_J.set(0, colIndex, perpendicular);
   }

   public void setDesiredParallelAdjustment(double adjustment)
   {
      desiredParallelAdjustment = adjustment;
   }

   public void setCurrentTransferDuration(double duration, double alpha)
   {
      double initialDuration = alpha * duration;
      double endDuration = (1.0 - alpha) * duration;

      solverInput_Lb.set(currentInitialTransferIndex, 0, yoMinimumInitialTransferDuration.getDoubleValue() - initialDuration);
      solverInput_Lb.set(currentEndTransferIndex, 0, yoMinimumEndTransferDuration.getDoubleValue() - endDuration);

      solverInput_bin.set(0, 0, -yoMinimumTransferDuration.getDoubleValue() + duration);
      solverInput_bin.set(3, 0, yoMaximumTransferDuration.getDoubleValue() - duration);
   }

   public void setCurrentSwingDuration(double duration, double alpha)
   {
      double initialDuration = alpha * duration;
      double endDuration = (1.0 - alpha) * duration;

      solverInput_Lb.set(currentInitialSwingIndex, 0, yoMinimumInitialSwingDuration.getDoubleValue() - initialDuration);
      solverInput_Lb.set(currentEndSwingIndex, 0, yoMinimumEndSwingDuration.getDoubleValue() - endDuration);

      solverInput_bin.set(1, 0, -yoMinimumSwingDuration.getDoubleValue() + duration);
      solverInput_bin.set(4, 0, yoMaximumSwingDuration.getDoubleValue() - duration);
   }

   public void setNextTransferDuration(double duration, double alpha)
   {
      double initialDuration = alpha * duration;
      double endDuration = (1.0 - alpha) * duration;

      solverInput_Lb.set(nextInitialTransferIndex, 0, yoMinimumInitialTransferDuration.getDoubleValue() - initialDuration);
      solverInput_Lb.set(nextEndTransferIndex, 0, yoMinimumEndTransferDuration.getDoubleValue() - endDuration);

      solverInput_bin.set(2, 0, -yoMinimumTransferDuration.getDoubleValue() + duration);
      solverInput_bin.set(5, 0, yoMaximumTransferDuration.getDoubleValue() - duration);
   }

   public void compute() throws NoConvergenceException
   {
      // compute objectives
      double scalarCost = computeDesiredAdjustmentObjective();
      computePerpendicularAdjustmentMinimizationObjective();
      computeTimeAdjustmentMinimizationObjective();

      CommonOps.add(solverInput_H, segmentAdjustmentObjective_H, solverInput_H);
      CommonOps.add(solverInput_H, stepAdjustmentObjective_H, solverInput_H);
      CommonOps.add(solverInput_H, perpendicularObjective_H, solverInput_H);
      CommonOps.add(solverInput_H, parallelObjective_H, solverInput_H);

      CommonOps.add(solverInput_h, parallelObjective_h, solverInput_h);

      // define bounds and timing constraints
      assembleVariableBounds();
      assembleTimingConstraints();

      activeSetSolver.clear();

      activeSetSolver.setQuadraticCostFunction(solverInput_H, solverInput_h, scalarCost);
      activeSetSolver.setVariableBounds(solverInput_Lb, solverInput_Ub);
      activeSetSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);

      int numberOfIterations = activeSetSolver.solve(solution);

      if (MatrixTools.containsNaN(solution))
      {
         throw new NoConvergenceException(numberOfIterations);
      }
   }

   private double computeDesiredAdjustmentObjective()
   {
      CommonOps.multTransA(parallel_J, parallel_J, parallelObjective_H);
      CommonOps.scale(constraintWeight, parallelObjective_H);

      CommonOps.transpose(parallel_J, parallelObjective_h);
      CommonOps.scale(-2.0 * desiredParallelAdjustment * constraintWeight, parallelObjective_h);

      return Math.pow(desiredParallelAdjustment, 2.0) * constraintWeight;
   }

   private void computePerpendicularAdjustmentMinimizationObjective()
   {
      CommonOps.multTransA(perpendicular_J, perpendicular_J, perpendicularObjective_H);
      CommonOps.scale(perpendicularWeight, perpendicularObjective_H);
   }

   private void computeTimeAdjustmentMinimizationObjective()
   {
      segmentAdjustmentObjective_H.set(0, 0, transferAdjustmentWeight);
      segmentAdjustmentObjective_H.set(1, 1, transferAdjustmentWeight);
      segmentAdjustmentObjective_H.set(2, 2, swingAdjustmentWeight);
      segmentAdjustmentObjective_H.set(3, 3, swingAdjustmentWeight);
      segmentAdjustmentObjective_H.set(4, 4, transferAdjustmentWeight);
      segmentAdjustmentObjective_H.set(5, 5, transferAdjustmentWeight);

      stepAdjustmentObjective_H.set(0, 0, transferAdjustmentWeight);
      stepAdjustmentObjective_H.set(0, 1, transferAdjustmentWeight);
      stepAdjustmentObjective_H.set(1, 0, transferAdjustmentWeight);
      stepAdjustmentObjective_H.set(1, 1, transferAdjustmentWeight);
      stepAdjustmentObjective_H.set(2, 2, swingAdjustmentWeight);
      stepAdjustmentObjective_H.set(2, 3, swingAdjustmentWeight);
      stepAdjustmentObjective_H.set(3, 2, swingAdjustmentWeight);
      stepAdjustmentObjective_H.set(3, 3, swingAdjustmentWeight);
      stepAdjustmentObjective_H.set(4, 4, transferAdjustmentWeight);
      stepAdjustmentObjective_H.set(4, 5, transferAdjustmentWeight);
      stepAdjustmentObjective_H.set(5, 4, transferAdjustmentWeight);
      stepAdjustmentObjective_H.set(5, 5, transferAdjustmentWeight);
   }

   private void assembleVariableBounds()
   {
      solverInput_Ub.set(0, 0, Double.POSITIVE_INFINITY);
      solverInput_Ub.set(1, 0, Double.POSITIVE_INFINITY);
      solverInput_Ub.set(2, 0, Double.POSITIVE_INFINITY);
      solverInput_Ub.set(3, 0, Double.POSITIVE_INFINITY);
      solverInput_Ub.set(4, 0, Double.POSITIVE_INFINITY);
      solverInput_Ub.set(5, 0, Double.POSITIVE_INFINITY);
   }

   private void assembleTimingConstraints()
   {
      solverInput_Ain.set(0, 0, -1.0);
      solverInput_Ain.set(0, 1, -1.0);
      solverInput_Ain.set(1, 2, -1.0);
      solverInput_Ain.set(1, 3, -1.0);
      solverInput_Ain.set(2, 4, -1.0);
      solverInput_Ain.set(2, 5, -1.0);

      solverInput_Ain.set(3, 0, 1.0);
      solverInput_Ain.set(3, 1, 1.0);
      solverInput_Ain.set(4, 2, 1.0);
      solverInput_Ain.set(4, 3, 1.0);
      solverInput_Ain.set(5, 4, 1.0);
      solverInput_Ain.set(5, 5, 1.0);
   }

   public double getCurrentInitialTransferAdjustment()
   {
      return solution.get(currentInitialTransferIndex, 0);
   }

   public double getCurrentEndTransferAdjustment()
   {
      return solution.get(currentEndTransferIndex, 0);
   }

   public double getCurrentInitialSwingAdjustment()
   {
      return solution.get(currentInitialSwingIndex, 0);
   }

   public double getCurrentEndSwingAdjustment()
   {
      return solution.get(currentEndSwingIndex, 0);
   }

   public double getNextInitialTransferAdjustment()
   {
      return solution.get(nextInitialTransferIndex, 0);
   }

   public double getNextEndTransferAdjustment()
   {
      return solution.get(nextEndTransferIndex, 0);
   }
}
