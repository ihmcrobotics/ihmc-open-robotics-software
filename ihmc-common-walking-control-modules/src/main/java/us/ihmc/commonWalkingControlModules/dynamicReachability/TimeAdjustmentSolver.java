package us.ihmc.commonWalkingControlModules.dynamicReachability;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.configurations.DynamicReachabilityParameters;
import us.ihmc.convexOptimization.quadraticProgram.*;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
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

   private final JavaQuadProgSolver javaSolver = new JavaQuadProgSolver();

   private final DynamicReachabilityParameters dynamicReachabilityParameters;

   private final double minimumInitialTransferDuration;
   private final double minimumEndTransferDuration;
   private final double minimumInitialSwingDuration;
   private final double minimumEndSwingDuration;
   private final double minimumTransferDuration;
   private final double maximumTransferDuration;
   private final double minimumSwingDuration;
   private final double maximumSwingDuration;

   private double desiredParallelAdjustment;

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_h;

   private final DenseMatrix64F segmentAdjustmentObjective_H;
   private final DenseMatrix64F stepAdjustmentObjective_H;
   private final DenseMatrix64F perpendicularObjective_H;
   private final DenseMatrix64F parallelObjective_H;
   private final DenseMatrix64F parallelObjective_h;
   private final DenseMatrix64F equalAdjustmentObjective_H;

   private final DenseMatrix64F parallel_J;
   private final DenseMatrix64F perpendicular_J;
   private final DenseMatrix64F equalAdjustment_J;
   private final DenseMatrix64F tempTrans_J;

   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;

   private final DenseMatrix64F solution;

   private final boolean useHigherOrderSteps;

   private int numberOfFootstepsToConsider;
   private int numberOfFootstepsRegistered;
   private int numberOfHigherSteps;
   private int numberOfIterations;

   public TimeAdjustmentSolver(int maxNumberOfFootstepsToConsider, DynamicReachabilityParameters dynamicReachabilityParameters)
   {
      this.dynamicReachabilityParameters = dynamicReachabilityParameters;
      this.useHigherOrderSteps = dynamicReachabilityParameters.useHigherOrderSteps();

      minimumInitialTransferDuration = dynamicReachabilityParameters.getMinimumInitialTransferDuration();
      minimumEndTransferDuration = dynamicReachabilityParameters.getMinimumEndTransferDuration();
      minimumInitialSwingDuration = dynamicReachabilityParameters.getMinimumInitialSwingDuration();
      minimumEndSwingDuration = dynamicReachabilityParameters.getMinimumEndSwingDuration();

      minimumTransferDuration = dynamicReachabilityParameters.getMinimumTransferDuration();
      maximumTransferDuration = dynamicReachabilityParameters.getMaximumTransferDuration();
      minimumSwingDuration = dynamicReachabilityParameters.getMinimumSwingDuration();
      maximumSwingDuration = dynamicReachabilityParameters.getMaximumSwingDuration();

      int problemSize = 6 + 2 * (maxNumberOfFootstepsToConsider - 3);

      solverInput_H = new DenseMatrix64F(problemSize, problemSize);
      solverInput_h = new DenseMatrix64F(problemSize, 1);

      solverInput_Ain = new DenseMatrix64F(12, problemSize);
      solverInput_bin = new DenseMatrix64F(12, 1);

      solution = new DenseMatrix64F(problemSize, 1);

      parallel_J = new DenseMatrix64F(1, problemSize);
      perpendicular_J = new DenseMatrix64F(1, problemSize);
      equalAdjustment_J = new DenseMatrix64F(3, problemSize);
      tempTrans_J = new DenseMatrix64F(problemSize, problemSize);

      segmentAdjustmentObjective_H = new DenseMatrix64F(problemSize, problemSize);
      stepAdjustmentObjective_H = new DenseMatrix64F(problemSize, problemSize);

      perpendicularObjective_H = new DenseMatrix64F(problemSize, problemSize);
      equalAdjustmentObjective_H = new DenseMatrix64F(problemSize, problemSize);

      parallelObjective_H = new DenseMatrix64F(problemSize, problemSize);
      parallelObjective_h = new DenseMatrix64F(problemSize, 1);
   }

   /**
    * Sets the number of footsteps to consider, which reflects the number considered in the
    * {@link us.ihmc.commonWalkingControlModules.capturePoint.ICPPlannerInterface}. This is used to set whether or not higher steps will be considered.
    *
    * @param numberOfFootstepsToConsider
    */
   public void setNumberOfFootstepsToConsider(int numberOfFootstepsToConsider)
   {
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
   }

   /**
    * Sets the number of footsteps that have been submitted to the planner. This is used to determine whether or not adjusting the timing of higher steps
    * will be considered.
    *
    * @param numberOfFootstepsRegistered
    */
   public void setNumberOfFootstepsRegistered(int numberOfFootstepsRegistered)
   {
      this.numberOfFootstepsRegistered = numberOfFootstepsRegistered;
   }

   /**
    * Resets the size of the problem. Before using this, {@link #setNumberOfFootstepsToConsider(int)} ()} and {@link #setNumberOfFootstepsRegistered(int)} must
    * be called, as this determines the problem size.
    */
   public void reshape()
   {
      int problemSize = 6;

      if (numberOfFootstepsToConsider > 3 & numberOfFootstepsRegistered > 1 && useHigherOrderSteps)
      {
         numberOfHigherSteps = computeHigherOrderSteps();
         problemSize += 2 * numberOfHigherSteps;
      }

      solution.reshape(problemSize, 1);

      segmentAdjustmentObjective_H.reshape(problemSize, problemSize);
      stepAdjustmentObjective_H.reshape(problemSize, problemSize);
      perpendicularObjective_H.reshape(problemSize, problemSize);
      parallelObjective_H.reshape(problemSize, problemSize);
      equalAdjustmentObjective_H.reshape(problemSize, problemSize);
      parallelObjective_h.reshape(problemSize, 1);

      parallel_J.reshape(1, problemSize);
      perpendicular_J.reshape(1, problemSize);
      equalAdjustment_J.reshape(3, problemSize);

      solverInput_H.reshape(problemSize, problemSize);
      solverInput_h.reshape(problemSize, 1);

      solverInput_Ain.reshape(problemSize + 6, problemSize);
      solverInput_bin.reshape(problemSize + 6, 1);

      solution.zero();

      segmentAdjustmentObjective_H.zero();
      stepAdjustmentObjective_H.zero();
      perpendicularObjective_H.zero();
      parallelObjective_H.zero();
      equalAdjustmentObjective_H.zero();
      parallelObjective_h.zero();

      parallel_J.zero();
      perpendicular_J.zero();

      solverInput_H.zero();
      solverInput_h.zero();

      solverInput_Ain.zero();
      solverInput_bin.zero();
   }

   private int computeHigherOrderSteps()
   {
      if (useHigherOrderSteps)
      {
         return Math.min(numberOfFootstepsToConsider - 3, numberOfFootstepsRegistered - 1);
      }
      else
      {
         return 0;
      }
   }

   public void zeroInputs()
   {
      solution.zero();

      segmentAdjustmentObjective_H.zero();
      stepAdjustmentObjective_H.zero();
      perpendicularObjective_H.zero();
      parallelObjective_H.zero();
      parallelObjective_h.zero();
      equalAdjustmentObjective_H.zero();

      solverInput_H.zero();
      solverInput_h.zero();
   }

   /**
    * Sets the gradient of adjusting the time of the current transfer phase that is spent on the previous exit CMP.
    * This represents the time spent loading the current foot.
    *
    * @param gradient gradient of time segment. Not Modified.
    */
   public void setCurrentInitialTransferGradient(FrameVector3D gradient)
   {
      setGradient(currentInitialTransferIndex, gradient);
   }

   /**
    * Sets the gradient of adjusting the time of the current transfer phase that is spent on the rear CMP of the upcoming stance foot.
    * This represents the time of transfer spent moving the CoP from the rear of the foot to the front of the foot.
    *
    * @param gradient gradient of the time segment. Not Modified.
    */
   public void setCurrentEndTransferGradient(FrameVector3D gradient)
   {
      setGradient(currentEndTransferIndex, gradient);
   }

   /**
    * Sets the gradient of adjusting the time of the current swing phase that is spent on the rear CMP of the stance foot.
    * This represents the time of swing spent moving the CoP from the rear of the foot to the front of the foot.
    *
    * @param gradient gradient of the time segment. Not Modified.
    */
   public void setCurrentInitialSwingGradient(FrameVector3D gradient)
   {
      setGradient(currentInitialSwingIndex, gradient);
   }

   /**
    * Sets the gradient of adjusting the time of the current swing phase that is spent on the front CMP of the stance foot.
    * This represents the time of swing spent with the CoP in the front of the foot.
    *
    * @param gradient gradient of the time segment. Not Modified.
    */
   public void setCurrentEndSwingGradient(FrameVector3D gradient)
   {
      setGradient(currentEndSwingIndex, gradient);
   }

   /**
    * Sets the gradient of adjusting the time of the next transfer phase that is spent on the current stance foot's toe CMP.
    * This represents the time spent moving the robot weight from the current stance foot to the next stance foot.
    *
    * @param gradient gradient of the time segment. Not Modified.
    */
   public void setNextInitialTransferGradient(FrameVector3D gradient)
   {
      setGradient(nextInitialTransferIndex, gradient);
   }

   /**
    * Sets the gradient of adjusting the time of the next transfer phase that is spent on the next foot's heel CMP.
    *
    * @param gradient gradient of the time segment. Not Modified.
    */
   public void setNextEndTransferGradient(FrameVector3D gradient)
   {
      setGradient(nextEndTransferIndex, gradient);
   }

   /**
    * Sets the gradient of adjusting the time of the swing duration of upcoming steps.
    * In this case, {@param higherIndex} 0 represents the second swing phase, as that is the current swing index, and is set using
    * {@link #setCurrentInitialSwingGradient(FrameVector3D)} and {@link @setCurrentEndSwingGradient(FrameVector)}.
    *
    * @param higherIndex index of the swing phase to modify.
    * @param swingGradient gradient of the swing time. Not Modified.
    */
   public void setHigherSwingGradient(int higherIndex, FrameVector3D swingGradient)
   {
      setGradient(6 + 2 * higherIndex, swingGradient);
   }

   /**
    * Sets the gradient of adjusting the time of the transfer duration of upcoming steps.
    * In this case, {@param higherIndex} of 0 represents the third transfer phase. Transfer phase 0 is addressed using
    * {@link #setCurrentInitialTransferGradient(FrameVector3D)} and {@link #setCurrentEndTransferGradient(FrameVector3D)},
    * and transfer phase 1 is addressed using {@link #setNextInitialTransferGradient(FrameVector3D)} and
    * {@link #setNextEndTransferGradient(FrameVector3D)}.
    *
    * @param higherIndex index of the transfer phase to modify.
    * @param transferGradient gradient of the transfer time. Not Modified.
    */
   public void setHigherTransferGradient(int higherIndex, FrameVector3D transferGradient)
   {
      setGradient(7 + 2 * higherIndex, transferGradient);
   }

   private void setGradient(int colIndex, FrameVector3D gradient)
   {
      double parallel = gradient.getX();
      double perpendicular = gradient.getY();

      parallel_J.set(0, colIndex, parallel);
      perpendicular_J.set(0, colIndex, perpendicular);
   }

   /**
    * Sets the desired CoM parallel adjustment to be achieved by adjusting the timing.
    * @param adjustment desired adjustment in the direction of the step (m). Not Modified.
    */
   public void setDesiredParallelAdjustment(double adjustment)
   {
      desiredParallelAdjustment = adjustment;
   }

   /**
    * Sets the duration of the current transfer phase, as well as the ratio of the time spent on the initial phase.
    *
    * @param duration time in transfer phase.
    * @param alpha ratio of time spent on initial portion.
    */
   public void setCurrentTransferDuration(double duration, double alpha)
   {
      double initialDuration = alpha * duration;
      double endDuration = (1.0 - alpha) * duration;

      solverInput_bin.set(0, 0, -minimumTransferDuration + duration);
      solverInput_bin.set(3, 0, maximumTransferDuration - duration);
      solverInput_bin.set(6, 0, -(minimumInitialTransferDuration - initialDuration));
      solverInput_bin.set(7, 0, -(minimumEndTransferDuration - endDuration));
   }

   /**
    * Sets the duration of the current swing phase, as well as the ratio of the time spent on the initial phase.
    *
    * @param duration time in swing phase.
    * @param alpha ratio of time spent on initial portion.
    */
   public void setCurrentSwingDuration(double duration, double alpha)
   {
      double initialDuration = alpha * duration;
      double endDuration = (1.0 - alpha) * duration;

      solverInput_bin.set(1, 0, -minimumSwingDuration + duration);
      solverInput_bin.set(4, 0, maximumSwingDuration - duration);

      solverInput_bin.set(8, 0, -(minimumInitialSwingDuration - initialDuration));
      solverInput_bin.set(9, 0, -(minimumEndSwingDuration - endDuration));
   }

   /**
    * Sets the duration of the next transfer phase, as well as the ratio of the time spent on the initial phase.
    *
    * @param duration time in transfer phase.
    * @param alpha ratio of time spent on initial portion.
    */
   public void setNextTransferDuration(double duration, double alpha)
   {
      double initialDuration = alpha * duration;
      double endDuration = (1.0 - alpha) * duration;

      solverInput_bin.set(2, 0, -minimumTransferDuration + duration);
      solverInput_bin.set(5, 0, maximumTransferDuration - duration);
      solverInput_bin.set(10, 0, -(minimumInitialTransferDuration - initialDuration));
      solverInput_bin.set(11, 0, -(minimumEndTransferDuration - endDuration));
   }

   /**
    * Sets the duration of the swing phase of step {@param higherIndex}. In this case, {@param higherIndex} of 0 represents the
    * duration of the second step, as the first swing is handled by {@link #setCurrentSwingDuration(double, double)}.
    *
    * @param higherIndex step number to set.
    * @param duration swing duration to set.
    */
   public void setHigherSwingDuration(int higherIndex, double duration)
   {
      solverInput_Ain.set(12 + 2 * higherIndex, 6 + 2 * higherIndex, -1.0);
      solverInput_bin.set(12 + 2 * higherIndex, 0, -(minimumSwingDuration - duration));
   }

   /**
    * Sets the duration of the transfer phase of step {@param higherIndex}. In this case, {@param higherIndex} of 0 represents the
    * duration of the third transfer phase, as the first transfer is handled by {@link #setCurrentTransferDuration(double, double)}
    * and the second transfer is handled by {@link #setNextTransferDuration(double, double)}.
    *
    * @param higherIndex step number to set.
    * @param duration swing duration to set.
    */
   public void setHigherTransferDuration(int higherIndex, double duration)
   {
      solverInput_Ain.set(13 + 2 * higherIndex, 7 + 2 * higherIndex, -1.0);
      solverInput_bin.set(13 + 2 * higherIndex, 0, -(minimumTransferDuration - duration));
   }

   /**
    * <p>
    * Computes the desired step phase timing adjustment using the QP. If it fails, the adjustment reverts to 0.
    * </p>
    * <p>
    * Before calling this, {@link #reshape()} must first be called.
    * </p>
    * <p>
    *    After calling {@link #reshape()}, all the step timings and gradients must be submitted. These include:
    * </p>
    *
    * @throws NoConvergenceException
    */
   public void compute() throws NoConvergenceException
   {
      zeroInputs();

      // compute objectives
      double scalarCost = computeDesiredAdjustmentObjective();
      computePerpendicularAdjustmentMinimizationObjective();
      computeEqualAdjustmentObjective();
      computeTimeAdjustmentMinimizationObjective();

      CommonOps.addEquals(solverInput_H, segmentAdjustmentObjective_H);
      CommonOps.addEquals(solverInput_H, stepAdjustmentObjective_H);
      CommonOps.addEquals(solverInput_H, perpendicularObjective_H);
      CommonOps.addEquals(solverInput_H, parallelObjective_H);
      CommonOps.addEquals(solverInput_H, equalAdjustmentObjective_H);

      CommonOps.addEquals(solverInput_h, parallelObjective_h);

      // define bounds and timing constraints
      assembleTimingConstraints();

      javaSolver.clear();

      javaSolver.setQuadraticCostFunction(solverInput_H, solverInput_h, scalarCost);
      javaSolver.setLinearInequalityConstraints(solverInput_Ain, solverInput_bin);

      numberOfIterations = javaSolver.solve(solution);

      if (MatrixTools.containsNaN(solution))
      {
         solution.zero();
         throw new NoConvergenceException(numberOfIterations);
      }
   }

   private double computeDesiredAdjustmentObjective()
   {
      double constraintWeight = dynamicReachabilityParameters.getConstraintWeight();

      CommonOps.multTransA(parallel_J, parallel_J, parallelObjective_H);
      CommonOps.scale(constraintWeight, parallelObjective_H);

      CommonOps.transpose(parallel_J, parallelObjective_h);
      CommonOps.scale(-desiredParallelAdjustment * constraintWeight, parallelObjective_h);

      return Math.pow(desiredParallelAdjustment, 2.0) * constraintWeight;
   }

   private void computePerpendicularAdjustmentMinimizationObjective()
   {
      double perpendicularWeight = dynamicReachabilityParameters.getPerpendicularWeight();
      CommonOps.multTransA(perpendicular_J, perpendicular_J, perpendicularObjective_H);
      CommonOps.scale(perpendicularWeight, perpendicularObjective_H);
   }

   private void computeEqualAdjustmentObjective()
   {
      double transferEqualAdjustmentWeight = dynamicReachabilityParameters.getTransferEqualAdjustmentWeight();
      double swingEqualAdjustmentWeight = dynamicReachabilityParameters.getSwingEqualAdjustmentWeight();

      equalAdjustment_J.set(0, 0, 1.0);
      equalAdjustment_J.set(0, 1, -1.0);
      equalAdjustment_J.set(1, 2, 1.0);
      equalAdjustment_J.set(1, 3, -1.0);
      equalAdjustment_J.set(2, 4, 1.0);
      equalAdjustment_J.set(2, 5, -1.0);

      tempTrans_J.reshape(equalAdjustment_J.numCols, equalAdjustment_J.numRows);
      tempTrans_J.zero();
      CommonOps.transpose(equalAdjustment_J, tempTrans_J);
      MatrixTools.scaleRow(transferEqualAdjustmentWeight, 0, equalAdjustment_J);
      MatrixTools.scaleRow(swingEqualAdjustmentWeight, 1, equalAdjustment_J);
      MatrixTools.scaleRow(transferEqualAdjustmentWeight, 2, equalAdjustment_J);

      CommonOps.mult(tempTrans_J, equalAdjustment_J, equalAdjustmentObjective_H);
   }

   private void computeTimeAdjustmentMinimizationObjective()
   {
      double transferAdjustmentWeight = dynamicReachabilityParameters.getTransferAdjustmentWeight();
      double swingAdjustmentWeight = dynamicReachabilityParameters.getSwingAdjustmentWeight();

      double totalTransferAdjustmentWeight = dynamicReachabilityParameters.getTotalTransferAdjustmentWeight();
      double totalSwingAdjustmentWeight = dynamicReachabilityParameters.getTotalSwingAdjustmentWeight();

      segmentAdjustmentObjective_H.set(0, 0, transferAdjustmentWeight);
      segmentAdjustmentObjective_H.set(1, 1, transferAdjustmentWeight);
      segmentAdjustmentObjective_H.set(2, 2, swingAdjustmentWeight);
      segmentAdjustmentObjective_H.set(3, 3, swingAdjustmentWeight);
      segmentAdjustmentObjective_H.set(4, 4, transferAdjustmentWeight);
      segmentAdjustmentObjective_H.set(5, 5, transferAdjustmentWeight);

      for (int i = 0; i < numberOfHigherSteps; i++)
      {
         segmentAdjustmentObjective_H.set(6 + 2 * i, 6 + 2 * i, swingAdjustmentWeight);
         segmentAdjustmentObjective_H.set(7 + 2 * i, 7 + 2 * i, transferAdjustmentWeight);
      }

      stepAdjustmentObjective_H.set(0, 0, totalTransferAdjustmentWeight);
      stepAdjustmentObjective_H.set(0, 1, totalTransferAdjustmentWeight);
      stepAdjustmentObjective_H.set(1, 0, totalTransferAdjustmentWeight);
      stepAdjustmentObjective_H.set(1, 1, totalTransferAdjustmentWeight);
      stepAdjustmentObjective_H.set(2, 2, totalSwingAdjustmentWeight);
      stepAdjustmentObjective_H.set(2, 3, totalSwingAdjustmentWeight);
      stepAdjustmentObjective_H.set(3, 2, totalSwingAdjustmentWeight);
      stepAdjustmentObjective_H.set(3, 3, totalSwingAdjustmentWeight);
      stepAdjustmentObjective_H.set(4, 4, totalTransferAdjustmentWeight);
      stepAdjustmentObjective_H.set(4, 5, totalTransferAdjustmentWeight);
      stepAdjustmentObjective_H.set(5, 4, totalTransferAdjustmentWeight);
      stepAdjustmentObjective_H.set(5, 5, totalTransferAdjustmentWeight);
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

      solverInput_Ain.set(6, 0, -1.0);
      solverInput_Ain.set(7, 1, -1.0);
      solverInput_Ain.set(8, 2, -1.0);
      solverInput_Ain.set(9, 3, -1.0);
      solverInput_Ain.set(10, 4, -1.0);
      solverInput_Ain.set(11, 5, -1.0);
   }

   /**
    * Returns the adjustment to the initial portion of the current transfer phase.
    *
    * @return timing adjustment (s).
    */
   public double getCurrentInitialTransferAdjustment()
   {
      return solution.get(currentInitialTransferIndex, 0);
   }

   /**
    * Returns the adjustment to the ending portion of the current transfer phase.
    *
    * @return timing adjustment (s).
    */
   public double getCurrentEndTransferAdjustment()
   {
      return solution.get(currentEndTransferIndex, 0);
   }

   /**
    * Returns the adjustment to the initial portion of the current swing phase.
    *
    * @return timing adjustment (s).
    */
   public double getCurrentInitialSwingAdjustment()
   {
      return solution.get(currentInitialSwingIndex, 0);
   }

   /**
    * Returns the adjustment to the ending portion of the current swing phase.
    *
    * @return timing adjustment (s).
    */
   public double getCurrentEndSwingAdjustment()
   {
      return solution.get(currentEndSwingIndex, 0);
   }

   /**
    * Returns the adjustment to the initial portion of the next transfer phase.
    *
    * @return timing adjustment (s).
    */
   public double getNextInitialTransferAdjustment()
   {
      return solution.get(nextInitialTransferIndex, 0);
   }

   /**
    * Returns the adjustment to the ending portion of the next transfer phase.
    *
    * @return timing adjustment (s).
    */
   public double getNextEndTransferAdjustment()
   {
      return solution.get(nextEndTransferIndex, 0);
   }

   /**
    * Returns the adjustment of the higher swing phases. In this case, {@param higherIndex} of 0 represents the second swing phase,
    * as the first swing phase is handled by {@link #getCurrentInitialSwingAdjustment()} and {@link #getCurrentEndSwingAdjustment()}.
    *
    * @param higherIndex swing phase to query.
    *
    * @return timing adjustment (s).
    */
   public double getHigherSwingAdjustment(int higherIndex)
   {
      return solution.get(nextEndTransferIndex + 2 * (higherIndex + 1) - 1, 0);
   }

   /**
    * Returns the adjustment of the higher transfer phases. In this case, {@param higherIndex} of 0 represents the third transfer phase,
    * as the first transfer phase is handled by {@link #getCurrentInitialTransferAdjustment()} and {@link #getCurrentEndTransferAdjustment()},
    * and the second transfer phase is handled by {@link #getNextInitialTransferAdjustment()} and {@link #getNextEndTransferAdjustment()}.
    *
    * @param higherIndex transfer phase to query.
    *
    * @return timing adjustment (s).
    */
   public double getHigherTransferAdjustment(int higherIndex)
   {
      return solution.get(nextEndTransferIndex + 2 * (higherIndex + 1), 0);
   }

   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }
}
