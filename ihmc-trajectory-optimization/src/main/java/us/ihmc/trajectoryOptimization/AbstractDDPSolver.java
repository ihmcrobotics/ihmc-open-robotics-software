package us.ihmc.trajectoryOptimization;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.lists.RecyclingArrayList;

import java.util.List;

public abstract class AbstractDDPSolver<E extends Enum> implements DDPSolverInterface<E>
{
   private static final boolean useDynamicsHessian = false;

   protected final DiscreteHybridDynamics<E> dynamics;

   protected final DiscreteOptimizationData optimalSequence;
   protected final DiscreteOptimizationData desiredSequence;
   protected final DiscreteOptimizationData updatedSequence;

   protected final DiscreteSequence constantsSequence;

   protected final DiscreteSequence feedBackGainSequence;
   protected final DiscreteSequence feedForwardSequence;

   protected final RecyclingArrayList<DenseMatrix64F> costStateGradientSequence;
   protected final RecyclingArrayList<DenseMatrix64F> costControlGradientSequence;
   protected final RecyclingArrayList<DenseMatrix64F> costStateHessianSequence;
   protected final RecyclingArrayList<DenseMatrix64F> costControlHessianSequence;
   protected final RecyclingArrayList<DenseMatrix64F> costStateControlHessianSequence;

   protected final DenseMatrix64F hamiltonianStateGradient;
   protected final DenseMatrix64F hamiltonianControlGradient;
   protected final DenseMatrix64F hamiltonianStateHessian;
   protected final DenseMatrix64F hamiltonianControlHessian;
   protected final DenseMatrix64F hamiltonianStateControlHessian;
   protected final DenseMatrix64F hamiltonianControlStateHessian;
   protected final DenseMatrix64F invQuu;

   protected final RecyclingArrayList<DenseMatrix64F> dynamicsStateGradientSequence;
   protected final RecyclingArrayList<DenseMatrix64F> dynamicsControlGradientSequence;
   protected final DenseMatrix64F dynamicsStateHessian;
   protected final DenseMatrix64F dynamicsControlHessian;
   protected final DenseMatrix64F dynamicsControlStateHessian;

   protected final RecyclingArrayList<DenseMatrix64F> valueStateGradientSequence;
   protected final RecyclingArrayList<DenseMatrix64F> valueStateHessianSequence;

   private final DenseMatrix64F Q_XX_col;
   private final DenseMatrix64F Q_UX_col;
   private final DenseMatrix64F Q_UU_col;

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);

   private final SingularValueDecomposition<DenseMatrix64F> decomposer = DecompositionFactory.svd(0, 0, true, true, true);
   protected final boolean debug;

   private static final double minimumModificationFactor = 2;
   private static final double lambdaMax = 1000;
   protected static final double lambdaMin = 1e-10;

   private double modificationFactor = 1e-10;
   protected double lambda = lambdaMin;

   protected double lineSearchGain = 1.0;

   public AbstractDDPSolver(DiscreteHybridDynamics<E> dynamics, boolean debug)
   {
      this.dynamics = dynamics;
      this.debug = debug;

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();
      int constantSize = dynamics.getConstantVectorSize();

      optimalSequence = new DiscreteOptimizationSequence(stateSize, controlSize);
      desiredSequence = new DiscreteOptimizationSequence(stateSize, controlSize);
      updatedSequence = new DiscreteOptimizationSequence(stateSize, controlSize);

      constantsSequence = new DiscreteSequence(constantSize, 1);

      feedBackGainSequence = new DiscreteSequence(controlSize, stateSize);
      feedForwardSequence = new DiscreteSequence(controlSize, 1);

      valueStateGradientSequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(stateSize, 1));
      valueStateHessianSequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(stateSize, stateSize));

      costStateGradientSequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(stateSize, 1));
      costControlGradientSequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(controlSize, 1));
      costStateHessianSequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(stateSize, stateSize));
      costControlHessianSequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(controlSize, controlSize));
      costStateControlHessianSequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(stateSize, controlSize));

      hamiltonianStateGradient = new DenseMatrix64F(stateSize, 1);
      hamiltonianControlGradient = new DenseMatrix64F(controlSize, 1);
      hamiltonianStateHessian = new DenseMatrix64F(stateSize, stateSize);
      hamiltonianControlHessian = new DenseMatrix64F(controlSize, controlSize);
      hamiltonianStateControlHessian = new DenseMatrix64F(stateSize, controlSize);
      hamiltonianControlStateHessian = new DenseMatrix64F(controlSize, stateSize);
      invQuu = new DenseMatrix64F(controlSize, controlSize);

      dynamicsStateGradientSequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(stateSize, stateSize));
      dynamicsControlGradientSequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(stateSize, controlSize));
      dynamicsStateHessian = new DenseMatrix64F(stateSize, stateSize);
      dynamicsControlHessian = new DenseMatrix64F(stateSize, controlSize);
      dynamicsControlStateHessian = new DenseMatrix64F(stateSize, controlSize);

      valueStateHessianSequence.clear();
      valueStateGradientSequence.clear();

      feedBackGainSequence.clear();
      feedForwardSequence.clear();

      Q_XX_col = new DenseMatrix64F(stateSize, 1);
      Q_UX_col = new DenseMatrix64F(controlSize, 1);
      Q_UU_col = new DenseMatrix64F(controlSize, 1);
   }

   @Override
   public void initializeFromLQRSolution(E dynamicsState, LQTrackingCostFunction<E> costFunction, DiscreteOptimizationData trajectory,
                                         DiscreteOptimizationData desiredSequence, DiscreteSequence constantsSequence,
                                         DiscreteSequence feedBackGainSequence, DiscreteSequence feedForwardSequence)
   {
      this.feedBackGainSequence.clear();
      this.feedForwardSequence.clear();

      valueStateHessianSequence.clear();
      valueStateGradientSequence.clear();

      costStateGradientSequence.clear();
      costControlGradientSequence.clear();
      costStateHessianSequence.clear();
      costControlHessianSequence.clear();
      costStateControlHessianSequence.clear();

      dynamicsControlGradientSequence.clear();
      dynamicsStateGradientSequence.clear();

      this.optimalSequence.set(trajectory);
      this.desiredSequence.set(desiredSequence);
      this.updatedSequence.setZero(trajectory);
      this.feedBackGainSequence.set(feedBackGainSequence);
      this.feedForwardSequence.set(feedForwardSequence);
      this.constantsSequence.set(constantsSequence);

      for (int i = 0; i < trajectory.size(); i++)
      {
         valueStateHessianSequence.add().zero();
         valueStateGradientSequence.add().zero();

         costStateGradientSequence.add().zero();
         costControlGradientSequence.add().zero();
         costStateHessianSequence.add().zero();
         costControlHessianSequence.add().zero();
         costStateControlHessianSequence.add().zero();

         dynamicsControlGradientSequence.add().zero();
         dynamicsStateGradientSequence.add().zero();
      }

      // FIXME
      forwardPass(dynamicsState, 0, optimalSequence.size() - 1, costFunction, optimalSequence.getState(0), updatedSequence);

      optimalSequence.set(updatedSequence);
   }

   @Override
   public void initializeSequencesFromDesireds(DenseMatrix64F initialState, DiscreteOptimizationData desiredSequence, DiscreteSequence constantsSequence)
   {
      this.feedBackGainSequence.clear();
      this.feedForwardSequence.clear();

      valueStateHessianSequence.clear();
      valueStateGradientSequence.clear();

      costStateGradientSequence.clear();
      costControlGradientSequence.clear();
      costStateHessianSequence.clear();
      costControlHessianSequence.clear();
      costStateControlHessianSequence.clear();

      dynamicsControlGradientSequence.clear();
      dynamicsStateGradientSequence.clear();

      this.optimalSequence.set(desiredSequence);
      this.desiredSequence.set(desiredSequence);
      this.updatedSequence.setZero(desiredSequence);
      this.constantsSequence.set(constantsSequence);

      this.feedBackGainSequence.setLength(desiredSequence.size());
      this.feedForwardSequence.setLength(desiredSequence.size());

      for (int i = 0; i < desiredSequence.size(); i++)
      {
         valueStateHessianSequence.add().zero();
         valueStateGradientSequence.add().zero();

         costStateGradientSequence.add().zero();
         costControlGradientSequence.add().zero();
         costStateHessianSequence.add().zero();
         costControlHessianSequence.add().zero();
         costStateControlHessianSequence.add().zero();

         dynamicsControlGradientSequence.add().zero();
         dynamicsStateGradientSequence.add().zero();
      }

      optimalSequence.setState(0, initialState);
   }

   public void computeFunctionApproximations(E dynamicsState, LQTrackingCostFunction<E> costFunction, int startIndex, int endIndex)
   {
      for (int t = startIndex; t <= endIndex; t++)
      {
         DenseMatrix64F currentState = optimalSequence.getState(t);
         DenseMatrix64F currentControl = optimalSequence.getControl(t);
         DenseMatrix64F desiredState = desiredSequence.getState(t);
         DenseMatrix64F desiredControl = desiredSequence.getControl(t);
         DenseMatrix64F constants = constantsSequence.get(t);

         dynamics.getDynamicsStateGradient(dynamicsState, currentState, currentControl, constants, dynamicsStateGradientSequence.get(t));
         dynamics.getDynamicsControlGradient(dynamicsState, currentState, currentControl, constants, dynamicsControlGradientSequence.get(t));

         costFunction.getCostStateGradient(dynamicsState, currentControl, currentState, desiredControl, desiredState, constants, costStateGradientSequence.get(t));
         costFunction.getCostControlGradient(dynamicsState, currentControl, currentState, desiredControl, desiredState, constants, costControlGradientSequence.get(t));
         costFunction.getCostStateHessian(dynamicsState, currentControl, currentState, constants, costStateHessianSequence.get(t));
         costFunction.getCostControlHessian(dynamicsState, currentControl, currentState, constants, costControlHessianSequence.get(t));
         costFunction.getCostControlGradientOfStateGradient(dynamicsState, currentControl, currentState, constants, costStateControlHessianSequence.get(t));
      }
   }


   private final DenseMatrix64F U = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F W = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F V = new DenseMatrix64F(0, 0);
   boolean computeFeedbackGainAndFeedForwardTerms(DenseMatrix64F hamiltonianControlGradient, DenseMatrix64F hamiltonianControlHessian,
                                                  DenseMatrix64F hamiltonianControlStateHessian, DenseMatrix64F feedbackGainToPack,
                                                  DenseMatrix64F feedForwardControlToPack)
   {
      // insure that the hessian is positive definite
      int controlSize = hamiltonianControlHessian.numCols;
      U.reshape(controlSize, controlSize);
      W.reshape(controlSize, controlSize);
      V.reshape(controlSize, controlSize);
      tempMatrix.reshape(controlSize, controlSize);

      decomposer.decompose(hamiltonianControlHessian);
      decomposer.getU(U, true);
      decomposer.getW(W);
      decomposer.getV(V, false);

      MatrixTools.addDiagonal(W, lambda);
      for (int i = 0; i < W.getNumRows(); i++)
      {
         if (W.get(i, i) <= 0.0)
         {
            if (debug) PrintTools.info("Hessian is not positive definite, aborting backward pass.");
            return false;
         }
      }

      DiagonalMatrixTools.invertDiagonalMatrix(W);
      CommonOps.mult(V, W, tempMatrix);
      invQuu.reshape(controlSize, controlSize);
      CommonOps.mult(tempMatrix, U, invQuu);

      // K = -inv(Quu) Qux
      CommonOps.mult(-1.0, invQuu, hamiltonianControlStateHessian, feedbackGainToPack);

      // du = -inv(Quu) Qu
      CommonOps.mult(-1.0, invQuu, hamiltonianControlGradient, feedForwardControlToPack);

      return true;
   }

   void updateHamiltonianApproximations(E dynamicsState, int t, DenseMatrix64F costStateGradient, DenseMatrix64F costControlGradient, DenseMatrix64F costStateHessian,
                                        DenseMatrix64F costControlHessian, DenseMatrix64F costStateControlHessian, DenseMatrix64F dynamicsStateGradient,
                                        DenseMatrix64F dynamicsControlGradient, DenseMatrix64F valueStateGradient, DenseMatrix64F valueStateHessian,
                                        DenseMatrix64F hamiltonianStateGradientToPack, DenseMatrix64F hamiltonianControlGradientToPack,
                                        DenseMatrix64F hamiltonianStateHessianToPack, DenseMatrix64F hamiltonianControlHessianToPack,
                                        DenseMatrix64F hamiltonianStateControlHessianToPack, DenseMatrix64F hamiltonianControlStateHessianToPack)
   {
      // Qx = Lx + A' Vx
      hamiltonianStateGradientToPack.set(costStateGradient);
      CommonOps.multAddTransA(dynamicsStateGradient, valueStateGradient, hamiltonianStateGradientToPack);

      // Qu = Lu + B' Vx
      hamiltonianControlGradientToPack.set(costControlGradient);
      CommonOps.multAddTransA(dynamicsControlGradient, valueStateGradient, hamiltonianControlGradientToPack);

      // Qxx = Lxx + A' Vxx A
      hamiltonianStateHessianToPack.set(costStateHessian);
      addMultQuad(dynamicsStateGradient, valueStateHessian, dynamicsStateGradient, hamiltonianStateHessianToPack);

      // Qxu = Lxu + A' Vxx B
      hamiltonianStateControlHessianToPack.set(costStateControlHessian);
      addMultQuad(dynamicsStateGradient, valueStateHessian, dynamicsControlGradient, hamiltonianStateControlHessianToPack);

      // Quu = Luu + B' Vxx B
      hamiltonianControlHessianToPack.set(costControlHessian);
      addMultQuad(dynamicsControlGradient, valueStateHessian, dynamicsControlGradient, hamiltonianControlHessianToPack);


      if (useDynamicsHessian)
      {
         DenseMatrix64F currentState = optimalSequence.getState(t);
         DenseMatrix64F currentControl = optimalSequence.getControl(t);
         DenseMatrix64F constants = constantsSequence.get(t);

         for (int stateIndex = 0; stateIndex < dynamics.getStateVectorSize(); stateIndex++)
         {
            dynamics.getDynamicsStateHessian(dynamicsState, stateIndex, currentState, currentControl, constants, dynamicsStateHessian);
            dynamics.getDynamicsStateGradientOfControlGradient(dynamicsState, stateIndex, currentState, currentControl, constants, dynamicsControlStateHessian);

            CommonOps.multTransA(dynamicsStateHessian, valueStateGradient, Q_XX_col);
            MatrixTools.addMatrixBlock(hamiltonianStateHessianToPack, 0, stateIndex, Q_XX_col, 0, 0, dynamics.getStateVectorSize(), 1, 1.0);

            CommonOps.multTransA(dynamicsControlStateHessian, valueStateGradient, Q_UX_col);
            MatrixTools.addMatrixBlock(hamiltonianControlStateHessianToPack, 0, stateIndex, Q_UX_col, 0, 0, dynamics.getControlVectorSize(), 1, 1.0);
         }

         for (int controlIndex = 0; controlIndex < dynamics.getControlVectorSize(); controlIndex++)
         {
            dynamics.getDynamicsControlHessian(dynamicsState, controlIndex, currentState, currentControl, constants, dynamicsControlHessian);

            CommonOps.multTransA(dynamicsControlHessian, valueStateGradient, Q_UU_col);
            MatrixTools.addMatrixBlock(hamiltonianControlHessianToPack, 0, controlIndex, Q_UU_col, 0, 0, dynamics.getControlVectorSize(), 1, 1.0);
         }

         throw new RuntimeException("This doesn't work properly.");
      }

      // Qux = Qxu'
      CommonOps.transpose(hamiltonianStateControlHessianToPack, hamiltonianControlStateHessianToPack);
   }


   private final DenseMatrix64F stateError = new DenseMatrix64F(0, 0);
   void computeUpdatedControl(DenseMatrix64F currentState, DenseMatrix64F updatedState, DenseMatrix64F feedbackGainMatrix, DenseMatrix64F feedforwardControl,
                              DenseMatrix64F currentControl, DenseMatrix64F updatedControlToPack)
   {
      stateError.reshape(currentState.getNumRows(), 1);
      CommonOps.subtract(updatedState, currentState, stateError);

      // u += K*(xhat - x)
      CommonOps.mult(feedbackGainMatrix, stateError, updatedControlToPack);

      // u = alpha * du + uref
      CommonOps.addEquals(updatedControlToPack, lineSearchGain, feedforwardControl);
      CommonOps.addEquals(updatedControlToPack, currentControl);
   }

   void computePreviousValueApproximation(DenseMatrix64F hamiltonianStateGradient, DenseMatrix64F hamiltonianControlGradient,
                                          DenseMatrix64F hamiltonianStateHessian, DenseMatrix64F hamiltonianStateControlHessian,
                                          DenseMatrix64F feedbackGainMatrix, DenseMatrix64F previousValueStateGradientToPack,
                                          DenseMatrix64F previousValueStateHessianToPack)
   {
      // Vx = Qx + K' Qu
      previousValueStateGradientToPack.set(hamiltonianStateGradient);
      CommonOps.multAddTransA(feedbackGainMatrix, hamiltonianControlGradient, previousValueStateGradientToPack);

      // Vxx = Qxx + Qxu K
      previousValueStateHessianToPack.set(hamiltonianStateHessian);
      CommonOps.multAdd(hamiltonianStateControlHessian, feedbackGainMatrix, previousValueStateHessianToPack);
   }

   /**
    * D = D + A^T *  B * C
    */
   void addMultQuad(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F C, DenseMatrix64F DToPack)
   {
      tempMatrix.reshape(A.numCols, B.numCols);
      CommonOps.multTransA(A, B, tempMatrix);
      CommonOps.multAdd(tempMatrix, C, DToPack);
   }

   /**
    * D = D + alpha * A^T *  B * C
    */
   void addMultQuad(double alpha, DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F C, DenseMatrix64F DToPack)
   {
      tempMatrix.reshape(A.numCols, B.numCols);
      CommonOps.multTransA(alpha, A, B, tempMatrix);
      CommonOps.multAdd(tempMatrix, C, DToPack);
   }

   void applyLevenbergMarquardtHeuristicForHessianRegularization(boolean success)
   {
      if (success)
      {
         modificationFactor = Math.min(1.0 / minimumModificationFactor, modificationFactor / minimumModificationFactor);
         lambda = Math.max(modificationFactor * lambda, lambdaMin);
      }
      else
      {
         modificationFactor = Math.max(minimumModificationFactor, modificationFactor * minimumModificationFactor);
         lambda = MathTools.clamp(lambda * modificationFactor, lambdaMin, lambdaMax);
      }
   }

   @Override
   public DiscreteOptimizationData getOptimalSequence()
   {
      return optimalSequence;
   }

   @Override
   public int computeSequence(E dynamicsState, LQTrackingCostFunction<E> costFunction, LQTrackingCostFunction<E> terminalCostFunction)
   {
      double cost, cost0;
      cost0 = Double.MAX_VALUE;

      int startIndex = 0;
      int endIndex = optimalSequence.size() - 1;

      for (int iterations = 0; iterations < 20; iterations++)
      {
         boolean lastIteration = false;
         computeFunctionApproximations(dynamicsState, costFunction, startIndex, endIndex);

         for (int iterB = 0; iterB < 20; iterB++)
         {
            boolean hessianWasPD = backwardPass(dynamicsState, startIndex, endIndex, terminalCostFunction, optimalSequence);

            if (hessianWasPD)
            {
               cost = forwardPass(dynamicsState, startIndex, endIndex, costFunction, optimalSequence.getState(0), updatedSequence);
               optimalSequence.set(updatedSequence);

               if (Math.abs(cost - cost0) / Math.abs(cost0) < 1e-1)
                  return iterations;

               cost0 = cost;

               applyLevenbergMarquardtHeuristicForHessianRegularization(true);
               break;
            }
            else
            {
               if (lastIteration)
                  break;

               applyLevenbergMarquardtHeuristicForHessianRegularization(false);
            }

            if (lambda == lambdaMin)
               lastIteration = true;
         }

      }

      throw new RuntimeException("Didn't converge.");
   }


   @Override
   public int computeSequence(List<E> dynamicsStates, List<LQTrackingCostFunction<E>> costFunctions, List<LQTrackingCostFunction<E>> terminalCostFunctions,
                              TIntArrayList startIndices, TIntArrayList endIndices)
   {
      double cost, cost0;
      cost0 = Double.MAX_VALUE;

      for (int iterations = 0; iterations < 20; iterations++)
      {
         boolean lastIteration = false;
         for (int segment = dynamicsStates.size() - 1; segment >= 0; segment--)
            computeFunctionApproximations(dynamicsStates.get(segment), costFunctions.get(segment), startIndices.get(segment), endIndices.get(segment));

         for (int iterB = 0; iterB < 20; iterB++)
         {
            boolean hessianWasPD = true;
            for (int segment = dynamicsStates.size() - 1; segment >= 0; segment--)
            {
               hessianWasPD = backwardPass(dynamicsStates.get(segment), startIndices.get(segment), endIndices.get(segment),
                                           terminalCostFunctions.get(segment), optimalSequence);
               if (!hessianWasPD)
                  break;
            }

            if (hessianWasPD)
            {
               cost = 0;
               for (int segment = 0; segment < dynamicsStates.size(); segment++)
               {
                  int startIndex = startIndices.get(segment);
                  cost += forwardPass(dynamicsStates.get(segment), startIndex, endIndices.get(segment), costFunctions.get(segment),
                                      optimalSequence.getState(startIndex), updatedSequence);
               }
               optimalSequence.set(updatedSequence);

               if (Math.abs(cost - cost0) / Math.abs(cost0) < 1e-1)
                  return iterations;

               cost0 = cost;

               applyLevenbergMarquardtHeuristicForHessianRegularization(true);
               break;
            }
            else
            {
               if (lastIteration)
                  break;

               if (debug) PrintTools.info("Dynamics are not positive definite");
               applyLevenbergMarquardtHeuristicForHessianRegularization(false);
            }

            if (lambda == lambdaMin)
               lastIteration = true;
         }

      }

      throw new RuntimeException("Didn't converge.");
   }

   @Override
   public void computeOnePass(List<E> dynamicsStates, List<LQTrackingCostFunction<E>> costFunctions, List<LQTrackingCostFunction<E>> terminalCostFunctions,
                              TIntArrayList startIndices, TIntArrayList endIndices)
   {

      boolean lastIteration = false;
      for (int segment = 0; segment < dynamicsStates.size(); segment++)
         computeFunctionApproximations(dynamicsStates.get(segment), costFunctions.get(segment), startIndices.get(segment), endIndices.get(segment));

      for (int iterB = 0; iterB < 20; iterB++)
      {
         boolean hessianWasPD = true;
         for (int segment = dynamicsStates.size() - 1; segment >= 0; segment--)
         {
            hessianWasPD = backwardPass(dynamicsStates.get(segment), startIndices.get(segment), endIndices.get(segment),
                                        terminalCostFunctions.get(segment), optimalSequence);
            if (!hessianWasPD)
               break;
         }

         if (hessianWasPD)
         {
            // set initial state
            updatedSequence.setState(0, optimalSequence.getState(0));

            for (int segment = 0; segment < dynamicsStates.size(); segment++)
            {
               int startIndex = startIndices.get(segment);
               int endIndex = endIndices.get(segment);
               DenseMatrix64F initialStateForSegment = updatedSequence.getState(startIndex);
               forwardPass(dynamicsStates.get(segment), startIndex, endIndex, costFunctions.get(segment), initialStateForSegment, updatedSequence);
            }
            optimalSequence.set(updatedSequence);

            applyLevenbergMarquardtHeuristicForHessianRegularization(true);
            break;
         }
         else
         {
            if (lastIteration)
               break;

            if (debug) PrintTools.info("Dynamics are not positive definite");
            applyLevenbergMarquardtHeuristicForHessianRegularization(false);
         }

         if (lambda == lambdaMin)
            lastIteration = true;
      }

   }

   @Override
   public abstract double forwardPass(E dynamicsState, int startIndex, int endIndex, LQTrackingCostFunction<E> costFunction, DenseMatrix64F initialState,
                                      DiscreteOptimizationData updatedSequence);

   @Override
   public abstract boolean backwardPass(E dynamicsState, int startIndex, int endIndex, LQTrackingCostFunction<E> terminalCostFunction,
                                        DiscreteOptimizationData trajectory);
}
