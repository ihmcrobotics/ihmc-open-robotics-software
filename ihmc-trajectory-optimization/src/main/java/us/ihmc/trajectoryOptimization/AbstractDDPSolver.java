package us.ihmc.trajectoryOptimization;

import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.matrixlib.DiagonalMatrixTools;
import us.ihmc.matrixlib.MatrixTools;

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

   protected final RecyclingArrayList<DMatrixRMaj> costStateGradientSequence;
   protected final RecyclingArrayList<DMatrixRMaj> costControlGradientSequence;
   protected final RecyclingArrayList<DMatrixRMaj> costStateHessianSequence;
   protected final RecyclingArrayList<DMatrixRMaj> costControlHessianSequence;
   protected final RecyclingArrayList<DMatrixRMaj> costStateControlHessianSequence;

   protected final DMatrixRMaj hamiltonianStateGradient;
   protected final DMatrixRMaj hamiltonianControlGradient;
   protected final DMatrixRMaj hamiltonianStateHessian;
   protected final DMatrixRMaj hamiltonianControlHessian;
   protected final DMatrixRMaj hamiltonianStateControlHessian;
   protected final DMatrixRMaj hamiltonianControlStateHessian;
   protected final DMatrixRMaj invQuu;

   protected final RecyclingArrayList<DMatrixRMaj> dynamicsStateGradientSequence;
   protected final RecyclingArrayList<DMatrixRMaj> dynamicsControlGradientSequence;
   protected final DMatrixRMaj dynamicsStateHessian;
   protected final DMatrixRMaj dynamicsControlHessian;
   protected final DMatrixRMaj dynamicsControlStateHessian;

   protected final RecyclingArrayList<DMatrixRMaj> valueStateGradientSequence;
   protected final RecyclingArrayList<DMatrixRMaj> valueStateHessianSequence;

   private final DMatrixRMaj Q_XX_col;
   private final DMatrixRMaj Q_UX_col;
   private final DMatrixRMaj Q_UU_col;

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);

   private final SingularValueDecomposition_F64<DMatrixRMaj> decomposer = DecompositionFactory_DDRM.svd(0, 0, true, true, true);
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

      hamiltonianStateGradient = new DMatrixRMaj(stateSize, 1);
      hamiltonianControlGradient = new DMatrixRMaj(controlSize, 1);
      hamiltonianStateHessian = new DMatrixRMaj(stateSize, stateSize);
      hamiltonianControlHessian = new DMatrixRMaj(controlSize, controlSize);
      hamiltonianStateControlHessian = new DMatrixRMaj(stateSize, controlSize);
      hamiltonianControlStateHessian = new DMatrixRMaj(controlSize, stateSize);
      invQuu = new DMatrixRMaj(controlSize, controlSize);

      dynamicsStateGradientSequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(stateSize, stateSize));
      dynamicsControlGradientSequence = new RecyclingArrayList<>(1000, new VariableVectorBuilder(stateSize, controlSize));
      dynamicsStateHessian = new DMatrixRMaj(stateSize, stateSize);
      dynamicsControlHessian = new DMatrixRMaj(stateSize, controlSize);
      dynamicsControlStateHessian = new DMatrixRMaj(stateSize, controlSize);

      valueStateHessianSequence.clear();
      valueStateGradientSequence.clear();

      feedBackGainSequence.clear();
      feedForwardSequence.clear();

      Q_XX_col = new DMatrixRMaj(stateSize, 1);
      Q_UX_col = new DMatrixRMaj(controlSize, 1);
      Q_UU_col = new DMatrixRMaj(controlSize, 1);
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
   public void initializeSequencesFromDesireds(DMatrixRMaj initialState, DiscreteOptimizationData desiredSequence, DiscreteSequence constantsSequence)
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
         DMatrixRMaj currentState = optimalSequence.getState(t);
         DMatrixRMaj currentControl = optimalSequence.getControl(t);
         DMatrixRMaj desiredState = desiredSequence.getState(t);
         DMatrixRMaj desiredControl = desiredSequence.getControl(t);
         DMatrixRMaj constants = constantsSequence.get(t);

         dynamics.getDynamicsStateGradient(dynamicsState, currentState, currentControl, constants, dynamicsStateGradientSequence.get(t));
         dynamics.getDynamicsControlGradient(dynamicsState, currentState, currentControl, constants, dynamicsControlGradientSequence.get(t));

         costFunction.getCostStateGradient(dynamicsState, currentControl, currentState, desiredControl, desiredState, constants, costStateGradientSequence.get(t));
         costFunction.getCostControlGradient(dynamicsState, currentControl, currentState, desiredControl, desiredState, constants, costControlGradientSequence.get(t));
         costFunction.getCostStateHessian(dynamicsState, currentControl, currentState, constants, costStateHessianSequence.get(t));
         costFunction.getCostControlHessian(dynamicsState, currentControl, currentState, constants, costControlHessianSequence.get(t));
         costFunction.getCostControlGradientOfStateGradient(dynamicsState, currentControl, currentState, constants, costStateControlHessianSequence.get(t));
      }
   }


   private final DMatrixRMaj U = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj W = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj V = new DMatrixRMaj(0, 0);
   boolean computeFeedbackGainAndFeedForwardTerms(DMatrixRMaj hamiltonianControlGradient, DMatrixRMaj hamiltonianControlHessian,
                                                  DMatrixRMaj hamiltonianControlStateHessian, DMatrixRMaj feedbackGainToPack,
                                                  DMatrixRMaj feedForwardControlToPack)
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
      CommonOps_DDRM.mult(V, W, tempMatrix);
      invQuu.reshape(controlSize, controlSize);
      CommonOps_DDRM.mult(tempMatrix, U, invQuu);

      // K = -inv(Quu) Qux
      CommonOps_DDRM.mult(-1.0, invQuu, hamiltonianControlStateHessian, feedbackGainToPack);

      // du = -inv(Quu) Qu
      CommonOps_DDRM.mult(-1.0, invQuu, hamiltonianControlGradient, feedForwardControlToPack);

      return true;
   }

   void updateHamiltonianApproximations(E dynamicsState, int t, DMatrixRMaj costStateGradient, DMatrixRMaj costControlGradient, DMatrixRMaj costStateHessian,
                                        DMatrixRMaj costControlHessian, DMatrixRMaj costStateControlHessian, DMatrixRMaj dynamicsStateGradient,
                                        DMatrixRMaj dynamicsControlGradient, DMatrixRMaj valueStateGradient, DMatrixRMaj valueStateHessian,
                                        DMatrixRMaj hamiltonianStateGradientToPack, DMatrixRMaj hamiltonianControlGradientToPack,
                                        DMatrixRMaj hamiltonianStateHessianToPack, DMatrixRMaj hamiltonianControlHessianToPack,
                                        DMatrixRMaj hamiltonianStateControlHessianToPack, DMatrixRMaj hamiltonianControlStateHessianToPack)
   {
      // Qx = Lx + A' Vx
      hamiltonianStateGradientToPack.set(costStateGradient);
      CommonOps_DDRM.multAddTransA(dynamicsStateGradient, valueStateGradient, hamiltonianStateGradientToPack);

      // Qu = Lu + B' Vx
      hamiltonianControlGradientToPack.set(costControlGradient);
      CommonOps_DDRM.multAddTransA(dynamicsControlGradient, valueStateGradient, hamiltonianControlGradientToPack);

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
         DMatrixRMaj currentState = optimalSequence.getState(t);
         DMatrixRMaj currentControl = optimalSequence.getControl(t);
         DMatrixRMaj constants = constantsSequence.get(t);

         for (int stateIndex = 0; stateIndex < dynamics.getStateVectorSize(); stateIndex++)
         {
            dynamics.getDynamicsStateHessian(dynamicsState, stateIndex, currentState, currentControl, constants, dynamicsStateHessian);
            dynamics.getDynamicsStateGradientOfControlGradient(dynamicsState, stateIndex, currentState, currentControl, constants, dynamicsControlStateHessian);

            CommonOps_DDRM.multTransA(dynamicsStateHessian, valueStateGradient, Q_XX_col);
            MatrixTools.addMatrixBlock(hamiltonianStateHessianToPack, 0, stateIndex, Q_XX_col, 0, 0, dynamics.getStateVectorSize(), 1, 1.0);

            CommonOps_DDRM.multTransA(dynamicsControlStateHessian, valueStateGradient, Q_UX_col);
            MatrixTools.addMatrixBlock(hamiltonianControlStateHessianToPack, 0, stateIndex, Q_UX_col, 0, 0, dynamics.getControlVectorSize(), 1, 1.0);
         }

         for (int controlIndex = 0; controlIndex < dynamics.getControlVectorSize(); controlIndex++)
         {
            dynamics.getDynamicsControlHessian(dynamicsState, controlIndex, currentState, currentControl, constants, dynamicsControlHessian);

            CommonOps_DDRM.multTransA(dynamicsControlHessian, valueStateGradient, Q_UU_col);
            MatrixTools.addMatrixBlock(hamiltonianControlHessianToPack, 0, controlIndex, Q_UU_col, 0, 0, dynamics.getControlVectorSize(), 1, 1.0);
         }

         throw new RuntimeException("This doesn't work properly.");
      }

      // Qux = Qxu'
      CommonOps_DDRM.transpose(hamiltonianStateControlHessianToPack, hamiltonianControlStateHessianToPack);
   }


   private final DMatrixRMaj stateError = new DMatrixRMaj(0, 0);
   void computeUpdatedControl(DMatrixRMaj currentState, DMatrixRMaj updatedState, DMatrixRMaj feedbackGainMatrix, DMatrixRMaj feedforwardControl,
                              DMatrixRMaj currentControl, DMatrixRMaj updatedControlToPack)
   {
      stateError.reshape(currentState.getNumRows(), 1);
      CommonOps_DDRM.subtract(updatedState, currentState, stateError);

      // u += K*(xhat - x)
      CommonOps_DDRM.mult(feedbackGainMatrix, stateError, updatedControlToPack);

      // u = alpha * du + uref
      CommonOps_DDRM.addEquals(updatedControlToPack, lineSearchGain, feedforwardControl);
      CommonOps_DDRM.addEquals(updatedControlToPack, currentControl);
   }

   void computePreviousValueApproximation(DMatrixRMaj hamiltonianStateGradient, DMatrixRMaj hamiltonianControlGradient,
                                          DMatrixRMaj hamiltonianStateHessian, DMatrixRMaj hamiltonianStateControlHessian,
                                          DMatrixRMaj feedbackGainMatrix, DMatrixRMaj previousValueStateGradientToPack,
                                          DMatrixRMaj previousValueStateHessianToPack)
   {
      // Vx = Qx + K' Qu
      previousValueStateGradientToPack.set(hamiltonianStateGradient);
      CommonOps_DDRM.multAddTransA(feedbackGainMatrix, hamiltonianControlGradient, previousValueStateGradientToPack);

      // Vxx = Qxx + Qxu K
      previousValueStateHessianToPack.set(hamiltonianStateHessian);
      CommonOps_DDRM.multAdd(hamiltonianStateControlHessian, feedbackGainMatrix, previousValueStateHessianToPack);
   }

   /**
    * D = D + A^T *  B * C
    */
   void addMultQuad(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj DToPack)
   {
      tempMatrix.reshape(A.numCols, B.numCols);
      CommonOps_DDRM.multTransA(A, B, tempMatrix);
      CommonOps_DDRM.multAdd(tempMatrix, C, DToPack);
   }

   /**
    * D = D + alpha * A^T *  B * C
    */
   void addMultQuad(double alpha, DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj DToPack)
   {
      tempMatrix.reshape(A.numCols, B.numCols);
      CommonOps_DDRM.multTransA(alpha, A, B, tempMatrix);
      CommonOps_DDRM.multAdd(tempMatrix, C, DToPack);
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
               DMatrixRMaj initialStateForSegment = updatedSequence.getState(startIndex);
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
   public abstract double forwardPass(E dynamicsState, int startIndex, int endIndex, LQTrackingCostFunction<E> costFunction, DMatrixRMaj initialState,
                                      DiscreteOptimizationData updatedSequence);

   @Override
   public abstract boolean backwardPass(E dynamicsState, int startIndex, int endIndex, LQTrackingCostFunction<E> terminalCostFunction,
                                        DiscreteOptimizationData trajectory);
}
