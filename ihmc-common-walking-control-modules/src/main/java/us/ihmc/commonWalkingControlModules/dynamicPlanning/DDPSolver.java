package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class DDPSolver<E extends Enum> extends AbstractDDPSolver<E> implements DDPSolverInterface<E>
{
   private static final double lineSearchScaling = 0.1;
   private static final double lineSearchStartGain = 1.0;
   private static final double lineSearchGainMinimum = 0.0;

   private final RecyclingArrayList<DenseMatrix64F> previousUpdatedStateTrajectory;
   private final RecyclingArrayList<DenseMatrix64F> previousUpdatedControlTrajectory;

   public DDPSolver(DiscreteHybridDynamics<E> dynamics, LQCostFunction costFunction, LQCostFunction terminalCostFunction)
   {
      this(dynamics, costFunction, terminalCostFunction, false);
   }

   public DDPSolver(DiscreteHybridDynamics<E> dynamics, LQCostFunction costFunction, LQCostFunction terminalCostFunction, boolean debug)
   {
      super(dynamics, costFunction, terminalCostFunction, debug);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      VariableVectorBuilder controlBuilder = new VariableVectorBuilder(controlSize, 1);
      VariableVectorBuilder stateBuilder = new VariableVectorBuilder(stateSize, 1);

      previousUpdatedStateTrajectory = new RecyclingArrayList<>(1000, stateBuilder);
      previousUpdatedControlTrajectory = new RecyclingArrayList<>(1000, controlBuilder);
      previousUpdatedStateTrajectory.clear();
      previousUpdatedControlTrajectory.clear();
   }

   @Override
   public void initializeFromLQRSolution(E dynamicsState, RecyclingArrayList<DenseMatrix64F> stateTrajectory, RecyclingArrayList<DenseMatrix64F> controlTrajectory,
                                         RecyclingArrayList<DenseMatrix64F> desiredStateTrajectory, RecyclingArrayList<DenseMatrix64F> desiredControlTrajectory,
                                         RecyclingArrayList<DenseMatrix64F> feedBackGainTrajectory, RecyclingArrayList<DenseMatrix64F> feedForwardTrajectory)
   {
      super.initializeFromLQRSolution(dynamicsState, stateTrajectory, controlTrajectory, desiredStateTrajectory, desiredControlTrajectory, feedBackGainTrajectory,
                                      feedForwardTrajectory);

      previousUpdatedControlTrajectory.clear();
      previousUpdatedStateTrajectory.clear();

      for (int i = 0; i < stateTrajectory.size(); i++)
      {
         previousUpdatedControlTrajectory.add().zero();
         previousUpdatedStateTrajectory.add().zero();
      }
   }

   @Override
   public void initializeTrajectoriesFromDesireds(DenseMatrix64F initialCoM, RecyclingArrayList<DenseMatrix64F> desiredStateTrajectory, RecyclingArrayList<DenseMatrix64F> desiredControlTrajectory)
   {
      super.initializeTrajectoriesFromDesireds(initialCoM, desiredStateTrajectory, desiredControlTrajectory);

      previousUpdatedControlTrajectory.clear();
      previousUpdatedStateTrajectory.clear();

      for (int i = 0; i < desiredStateTrajectory.size(); i++)
      {
         previousUpdatedControlTrajectory.add().zero();
         previousUpdatedStateTrajectory.add().zero();
      }
   }



   @Override
   public boolean backwardPass(E dynamicsState, int startIndex, int endIndex, RecyclingArrayList<DenseMatrix64F> stateTrajectory,
                               RecyclingArrayList<DenseMatrix64F> controlTrajectory)
   {
      boolean success = true;

      terminalCostFunction.getCostStateHessian(controlTrajectory.get(endIndex), stateTrajectory.get(endIndex), valueStateHessianTrajectory.get(endIndex));
      terminalCostFunction.getCostStateGradient(controlTrajectory.get(endIndex), stateTrajectory.get(endIndex), desiredControlTrajectory.get(endIndex),
                                                desiredStateTrajectory.get(endIndex), valueStateGradientTrajectory.get(endIndex));

      for (int t = endIndex; t >= startIndex; t--)
      {
         DenseMatrix64F valueStateHessian = valueStateHessianTrajectory.get(t);
         DenseMatrix64F valueStateGradient = valueStateGradientTrajectory.get(t);

         DenseMatrix64F dynamicsStateGradient = dynamicsStateGradientTrajectory.get(t);
         DenseMatrix64F dynamicsControlGradient = dynamicsControlGradientTrajectory.get(t);

         DenseMatrix64F costStateGradient = costStateGradientTrajectory.get(t);
         DenseMatrix64F costControlGradient = costControlGradientTrajectory.get(t);
         DenseMatrix64F costStateHessian = costStateHessianTrajectory.get(t);
         DenseMatrix64F costControlHessian = costControlHessianTrajectory.get(t);
         DenseMatrix64F costStateControlHessian = costStateControlHessianTrajectory.get(t);

         updateHamiltonianApproximations(dynamicsState, t, costStateGradient, costControlGradient, costStateHessian, costControlHessian, costStateControlHessian,
                                         dynamicsStateGradient, dynamicsControlGradient, valueStateGradient, valueStateHessian, hamiltonianStateGradient,
                                         hamiltonianControlGradient, hamiltonianStateHessian, hamiltonianControlHessian, hamiltonianStateControlHessian,
                                         hamiltonianControlStateHessian);

         success = computeFeedbackGainAndFeedForwardTerms(hamiltonianControlGradient, hamiltonianControlHessian, hamiltonianControlStateHessian,
                                                          feedBackGainTrajectory.get(t), feedForwardTrajectory.get(t));
         if (!success)
            break;


         if (t > 0)
         {
            computePreviousValueApproximation(hamiltonianStateGradient, hamiltonianControlGradient, hamiltonianStateHessian, hamiltonianStateControlHessian,
                                              feedBackGainTrajectory.get(t), valueStateGradientTrajectory.get(t- 1), valueStateHessianTrajectory.get(t - 1));
         }
      }

      return success;
   }

   @Override
   public double forwardPass(E dynamicsState, int startIndex, int endIndex, DenseMatrix64F initialCoM,
                             RecyclingArrayList<DenseMatrix64F> updatedStateTrajectory, RecyclingArrayList<DenseMatrix64F> updatedControlTrajectory)
   {
      lineSearchGain = lineSearchStartGain;
      boolean iterate = true;
      boolean lastIteration = false;
      double updatedCost = 0.0;
      while(iterate)
      {
         updatedCost = solveForwardDDPPassInternal(dynamicsState, startIndex, endIndex, initialCoM, previousUpdatedStateTrajectory, previousUpdatedControlTrajectory);

         if (Double.isInfinite(updatedCost))
         {
            if (lastIteration)
               break;

            lineSearchGain = Math.max(lineSearchGain - lineSearchScaling, lineSearchGainMinimum);
            PrintTools.info("Solution diverged, decrease line search gain to " + lineSearchGain + " and trying again.");
         }
         else
         {
            for (int i = 0; i < previousUpdatedControlTrajectory.size(); i++)
            {
               updatedStateTrajectory.get(i).set(previousUpdatedStateTrajectory.get(i));
               updatedControlTrajectory.get(i).set(previousUpdatedControlTrajectory.get(i));
            }
            iterate = false;
         }

         if (lineSearchGain == lineSearchGainMinimum)
            lastIteration = true;
      }

      return updatedCost;
   }

   private double solveForwardDDPPassInternal(E dynamicsState, int startIndex, int endIndex, DenseMatrix64F initialCoM,
                                              RecyclingArrayList<DenseMatrix64F> updatedStateTrajectory, RecyclingArrayList<DenseMatrix64F> updatedControlTrajectory)
   {
      updatedStateTrajectory.get(startIndex).set(initialCoM);

      double cost = 0.0;

      for (int t = startIndex; t < endIndex; t++)
      {
         DenseMatrix64F state = stateTrajectory.get(t);
         DenseMatrix64F updatedState = updatedStateTrajectory.get(t);
         DenseMatrix64F updatedControl = updatedControlTrajectory.get(t);

         if (isStateDiverging(updatedState, state))
            return Double.POSITIVE_INFINITY;

         computeUpdatedControl(state, updatedState, feedBackGainTrajectory.get(t), feedForwardTrajectory.get(t), controlTrajectory.get(t), updatedControl);

         if (t < desiredStateTrajectory.size() - 1)
            dynamics.getNextState(dynamicsState, updatedState, updatedControl, updatedStateTrajectory.get(t + 1));

         // compute cost
         cost += costFunction.getCost(updatedControl, updatedState, desiredControlTrajectory.get(t), desiredStateTrajectory.get(t));
      }

      return cost;
   }

   private boolean isStateDiverging(DenseMatrix64F newState, DenseMatrix64F originalState)
   {
      for (int i = 0; i < newState.getNumElements(); i++)
      {
         if (!MathTools.epsilonEquals(newState.get(i), originalState.get(i), 1e20))
            return true;
      }
      return false;

   }
}
