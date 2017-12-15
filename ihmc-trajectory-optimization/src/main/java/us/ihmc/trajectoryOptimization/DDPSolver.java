package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class DDPSolver<E extends Enum> extends AbstractDDPSolver<E> implements DDPSolverInterface<E>
{
   private static final double lineSearchScaling = 0.1;
   private static final double lineSearchStartGain = 1.0;
   private static final double lineSearchGainMinimum = 0.0;

   private final DiscreteOptimizationTrajectory previousTrajectory;

   public DDPSolver(DiscreteHybridDynamics<E> dynamics, LQCostFunction costFunction, LQCostFunction terminalCostFunction)
   {
      this(dynamics, costFunction, terminalCostFunction, false);
   }

   public DDPSolver(DiscreteHybridDynamics<E> dynamics, LQCostFunction costFunction, LQCostFunction terminalCostFunction, boolean debug)
   {
      super(dynamics, costFunction, terminalCostFunction, debug);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      previousTrajectory = new DiscreteOptimizationTrajectory(stateSize, controlSize);
   }

   @Override
   public void initializeFromLQRSolution(E dynamicsState, DiscreteOptimizationTrajectory optimalTrajectory, DiscreteOptimizationTrajectory desiredTrajectory,
                                         RecyclingArrayList<DenseMatrix64F> feedbackGainTrajectory, RecyclingArrayList<DenseMatrix64F> feedForwardTrajectory)
   {
      super.initializeFromLQRSolution(dynamicsState, optimalTrajectory, desiredTrajectory, feedBackGainTrajectory, feedForwardTrajectory);

      previousTrajectory.setZeroTrajectory(optimalTrajectory);
   }

   @Override
   public void initializeTrajectoriesFromDesireds(DenseMatrix64F initialCoM, DiscreteOptimizationTrajectory desiredTrajectory)
   {
      super.initializeTrajectoriesFromDesireds(initialCoM, desiredTrajectory);

      previousTrajectory.setZeroTrajectory(desiredTrajectory);
   }



   @Override
   public boolean backwardPass(E dynamicsState, int startIndex, int endIndex, DiscreteOptimizationTrajectory trajectory)
   {
      boolean success = true;

      DiscreteTrajectory stateTrajectory = trajectory.getStateTrajectory();
      DiscreteTrajectory controlTrajectory = trajectory.getControlTrajectory();
      DiscreteTrajectory desiredStateTrajectory = desiredTrajectory.getStateTrajectory();
      DiscreteTrajectory desiredControlTrajectory = desiredTrajectory.getControlTrajectory();

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
   public double forwardPass(E dynamicsState, int startIndex, int endIndex, DenseMatrix64F initialCoM, DiscreteOptimizationTrajectory updatedTrajectory)
   {
      lineSearchGain = lineSearchStartGain;
      boolean iterate = true;
      boolean lastIteration = false;
      double updatedCost = 0.0;
      while(iterate)
      {
         updatedCost = solveForwardDDPPassInternal(dynamicsState, startIndex, endIndex, initialCoM, previousTrajectory);

         if (Double.isInfinite(updatedCost))
         {
            if (lastIteration)
               break;

            lineSearchGain = Math.max(lineSearchGain - lineSearchScaling, lineSearchGainMinimum);
            PrintTools.info("Solution diverged, decrease line search gain to " + lineSearchGain + " and trying again.");
         }
         else
         {
            updatedTrajectory.set(previousTrajectory);
            iterate = false;
         }

         if (lineSearchGain == lineSearchGainMinimum)
            lastIteration = true;
      }

      return updatedCost;
   }

   private double solveForwardDDPPassInternal(E dynamicsState, int startIndex, int endIndex, DenseMatrix64F initialCoM,
                                              DiscreteOptimizationTrajectory updatedTrajectory)
   {
      updatedTrajectory.setState(startIndex, initialCoM);

      double cost = 0.0;

      for (int t = startIndex; t < endIndex; t++)
      {
         DenseMatrix64F state = optimalTrajectory.getState(t);
         DenseMatrix64F updatedState = updatedTrajectory.getState(t);
         DenseMatrix64F updatedControl = updatedTrajectory.getControl(t);

         if (isStateDiverging(updatedState, state))
            return Double.POSITIVE_INFINITY;

         computeUpdatedControl(state, updatedState, feedBackGainTrajectory.get(t), feedForwardTrajectory.get(t), optimalTrajectory.getControl(t), updatedControl);

         if (t < desiredTrajectory.size() - 1)
            dynamics.getNextState(dynamicsState, updatedState, updatedControl, updatedTrajectory.getState(t + 1));

         // compute cost
         cost += costFunction.getCost(updatedControl, updatedState, desiredTrajectory.getControl(t), desiredTrajectory.getState(t));
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
