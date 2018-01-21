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

   private final DiscreteOptimizationData previousSequence;

   public DDPSolver(DiscreteHybridDynamics<E> dynamics)
   {
      this(dynamics, false);
   }

   public DDPSolver(DiscreteHybridDynamics<E> dynamics, boolean debug)
   {
      super(dynamics, debug);

      int stateSize = dynamics.getStateVectorSize();
      int controlSize = dynamics.getControlVectorSize();

      previousSequence = new DiscreteOptimizationSequence(stateSize, controlSize);
   }

   @Override
   public void initializeFromLQRSolution(E dynamicsState, LQTrackingCostFunction<E> costFunction, DiscreteOptimizationData optimalSequence,
                                         DiscreteOptimizationData desiredSequence, DiscreteSequence constantsSequence,
                                         DiscreteSequence feedbackGainSequence, DiscreteSequence feedForwardSequence)
   {
      super.initializeFromLQRSolution(dynamicsState, costFunction, optimalSequence, desiredSequence, constantsSequence, feedBackGainSequence, feedForwardSequence);

      previousSequence.setZero(optimalSequence);
   }

   @Override
   public void initializeSequencesFromDesireds(DenseMatrix64F initialState, DiscreteOptimizationData desiredSequence, DiscreteSequence constantsSequence)
   {
      super.initializeSequencesFromDesireds(initialState, desiredSequence, constantsSequence);

      previousSequence.setZero(desiredSequence);
   }



   @Override
   public boolean backwardPass(E dynamicsState, int startIndex, int endIndex,
                               LQTrackingCostFunction<E> terminalCostFunction, DiscreteOptimizationData trajectory)
   {
      boolean success = true;

      DiscreteData stateSequence = trajectory.getStateSequence();
      DiscreteData controlSequence = trajectory.getControlSequence();
      DiscreteData desiredStateSequence = desiredSequence.getStateSequence();
      DiscreteData desiredControlSequence = desiredSequence.getControlSequence();

      if (terminalCostFunction != null)
      {
         terminalCostFunction.getCostStateHessian(dynamicsState, controlSequence.get(endIndex), stateSequence.get(endIndex), constantsSequence.get(endIndex),
                                                  valueStateHessianSequence.get(endIndex));
         terminalCostFunction.getCostStateGradient(dynamicsState, controlSequence.get(endIndex), stateSequence.get(endIndex), desiredControlSequence.get(endIndex),
                                                   desiredStateSequence.get(endIndex), constantsSequence.get(endIndex), valueStateGradientSequence.get(endIndex));
      }

      for (int t = endIndex; t >= startIndex; t--)
      {
         DenseMatrix64F valueStateHessian = valueStateHessianSequence.get(t);
         DenseMatrix64F valueStateGradient = valueStateGradientSequence.get(t);

         DenseMatrix64F dynamicsStateGradient = dynamicsStateGradientSequence.get(t);
         DenseMatrix64F dynamicsControlGradient = dynamicsControlGradientSequence.get(t);

         DenseMatrix64F costStateGradient = costStateGradientSequence.get(t);
         DenseMatrix64F costControlGradient = costControlGradientSequence.get(t);
         DenseMatrix64F costStateHessian = costStateHessianSequence.get(t);
         DenseMatrix64F costControlHessian = costControlHessianSequence.get(t);
         DenseMatrix64F costStateControlHessian = costStateControlHessianSequence.get(t);

         updateHamiltonianApproximations(dynamicsState, t, costStateGradient, costControlGradient, costStateHessian, costControlHessian, costStateControlHessian,
                                         dynamicsStateGradient, dynamicsControlGradient, valueStateGradient, valueStateHessian, hamiltonianStateGradient,
                                         hamiltonianControlGradient, hamiltonianStateHessian, hamiltonianControlHessian, hamiltonianStateControlHessian,
                                         hamiltonianControlStateHessian);

         success = computeFeedbackGainAndFeedForwardTerms(hamiltonianControlGradient, hamiltonianControlHessian, hamiltonianControlStateHessian,
                                                          feedBackGainSequence.get(t), feedForwardSequence.get(t));
         if (!success)
            break;


         if (t > 0)
         {
            computePreviousValueApproximation(hamiltonianStateGradient, hamiltonianControlGradient, hamiltonianStateHessian, hamiltonianStateControlHessian,
                                              feedBackGainSequence.get(t), valueStateGradientSequence.get(t- 1), valueStateHessianSequence.get(t - 1));
         }
      }

      return success;
   }

   @Override
   public double forwardPass(E dynamicsState, int startIndex, int endIndex, LQTrackingCostFunction<E> costFunction, DenseMatrix64F initialState,
                             DiscreteOptimizationData updatedSequence)
   {
      lineSearchGain = lineSearchStartGain;
      boolean iterate = true;
      boolean lastIteration = false;
      double updatedCost = 0.0;
      while(iterate)
      {
         updatedCost = solveForwardDDPPassInternal(dynamicsState, startIndex, endIndex, costFunction, initialState, previousSequence);

         if (Double.isInfinite(updatedCost))
         {
            if (lastIteration)
               break;

            lineSearchGain = Math.max(lineSearchGain - lineSearchScaling, lineSearchGainMinimum);
            PrintTools.info("Solution diverged, decrease line search gain to " + lineSearchGain + " and trying again.");
         }
         else
         {
            updatedSequence.set(previousSequence);
            iterate = false;
         }

         if (lineSearchGain == lineSearchGainMinimum)
            lastIteration = true;
      }

      return updatedCost;
   }

   private double solveForwardDDPPassInternal(E dynamicsState, int startIndex, int endIndex, LQTrackingCostFunction<E> costFunction, DenseMatrix64F initialState,
                                              DiscreteOptimizationData updatedSequence)
   {
      updatedSequence.setState(startIndex, initialState);

      double cost = 0.0;

      for (int t = startIndex; t < endIndex; t++)
      {
         DenseMatrix64F state = optimalSequence.getState(t);
         DenseMatrix64F updatedState = updatedSequence.getState(t);
         DenseMatrix64F updatedControl = updatedSequence.getControl(t);
         DenseMatrix64F constants = constantsSequence.get(t);

         if (isStateDiverging(updatedState, state))
            return Double.POSITIVE_INFINITY;

         computeUpdatedControl(state, updatedState, feedBackGainSequence.get(t), feedForwardSequence.get(t), optimalSequence.getControl(t), updatedControl);

         if (t < desiredSequence.size() - 1)
            dynamics.getNextState(dynamicsState, updatedState, updatedControl, constants, updatedSequence.getState(t + 1));

         // compute cost
         cost += costFunction.getCost(dynamicsState, updatedControl, updatedState, desiredSequence.getControl(t), desiredSequence.getState(t), constants);
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
