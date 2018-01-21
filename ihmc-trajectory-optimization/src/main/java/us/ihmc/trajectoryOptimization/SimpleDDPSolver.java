package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;

public class SimpleDDPSolver<E extends Enum> extends AbstractDDPSolver<E> implements DDPSolverInterface<E>
{
   public SimpleDDPSolver(DiscreteHybridDynamics<E> dynamics)
   {
      this(dynamics, false);
   }

   public SimpleDDPSolver(DiscreteHybridDynamics<E> dynamics, boolean debug)
   {
      super(dynamics, debug);
      lineSearchGain = 1.0;
   }

   @Override
   public double forwardPass(E dynamicsState, int startIndex, int endIndex, LQTrackingCostFunction<E> costFunction,
                             DenseMatrix64F initialState, DiscreteOptimizationData updatedSequence)
   {
      updatedSequence.setState(startIndex, initialState);

      double cost = 0.0;

      for (int t = startIndex; t <= endIndex; t++)
      {
         DenseMatrix64F desiredControl = desiredSequence.getControl(t);
         DenseMatrix64F desiredState = desiredSequence.getState(t);
         DenseMatrix64F updatedControl = updatedSequence.getControl(t);
         DenseMatrix64F updatedState = updatedSequence.getState(t);

         DenseMatrix64F currentState = optimalSequence.getState(t);
         DenseMatrix64F currentControl = optimalSequence.getControl(t);

         DenseMatrix64F constants = constantsSequence.get(t);

         computeUpdatedControl(currentState, updatedState, feedBackGainSequence.get(t), feedForwardSequence.get(t), currentControl, updatedControl);

         // compute cost
         cost += costFunction.getCost(dynamicsState, updatedControl, updatedState, desiredControl, desiredState, constants);

         // integrate // FIXME this is wrong
         if (t < desiredSequence.size() - 1)
            dynamics.getNextState(dynamicsState, updatedState, updatedControl, constants, updatedSequence.getState(t + 1));
      }

      return cost;
   }

   @Override
   public boolean backwardPass(E dynamicsState, int startIndex, int endIndex, LQTrackingCostFunction<E> terminalCostFunction, DiscreteOptimizationData sequence)
   {
      boolean success = true;

      DiscreteData stateSequence = sequence.getStateSequence();
      DiscreteData controlSequence = sequence.getControlSequence();
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
}
