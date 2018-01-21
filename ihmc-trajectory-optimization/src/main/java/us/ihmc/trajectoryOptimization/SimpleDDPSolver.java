package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class SimpleDDPSolver<E extends Enum> extends AbstractDDPSolver<E> implements DDPSolverInterface<E>
{
   public SimpleDDPSolver(DiscreteHybridDynamics<E> dynamics, LQCostFunction costFunction, LQCostFunction terminalCostFunction)
   {
      this(dynamics, costFunction, terminalCostFunction, false);
   }

   public SimpleDDPSolver(DiscreteHybridDynamics<E> dynamics, LQCostFunction costFunction, LQCostFunction terminalCostFunction, boolean debug)
   {
      super(dynamics, costFunction, terminalCostFunction, debug);
      lineSearchGain = 1.0;
   }

   @Override
   public double forwardPass(E dynamicsState, int startIndex, int endIndex, DenseMatrix64F initialCoM, DiscreteOptimizationTrajectory updatedTrajectory)
   {
      updatedTrajectory.setState(startIndex, initialCoM);

      double cost = 0.0;

      for (int t = startIndex; t <= endIndex; t++)
      {
         DenseMatrix64F desiredControl = desiredTrajectory.getControl(t);
         DenseMatrix64F desiredState = desiredTrajectory.getState(t);
         DenseMatrix64F updatedControl = updatedTrajectory.getControl(t);
         DenseMatrix64F updatedState = updatedTrajectory.getState(t);

         DenseMatrix64F currentState = optimalTrajectory.getState(t);
         DenseMatrix64F currentControl = optimalTrajectory.getControl(t);

         computeUpdatedControl(currentState, updatedState, feedBackGainTrajectory.get(t), feedForwardTrajectory.get(t),
                               currentControl, updatedControl);

         // compute cost
         cost += costFunction.getCost(updatedControl, updatedState, desiredControl, desiredState);

         // integrate
         if (t < desiredTrajectory.size() - 1)
            dynamics.getNextState(dynamicsState, updatedState, updatedControl, updatedTrajectory.getState(t + 1));
      }

      return cost;
   }

   @Override
   public boolean backwardPass(E dynamicsState, int startIndex, int endIndex, DiscreteOptimizationTrajectory trajectoryToPack)
   {
      boolean success = true;

      DiscreteTrajectory stateTrajectory = trajectoryToPack.getStateTrajectory();
      DiscreteTrajectory controlTrajectory = trajectoryToPack.getControlTrajectory();
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
}
