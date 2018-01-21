package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.lists.RecyclingArrayList;

public interface LQRSolverInterface<E extends Enum>
{
   void setDesiredSequence(DiscreteOptimizationData desiredSequence, DiscreteSequence constantsSequence, DenseMatrix64F initialState);
   void getOptimalSequence(DiscreteOptimizationData optimalSequenceToPack);

   DiscreteOptimizationData getOptimalSequence();
   DiscreteData getOptimalStateSequence();
   DiscreteData getOptimalControlSequence();
   DiscreteSequence getOptimalFeedbackGainSequence();
   DiscreteSequence getOptimalFeedForwardControlSequence();

   DenseMatrix64F getValueHessian();

   void solveRiccatiEquation(E dynamicState, int startIndex, int endIndex); // backwards pass
   void computeOptimalSequences(E dynamicsState, int startIndex, int endIndex); // forward pass
}
