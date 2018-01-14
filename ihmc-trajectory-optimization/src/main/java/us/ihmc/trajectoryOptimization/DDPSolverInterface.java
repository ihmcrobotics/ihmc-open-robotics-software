package us.ihmc.trajectoryOptimization;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.trajectoryOptimization.DiscreteOptimizationData;

import java.util.List;

public interface DDPSolverInterface<E extends Enum>
{
   void initializeFromLQRSolution(E dynamicsState, DiscreteOptimizationData optimalSequence, DiscreteOptimizationData desiredSequence,
                                  RecyclingArrayList<DenseMatrix64F> feedbackGainSequence, RecyclingArrayList<DenseMatrix64F> feedForwardSequence);
   void initializeSequencesFromDesireds(DenseMatrix64F initialCoM, DiscreteOptimizationData desiredSequence);

   DiscreteOptimizationData getOptimalSequence();

   int computeSequence(E dynamicsState);
   int computeSequence(List<E> dynamicsStates, TIntArrayList startIndices, TIntArrayList endIndices);
   void computeOnePass(List<E> dynamicsStates, TIntArrayList startIndices, TIntArrayList endIndices);

   double forwardPass(E dynamicsState, int startIndex, int endIndex, DenseMatrix64F initialCoM, DiscreteOptimizationData updatedSequence);
   boolean backwardPass(E dynamicsState, int startIndex, int endIndex, DiscreteOptimizationData trajectory);
}
