package us.ihmc.trajectoryOptimization;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.trajectoryOptimization.DiscreteOptimizationData;

import java.util.List;

public interface DDPSolverInterface<E extends Enum>
{
   void initializeFromLQRSolution(E dynamicsState, LQTrackingCostFunction<E> costFunction, DiscreteOptimizationData optimalSequence,
                                  DiscreteOptimizationData desiredSequence, RecyclingArrayList<DenseMatrix64F> feedbackGainSequence,
                                  RecyclingArrayList<DenseMatrix64F> feedForwardSequence);
   void initializeSequencesFromDesireds(DenseMatrix64F initialCoM, DiscreteOptimizationData desiredSequence);

   DiscreteOptimizationData getOptimalSequence();

   int computeSequence(E dynamicsState, LQTrackingCostFunction<E> costFunction, LQTrackingCostFunction<E> terminalCostFunction);
   int computeSequence(List<E> dynamicsStates, List<LQTrackingCostFunction<E>> costFunctions, List<LQTrackingCostFunction<E>> terminalCostFunctions,
                       TIntArrayList startIndices, TIntArrayList endIndices);
   void computeOnePass(List<E> dynamicsStates, List<LQTrackingCostFunction<E>> costFunctions, List<LQTrackingCostFunction<E>> terminalCostFunctions,
                       TIntArrayList startIndices, TIntArrayList endIndices);

   double forwardPass(E dynamicsState, int startIndex, int endIndex, LQTrackingCostFunction<E> costFunction, DenseMatrix64F initialCoM, DiscreteOptimizationData updatedSequence);
   boolean backwardPass(E dynamicsState, int startIndex, int endIndex, LQTrackingCostFunction<E> terminalCostFunction,
                        DiscreteOptimizationData trajectory);
}
