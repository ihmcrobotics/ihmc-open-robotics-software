package us.ihmc.trajectoryOptimization;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.lists.RecyclingArrayList;

import java.util.List;

public interface DDPSolverInterface<E extends Enum>
{
   void initializeFromLQRSolution(E dynamicsState, DiscreteOptimizationTrajectory optimalTrajectory, DiscreteOptimizationTrajectory desiredTrajectory,
                                  RecyclingArrayList<DenseMatrix64F> feedbackGainTrajectory, RecyclingArrayList<DenseMatrix64F> feedForwardTrajectory);
   void initializeTrajectoriesFromDesireds(DenseMatrix64F initialCoM, DiscreteOptimizationTrajectory desiredTrajectory);

   DiscreteOptimizationTrajectory getOptimalTrajectory();

   int computeTrajectory(E dynamicsState);
   int computeTrajectory(List<E> dynamicsStates, TIntArrayList startIndices, TIntArrayList endIndices);
   double forwardPass(E dynamicsState, int startIndex, int endIndex, DenseMatrix64F initialCoM, DiscreteOptimizationTrajectory updatedTrajectory);
   boolean backwardPass(E dynamicsState, int startIndex, int endIndex, DiscreteOptimizationTrajectory trajectory);
}
