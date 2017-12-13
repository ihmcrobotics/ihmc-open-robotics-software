package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.lists.RecyclingArrayList;

import java.util.List;

public interface DDPSolverInterface<E extends Enum>
{
   void initializeFromLQRSolution(E dynamicsState, RecyclingArrayList<DenseMatrix64F> stateTrajectory, RecyclingArrayList<DenseMatrix64F> controlTrajectory,
                                  RecyclingArrayList<DenseMatrix64F> desiredStateTrajectory, RecyclingArrayList<DenseMatrix64F> desiredControlTrajectory,
                                  RecyclingArrayList<DenseMatrix64F> feedbackGainTrajectory, RecyclingArrayList<DenseMatrix64F> feedForwardTrajectory);
   void initializeTrajectoriesFromDesireds(DenseMatrix64F initialCoM, RecyclingArrayList<DenseMatrix64F> desiredStateTrajectory,
                                          RecyclingArrayList<DenseMatrix64F> desiredControlTrajectory);

   RecyclingArrayList<DenseMatrix64F> getControlTrajectory();
   RecyclingArrayList<DenseMatrix64F> getStateTrajectory();

   int computeTrajectory(E dynamicsState);
   int computeTrajectory(List<E> dynamicsStates, TIntArrayList startIndices, TIntArrayList endIndices);
   double forwardPass(E dynamicsState, int startIndex, int endIndex, DenseMatrix64F initialCoM, RecyclingArrayList<DenseMatrix64F> updatedStateTrajectory,
                      RecyclingArrayList<DenseMatrix64F> updatedControlTrajectory);
   boolean backwardPass(E dynamicsState, int startIndex, int endIndex, RecyclingArrayList<DenseMatrix64F> stateTrajectory,
                        RecyclingArrayList<DenseMatrix64F> controlTrajectory);
}
