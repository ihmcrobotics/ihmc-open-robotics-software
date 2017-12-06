package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.lists.RecyclingArrayList;

public interface LQRSolverInterface<E extends Enum>
{
   void setDesiredTrajectories(RecyclingArrayList<DenseMatrix64F> desiredStateTrajectory, RecyclingArrayList<DenseMatrix64F> desiredControlTrajectory,
                               DenseMatrix64F initialState);
   void getOptimalTrajectories(RecyclingArrayList<DenseMatrix64F> optimalStateTrajectoryToPack, RecyclingArrayList<DenseMatrix64F> optimalControlTrajectoryToPack);

   void solveRiccatiEquation(E dynamicState, int startIndex, int endIndex); // backwards pass
   void computeOptimalTrajectories(E dynamicsState, int startIndex, int endIndex); // forward pass
}
