package us.ihmc.commonWalkingControlModules.dynamicPlanning;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class DiscreteOptimizationTrajectory
{
   private final DiscreteTrajectory stateTrajectory;
   private final DiscreteTrajectory controlTrajectory;

   public DiscreteOptimizationTrajectory(int stateSize, int controlSize)
   {
      this.stateTrajectory = new DiscreteTrajectory(stateSize);
      this.controlTrajectory = new DiscreteTrajectory(controlSize);
   }

   public void setTrajectorySize(double startTime, double endTime, double deltaT)
   {
      stateTrajectory.setTrajectorySize(startTime, endTime, deltaT);
      controlTrajectory.setTrajectorySize(startTime, endTime, deltaT);
   }

   public void compute(double time, DenseMatrix64F stateMatrixToPack, DenseMatrix64F controlMatrixToPack)
   {
      computeState(time, stateMatrixToPack);
      computeControl(time, controlMatrixToPack);
   }

   public void computeState(double time, DenseMatrix64F stateMatrixToPack)
   {
      stateTrajectory.compute(time, stateMatrixToPack);
   }

   public void computeControl(double time, DenseMatrix64F controlMatrixToPack)
   {
      controlTrajectory.compute(time, controlMatrixToPack);
   }

   public DiscreteTrajectory getControlTrajectory()
   {
      return controlTrajectory;
   }

   public DiscreteTrajectory getStateTrajectory()
   {
      return stateTrajectory;
   }
}
