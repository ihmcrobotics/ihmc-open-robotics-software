package us.ihmc.trajectoryOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class DiscreteOptimizationTrajectory implements DiscreteOptimizationData
{
   private final DiscreteTrajectory stateTrajectory;
   private final DiscreteTrajectory controlTrajectory;

   public DiscreteOptimizationTrajectory(int stateSize, int controlSize)
   {
      this.stateTrajectory = new DiscreteTrajectory(stateSize);
      this.controlTrajectory = new DiscreteTrajectory(controlSize);
   }

   public void set(DiscreteOptimizationTrajectory other)
   {
      this.stateTrajectory.set(other.stateTrajectory);
      this.controlTrajectory.set(other.controlTrajectory);
   }

   public void setTrajectoryDuration(double startTime, double endTime, double deltaT)
   {
      stateTrajectory.setTrajectoryDuration(startTime, endTime, deltaT);
      controlTrajectory.setTrajectoryDuration(startTime, endTime, deltaT);
   }

   public void compute(double time, DenseMatrix64F stateMatrixToPack, DenseMatrix64F controlMatrixToPack)
   {
      computeState(time, stateMatrixToPack);
      computeControl(time, controlMatrixToPack);
   }

   public double getStartTime()
   {
      return controlTrajectory.getStartTime();
   }

   public double getEndTime()
   {
      return controlTrajectory.getEndTime();
   }

   public double getDeltaT()
   {
      return controlTrajectory.getDeltaT();
   }


   public void computeState(double time, DenseMatrix64F stateMatrixToPack)
   {
      stateTrajectory.compute(time, stateMatrixToPack);
   }

   public void computeControl(double time, DenseMatrix64F controlMatrixToPack)
   {
      controlTrajectory.compute(time, controlMatrixToPack);
   }

   public DiscreteTrajectory getStateTrajectory()
   {
      return stateTrajectory;
   }

   public DiscreteTrajectory getControlTrajectory()
   {
      return controlTrajectory;
   }

   public void setZero(DiscreteOptimizationTrajectory other)
   {
      setTrajectoryDuration(other.getStartTime(), other.getEndTime(), other.getDeltaT());
   }

   @Override
   public DiscreteData getControlSequence()
   {
      return controlTrajectory;
   }

   @Override
   public DiscreteData getStateSequence()
   {
      return stateTrajectory;
   }

   @Override
   public DenseMatrix64F getState(int index)
   {
      return stateTrajectory.get(index);
   }

   @Override
   public DenseMatrix64F getControl(int index)
   {
      return controlTrajectory.get(index);
   }

   @Override
   public void setState(int index, DenseMatrix64F state)
   {
      stateTrajectory.get(index).set(state);
   }

   @Override
   public void setControl(int index, DenseMatrix64F control)
   {
      controlTrajectory.get(index).set(control);
   }

   @Override
   public void set(DiscreteOptimizationData other)
   {
      controlTrajectory.set(other.getControlSequence());
      stateTrajectory.set(other.getStateSequence());
   }

   @Override
   public int size()
   {
      return controlTrajectory.size();
   }


   @Override
   public void setZero(DiscreteOptimizationData other)
   {
      stateTrajectory.setZero(other.size());
      controlTrajectory.setZero(other.size());
   }
}
