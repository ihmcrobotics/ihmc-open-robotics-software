package us.ihmc.trajectoryOptimization;

import org.ejml.data.DMatrixRMaj;

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

   public void compute(double time, DMatrixRMaj stateMatrixToPack, DMatrixRMaj controlMatrixToPack)
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


   public void computeState(double time, DMatrixRMaj stateMatrixToPack)
   {
      stateTrajectory.compute(time, stateMatrixToPack);
   }

   public void computeControl(double time, DMatrixRMaj controlMatrixToPack)
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
   public DMatrixRMaj getState(int index)
   {
      return stateTrajectory.get(index);
   }

   @Override
   public DMatrixRMaj getControl(int index)
   {
      return controlTrajectory.get(index);
   }

   @Override
   public void setState(int index, DMatrixRMaj state)
   {
      stateTrajectory.get(index).set(state);
   }

   @Override
   public void setControl(int index, DMatrixRMaj control)
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
