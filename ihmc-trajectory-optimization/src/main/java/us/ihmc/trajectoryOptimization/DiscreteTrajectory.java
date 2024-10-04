package us.ihmc.trajectoryOptimization;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;

public class DiscreteTrajectory extends DiscreteSequence
{
   private double startTime = Double.NaN;
   private double endTime = Double.NaN;
   private double deltaT = Double.NaN;
   private double realDeltaT = Double.NaN;
   private int numberOfTimeSteps = 0;

   public DiscreteTrajectory(int dimensionality)
   {
      super(dimensionality);
   }

   public DiscreteTrajectory(int xDimension, int yDimension)
   {
      super(xDimension, yDimension);
   }

   public void set(DiscreteTrajectory other)
   {
      this.startTime = other.startTime;
      this.endTime = other.endTime;
      this.deltaT = other.deltaT;
      this.realDeltaT = other.realDeltaT;
      this.numberOfTimeSteps = other.numberOfTimeSteps;

      this.clear();
      for (int i = 0; i < other.numberOfTimeSteps; i++)
         this.add().set(other.get(i));
   }

   public void setTrajectoryDuration(double startTime, double endTime, double deltaT)
   {
      this.startTime = startTime;
      this.endTime = endTime;
      this.deltaT = deltaT;
      computeRequiredDeltaT((endTime - startTime), deltaT);

      this.setLength(numberOfTimeSteps);
   }

   public void compute(double time, DMatrixRMaj valueToPack)
   {
      if (!MathTools.intervalContains(time, startTime, endTime))
      {
         LogTools.warn("Requested computation time is outside the bounds of the trajectory. We do not extrapolate.");
         time = MathTools.clamp(time, startTime, endTime);
      }

      int startIndex = (int) Math.floor((time - startTime)/ realDeltaT);
      double startTime = startIndex * realDeltaT;
      double alpha = (time - startTime) / realDeltaT;

      CommonOps_DDRM.scale((1.0 - alpha), get(startIndex), valueToPack);
      CommonOps_DDRM.addEquals(valueToPack, alpha, get(startIndex + 1));
   }

   public double getStartTime()
   {
      return startTime;
   }

   public double getEndTime()
   {
      return endTime;
   }

   public double getDeltaT()
   {
      return deltaT;
   }

   private void computeRequiredDeltaT(double trajectoryDuration, double deltaT)
   {
      numberOfTimeSteps = (int) Math.floor(trajectoryDuration / deltaT);
      realDeltaT = trajectoryDuration / numberOfTimeSteps;
   }
}
