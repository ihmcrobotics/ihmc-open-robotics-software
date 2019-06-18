package us.ihmc.realtime.barrierScheduler.benchmarks.helperClasses;

import us.ihmc.concurrent.runtime.barrierScheduler.implicitContext.tasks.InPlaceCopyable;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class BarrierSchedulerLoadTestData implements InPlaceCopyable<BarrierSchedulerLoadTestData>
{
   private double slowTaskFirstResult = 0.0;
   private double slowTaskSecondResult = 0.0;
   private int slowTaskNumberOfOperations = -1;

   private double fastTaskFirstResult = 0.0;
   private double fastTaskSecondResult = 0.0;
   private int fastTaskNumberOfOperations = -1;

   public int getSlowTaskNumberOfOperations()
   {
      return slowTaskNumberOfOperations;
   }

   public void setSlowTaskNumberOfOperations(int slowTaskNumberOfOperations)
   {
      this.slowTaskNumberOfOperations = slowTaskNumberOfOperations;
   }

   public int getFastTaskNumberOfOperations()
   {
      return fastTaskNumberOfOperations;
   }

   public void setFastTaskNumberOfOperations(int fastTaskNumberOfOperations)
   {
      this.fastTaskNumberOfOperations = fastTaskNumberOfOperations;
   }

   public double getFastTaskFirstResult()
   {
      return fastTaskFirstResult;
   }

   public void setFastTaskFirstResult(double fastTaskFirstValue)
   {
      this.fastTaskFirstResult = fastTaskFirstValue;
   }

   public double getFastTaskSecondResult()
   {
      return fastTaskSecondResult;
   }

   public void setFastTaskSecondResult(double fastTaskSecondValue)
   {
      this.fastTaskSecondResult = fastTaskSecondValue;
   }

   public double getSlowTaskFirstResult()
   {
      return slowTaskFirstResult;
   }

   public void setSlowTaskFirstResult(double slowTaskFirstResult)
   {
      this.slowTaskFirstResult = slowTaskFirstResult;
   }

   public double getSlowTaskSecondResult()
   {
      return slowTaskSecondResult;
   }

   public void setSlowTaskSecondResult(double slowTaskSecondResult)
   {
      this.slowTaskSecondResult = slowTaskSecondResult;
   }

   @Override
   public void copyFrom(BarrierSchedulerLoadTestData src)
   {
      slowTaskFirstResult = src.slowTaskFirstResult;
      slowTaskSecondResult = src.slowTaskSecondResult;
      slowTaskNumberOfOperations = src.slowTaskNumberOfOperations;
      
      fastTaskFirstResult = src.fastTaskFirstResult;
      fastTaskSecondResult = src.fastTaskSecondResult;
      fastTaskNumberOfOperations = src.fastTaskNumberOfOperations;
   }
}
