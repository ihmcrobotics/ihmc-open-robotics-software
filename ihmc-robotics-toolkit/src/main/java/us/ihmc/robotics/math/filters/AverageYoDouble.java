package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class AverageYoDouble extends YoDouble
{
   private final YoInteger sampleSize;
   private final DoubleProvider dataSource;

   public AverageYoDouble(String name, YoRegistry registry)
   {
      this(name, null, registry);
   }

   public AverageYoDouble(String name, DoubleProvider dataSource, YoRegistry registry)
   {
      super(name, registry);

      this.dataSource = dataSource;
      sampleSize = new YoInteger(name + "SampleSize", registry);
   }

   public void update()
   {
      if (dataSource == null)
      {
         throw new NullPointerException(getClass().getSimpleName() + " must be constructed with a non null "
               + "dataSource variable to call update(), otherwise use update(double)");
      }

      update(dataSource.getValue());
   }

   public void update(double dataSource)
   {
      sampleSize.increment();
      add((dataSource - getValue()) / sampleSize.getValue());
   }

   public void reset()
   {
      sampleSize.set(0);
   }

   public int getSampleSize()
   {
      return sampleSize.getValue();
   }
}
