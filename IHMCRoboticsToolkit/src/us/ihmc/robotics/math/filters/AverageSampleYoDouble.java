package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class AverageSampleYoDouble extends YoDouble
{
   private final YoInteger dataLength;
   private final YoDouble dataSource;
   private final YoDouble dataCumulated;

   public AverageSampleYoDouble(String name, YoVariableRegistry registry)
   {
      this(name, null, registry);
   }

   public AverageSampleYoDouble(String name, YoDouble dataSource, YoVariableRegistry registry)
   {
      super(name, registry);

      this.dataSource = dataSource;
      dataLength = new YoInteger(name + "DataLength", registry);
      dataCumulated = new YoDouble(name + "DataCumulated", registry);
   }

   public void update()
   {
      if (dataSource == null)
      {
         throw new NullPointerException("AverageSampleYoDouble must be constructed with a non null "
               + "dataSource variable to call update(), otherwise use update(double)");
      }

      update(dataSource.getDoubleValue());
   }

   public void update(double dataSource)
   {
      dataLength.increment();
      dataCumulated.add(dataSource);
   }

   public void doAverage()
   {
      if (dataLength.getIntegerValue() < 1)
         return;

      set(dataCumulated.getDoubleValue() / dataLength.getValueAsDouble());
      reset();
   }

   public void reset()
   {
      dataLength.set(0);
      dataCumulated.set(0);
   }
}
