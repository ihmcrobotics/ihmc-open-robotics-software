package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

public class AverageSampleDoubleYoVariable extends DoubleYoVariable
{
   private final IntegerYoVariable dataLength;
   private final DoubleYoVariable dataSource;
   private final DoubleYoVariable dataCumulated;

   public AverageSampleDoubleYoVariable(String name, YoVariableRegistry registry)
   {
      this(name, null, registry);
   }

   public AverageSampleDoubleYoVariable(String name, DoubleYoVariable dataSource, YoVariableRegistry registry)
   {
      super(name, registry);

      this.dataSource = dataSource;
      dataLength = new IntegerYoVariable(name + "DataLength", registry);
      dataCumulated = new DoubleYoVariable(name + "DataCumulated", registry);
   }

   public void update()
   {
      if (dataSource == null)
      {
         throw new NullPointerException("AverageSampleDoubleYoVariable must be constructed with a non null "
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
