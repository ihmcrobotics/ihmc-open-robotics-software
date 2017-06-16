package us.ihmc.sensorProcessing.encoder.processors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.IntegerYoVariable;

public class NaiveEncoderProcessor extends AbstractEncoderProcessor
{
   private final IntegerYoVariable previousPosition;
   private final YoDouble previousTime, dx, dt;

   public NaiveEncoderProcessor(String name, IntegerYoVariable rawTicks, YoDouble time, double distancePerTick, YoVariableRegistry registry)
   {
      super(name, rawTicks, time, distancePerTick, registry);

      this.previousPosition = new IntegerYoVariable(name + "PrevPos", registry);
      this.previousTime = new YoDouble(name + "PrevTime", registry);
      
      this.dx = new YoDouble(name + "DX", registry);
      this.dt = new YoDouble(name + "DT", registry);
   }
   
   public void initialize()
   {
      previousPosition.set(rawTicks.getIntegerValue());
      previousTime.set(Double.NEGATIVE_INFINITY);
   }

   public void update()
   {
      processedTicks.set(rawTicks.getIntegerValue());
      double dx = rawTicks.getIntegerValue() - previousPosition.getIntegerValue();
      double dt = time.getDoubleValue() - previousTime.getDoubleValue();
      processedTickRate.set(dx / dt);
      
      this.dx.set(dx);
      this.dt.set(dt);

      previousPosition.set(rawTicks.getIntegerValue());
      previousTime.set(time.getDoubleValue());
   }
}
