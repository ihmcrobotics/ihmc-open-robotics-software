package us.ihmc.sensorProcessing.encoder.processors;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class NaiveEncoderProcessor extends AbstractEncoderProcessor
{
   private final IntegerYoVariable previousPosition;
   private final DoubleYoVariable previousTime;

   public NaiveEncoderProcessor(String name, IntegerYoVariable rawTicks, DoubleYoVariable time, double distancePerTick, YoVariableRegistry registry)
   {
      super(name, rawTicks, time, distancePerTick, registry);

      this.previousPosition = new IntegerYoVariable(name + "PrevPos", registry);
      this.previousTime = new DoubleYoVariable(name + "PrevTime", registry);
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

      previousPosition.set(rawTicks.getIntegerValue());
      previousTime.set(time.getDoubleValue());
   }
}
