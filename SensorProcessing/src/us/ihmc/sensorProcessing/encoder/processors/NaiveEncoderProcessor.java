package us.ihmc.sensorProcessing.encoder.processors;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class NaiveEncoderProcessor extends AbstractEncoderProcessor
{
   private final IntYoVariable previousPosition;
   private final DoubleYoVariable previousTime;

   public NaiveEncoderProcessor(String name, IntYoVariable rawTicks, DoubleYoVariable time, double distancePerTick, YoVariableRegistry registry)
   {
      super(name, rawTicks, time, distancePerTick, registry);

      this.previousPosition = new IntYoVariable(name + "PrevPos", registry);
      this.previousTime = new DoubleYoVariable(name + "PrevTime", registry);
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
