package us.ihmc.sensorProcessing.encoder.processors;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public abstract class AbstractEncoderProcessor implements EncoderProcessor
{
   protected final YoVariableRegistry registry;
   protected final IntYoVariable rawTicks;
   protected final DoubleYoVariable time;

   protected final DoubleYoVariable processedTicks;
   protected final DoubleYoVariable processedTickRate;

   private final double distancePerTick;

   public AbstractEncoderProcessor(String name, IntYoVariable rawTicks, DoubleYoVariable time, double distancePerTick, YoVariableRegistry registry)
   {
      this.registry = registry;
      this.rawTicks = rawTicks;
      this.time = time;
      this.processedTicks = new DoubleYoVariable(name + "ProcTicks", registry);
      this.processedTickRate = new DoubleYoVariable(name + "ProcTickRate", registry);
      this.distancePerTick = distancePerTick;
   }

   public final double getQ()
   {
      return processedTicks.getDoubleValue() * distancePerTick;
   }

   public final double getQd()
   {
      return processedTickRate.getDoubleValue() * distancePerTick;
   }

   public abstract void update();
}
