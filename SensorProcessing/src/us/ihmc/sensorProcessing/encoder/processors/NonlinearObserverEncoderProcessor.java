package us.ihmc.sensorProcessing.encoder.processors;


import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class NonlinearObserverEncoderProcessor extends AbstractEncoderProcessor
{
   private final YoDouble discretePosition;
   private final YoDouble error, alpha1, alpha2;

   private final YoDouble previousTime;
   
   public NonlinearObserverEncoderProcessor(String name, YoInteger rawTicks, YoDouble time, double distancePerTick, YoVariableRegistry registry)
   {
      super(name, rawTicks, time, distancePerTick, registry);
      this.previousTime = new YoDouble(name + "PrevTime", registry);

      this.discretePosition = new YoDouble(name + "DiscretePosition", registry);
      this.error = new YoDouble(name + "Error", registry);

      this.alpha1 = new YoDouble(name + "Alpha1", registry);
      this.alpha2 = new YoDouble(name + "Alpha2", registry);

      alpha1.set(0.03);
      alpha2.set(1.0);
   }
   
   public void initialize()
   {
      // empty
   }
   
   public void update()
   {
      discretePosition.set(processedTicks.getDoubleValue());

      error.set(rawTicks.getIntegerValue() - discretePosition.getDoubleValue());

      processedTickRate.set(processedTickRate.getDoubleValue() + alpha2.getDoubleValue() * error.getDoubleValue());
      double dt = time.getDoubleValue() - previousTime.getDoubleValue();
      processedTicks.set(processedTicks.getDoubleValue() + processedTickRate.getDoubleValue() * dt + alpha1.getDoubleValue() * error.getDoubleValue());
      previousTime.set(time.getDoubleValue());
   }
}
