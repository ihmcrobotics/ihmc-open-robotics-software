package us.ihmc.sensorProcessing.encoder.processors;


import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;


public class NonlinearObserverEncoderProcessor extends AbstractEncoderProcessor
{
   private final DoubleYoVariable discretePosition;
   private final DoubleYoVariable error, alpha1, alpha2;

   private final DoubleYoVariable previousTime;
   
   public NonlinearObserverEncoderProcessor(String name, IntegerYoVariable rawTicks, DoubleYoVariable time, double distancePerTick, YoVariableRegistry registry)
   {
      super(name, rawTicks, time, distancePerTick, registry);
      this.previousTime = new DoubleYoVariable(name + "PrevTime", registry);

      this.discretePosition = new DoubleYoVariable(name + "DiscretePosition", registry);
      this.error = new DoubleYoVariable(name + "Error", registry);

      this.alpha1 = new DoubleYoVariable(name + "Alpha1", registry);
      this.alpha2 = new DoubleYoVariable(name + "Alpha2", registry);

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
