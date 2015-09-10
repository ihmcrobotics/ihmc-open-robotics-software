package us.ihmc.sensorProcessing.encoder.processors;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;


public class StateMachineSimpleEncoderProcessor extends AbstractEncoderProcessor
{
   private final DoubleYoVariable changeInTime;
   private final IntegerYoVariable changeInPosition;

   private final IntegerYoVariable previousPosition;
   private final DoubleYoVariable previousTime;

   private boolean hasBeenCalled = false;
   
   public StateMachineSimpleEncoderProcessor(String name, IntegerYoVariable rawTicks, DoubleYoVariable time, double distancePerTick, YoVariableRegistry registry)
   {
      super(name, rawTicks, time, distancePerTick, registry);

      this.previousPosition = new IntegerYoVariable(name + "PrevPos", registry);
      this.previousTime = new DoubleYoVariable(name + "PrevTime", registry);
      this.changeInPosition = new IntegerYoVariable(name + "ChangeInPos",  registry);
      this.changeInTime = new DoubleYoVariable(name + "ChangeInTime", registry);
   }
   
   public void initialize()
   {
      // empty
   }

   public void update()
   {
      if (!hasBeenCalled)
      {
         setPreviousValues();
         hasBeenCalled = true;
      }

      changeInPosition.set(rawTicks.getIntegerValue() - previousPosition.getIntegerValue());
      changeInTime.set(time.getDoubleValue() - previousTime.getDoubleValue());


      processedTicks.set(rawTicks.getIntegerValue());

      if (changeInTime.getDoubleValue() != 0.0)
         processedTickRate.set(((double) changeInPosition.getIntegerValue()) / changeInTime.getDoubleValue());
      else
         processedTickRate.set(0.0);

      setPreviousValues();
   }

   private void setPreviousValues()
   {
      previousPosition.set(rawTicks.getIntegerValue());
      previousTime.set(time.getDoubleValue());
   }
}
