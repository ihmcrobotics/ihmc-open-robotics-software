package us.ihmc.sensorProcessing.encoder.processors;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class StateMachineSimpleEncoderProcessor extends AbstractEncoderProcessor
{
   private final DoubleYoVariable changeInTime;
   private final IntYoVariable changeInPosition;

   private final IntYoVariable previousPosition;
   private final DoubleYoVariable previousTime;

   private boolean hasBeenCalled = false;
   
   public StateMachineSimpleEncoderProcessor(String name, IntYoVariable rawTicks, DoubleYoVariable time, double distancePerTick, YoVariableRegistry registry)
   {
      super(name, rawTicks, time, distancePerTick, registry);

      this.previousPosition = new IntYoVariable(name + "PrevPos", registry);
      this.previousTime = new DoubleYoVariable(name + "PrevTime", registry);
      this.changeInPosition = new IntYoVariable(name + "ChangeInPos",  registry);
      this.changeInTime = new DoubleYoVariable(name + "ChangeInTime", registry);
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
