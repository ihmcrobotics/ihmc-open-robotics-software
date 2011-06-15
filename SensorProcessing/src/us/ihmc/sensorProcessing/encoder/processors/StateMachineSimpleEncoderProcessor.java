package us.ihmc.sensorProcessing.encoder.processors;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class StateMachineSimpleEncoderProcessor implements EncoderProcessor
{
   private final IntYoVariable rawPosition; 
   private final DoubleYoVariable changeInTime;
   private final DoubleYoVariable processedPosition, processedRate;
   private final IntYoVariable changeInPosition;

   private final IntYoVariable previousPosition;
   private final DoubleYoVariable previousTime;

   private final DoubleYoVariable time;

   private boolean hasBeenCalled;

   private double unitDistancePerCount = 1.0;
   
   public StateMachineSimpleEncoderProcessor(String name, IntYoVariable rawPosition, DoubleYoVariable time, YoVariableRegistry parentRegistry)
   {
      this.rawPosition = rawPosition;
      this.time = time;

      name = name + "_";

      YoVariableRegistry registry = new YoVariableRegistry(name);

      this.processedPosition = new DoubleYoVariable(name + "procPos", registry);
      this.processedRate = new DoubleYoVariable(name + "procRate", registry);

      this.previousPosition = new IntYoVariable(name + "prevPos", registry);
      this.previousTime = new DoubleYoVariable(name + "prevTime", registry);
      this.changeInPosition = new IntYoVariable(name + "changeInPos",  registry);
      this.changeInTime = new DoubleYoVariable(name + "changeInTime", registry);

      parentRegistry.addChild(registry);

      hasBeenCalled = false;
   }

   public double getQ()
   {
      return processedPosition.getDoubleValue() * unitDistancePerCount;
   }

   public double getQd()
   {
      return processedRate.getDoubleValue() * unitDistancePerCount;
   }

   public void update()
   {
      if (!hasBeenCalled)
      {
         setPreviousValues();
         hasBeenCalled = true;
      }

      changeInPosition.set(rawPosition.getIntegerValue() - previousPosition.getIntegerValue());
      changeInTime.set(time.getDoubleValue() - previousTime.getDoubleValue());


      processedPosition.set(rawPosition.getIntegerValue());

      if (changeInTime.getDoubleValue() != 0.0)
         processedRate.set(((double) changeInPosition.getIntegerValue()) / changeInTime.getDoubleValue());
      else
         processedRate.set(0.0);

      setPreviousValues();
   }

   private void setPreviousValues()
   {
      previousPosition.set(rawPosition.getIntegerValue());
      previousTime.set(time.getDoubleValue());
   }
   
   public void setUnitDistancePerCount(double unitDistancePerCount)
   {
      this.unitDistancePerCount = unitDistancePerCount;
   }

}
