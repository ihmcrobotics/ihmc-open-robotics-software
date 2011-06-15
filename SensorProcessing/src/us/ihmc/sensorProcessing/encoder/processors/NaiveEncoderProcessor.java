package us.ihmc.sensorProcessing.encoder.processors;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class NaiveEncoderProcessor implements EncoderProcessor
{
   private final IntYoVariable rawPosition;
   private final DoubleYoVariable processedPosition, processedRate;
   private final DoubleYoVariable previousPosition, previousTime;

   private final DoubleYoVariable time;

   private double unitDistancePerCount = 1.0;

   public NaiveEncoderProcessor(String name, IntYoVariable rawPosition, DoubleYoVariable time, YoVariableRegistry registry)
   {
      this.rawPosition = rawPosition;
      this.time = time;

      this.processedPosition = new DoubleYoVariable(name + "procPos", registry);
      this.processedRate = new DoubleYoVariable(name + "procRate", registry);

      this.previousPosition = new DoubleYoVariable(name + "prevPos", registry);
      this.previousTime = new DoubleYoVariable(name + "prevTime", registry);
   }

   public void update()
   {
      processedPosition.set(rawPosition.getIntegerValue());
      double dx = rawPosition.getIntegerValue() - previousPosition.getDoubleValue();
      double dt = time.getDoubleValue() - previousTime.getDoubleValue();
      processedRate.set(dx / dt);

      previousPosition.set(rawPosition.getIntegerValue());
      previousTime.set(time.getDoubleValue());
   }

   public double getQ()
   {
      return this.processedPosition.getDoubleValue() * unitDistancePerCount;
   }

   public double getQd()
   {
      return this.processedRate.getDoubleValue() * unitDistancePerCount;
   }
   
   public void setUnitDistancePerCount(double unitDistancePerCount)
   {
      this.unitDistancePerCount = unitDistancePerCount;
   }
}
