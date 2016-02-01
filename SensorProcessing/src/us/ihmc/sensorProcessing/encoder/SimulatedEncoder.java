package us.ihmc.sensorProcessing.encoder;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;


public class SimulatedEncoder
{
   private final double encoderTicksPerUnitOfPosition;
   private DoubleYoVariable positionFromEncoder;
   private IntegerYoVariable encoderTicks;

   public SimulatedEncoder(double encoderTicksPerUnitOfPosition, String name, YoVariableRegistry parentRegistry)
   {
      if (encoderTicksPerUnitOfPosition <= 0.0)
         throw new RuntimeException("encoderTicksPerUnitOfPosition must be > 0.0");

      YoVariableRegistry registry = new YoVariableRegistry("simulatedEncoder_" + name);

      positionFromEncoder = new DoubleYoVariable("positionFromEncoder_" + name, registry);
      encoderTicks = new IntegerYoVariable("encoderTicks_" + name, registry);

      this.encoderTicksPerUnitOfPosition = encoderTicksPerUnitOfPosition;
      parentRegistry.addChild(registry);
   }

   public void setActualPosition(double actualPosition)
   {
      encoderTicks.set((int) Math.round(actualPosition * encoderTicksPerUnitOfPosition));
      positionFromEncoder.set(converTicksToDistance(encoderTicks.getIntegerValue()));
   }

   public double getPositionFromEncoder()
   {
      return positionFromEncoder.getDoubleValue();
   }

   public int getEncoderTicks()
   {
      return encoderTicks.getIntegerValue();
   }

   public double converTicksToDistance(int ticks)
   {
      return ((double) ticks) / encoderTicksPerUnitOfPosition;
   }

   public double converTicksToDistance(double ticks)
   {
      return ticks / encoderTicksPerUnitOfPosition;
   }

   public double getEncoderTicksPerUnitOfPosition()
   {
      return encoderTicksPerUnitOfPosition;
   }
}
