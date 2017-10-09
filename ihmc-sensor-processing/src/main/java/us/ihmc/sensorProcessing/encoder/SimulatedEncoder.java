package us.ihmc.sensorProcessing.encoder;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SimulatedEncoder
{
   private final double encoderTicksPerUnitOfPosition;
   private YoDouble positionFromEncoder;
   private YoInteger encoderTicks;

   public SimulatedEncoder(double encoderTicksPerUnitOfPosition, String name, YoVariableRegistry parentRegistry)
   {
      if (encoderTicksPerUnitOfPosition <= 0.0)
         throw new RuntimeException("encoderTicksPerUnitOfPosition must be > 0.0");

      YoVariableRegistry registry = new YoVariableRegistry("simulatedEncoder_" + name);

      positionFromEncoder = new YoDouble("positionFromEncoder_" + name, registry);
      encoderTicks = new YoInteger("encoderTicks_" + name, registry);

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
