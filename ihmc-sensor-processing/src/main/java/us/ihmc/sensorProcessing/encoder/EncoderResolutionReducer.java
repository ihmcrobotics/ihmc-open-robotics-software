package us.ihmc.sensorProcessing.encoder;


import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.robotController.SensorProcessor;

public class EncoderResolutionReducer implements SensorProcessor
{
   private final YoInteger rawTicksIn;
   private final YoInteger rawTicksOut;
   private final int reductionFactor;
   
   public EncoderResolutionReducer(String name, YoInteger rawTicksIn, int reductionFactor, YoVariableRegistry registry)
   {
      this.rawTicksIn = rawTicksIn;
      this.rawTicksOut = new YoInteger(name + "rawTicksOut", registry);
      this.reductionFactor = reductionFactor;
   }

   public void initialize()
   {
      update();
   }

   public void update()
   {
      rawTicksOut.set(rawTicksIn.getIntegerValue() / reductionFactor);
   }
   
   public YoInteger getRawTicksOut()
   {
      return rawTicksOut;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   public String getName()
   {
      return "EncoderResolutionReducer";
   }

   public String getDescription()
   {
      return "EncoderResolutionReducer";
   }
}
