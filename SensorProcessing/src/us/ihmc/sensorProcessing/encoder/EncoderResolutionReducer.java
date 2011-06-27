package us.ihmc.sensorProcessing.encoder;

import us.ihmc.utilities.sensors.SensorProcessor;

import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class EncoderResolutionReducer implements SensorProcessor
{
   private final IntYoVariable rawTicksIn;
   private final IntYoVariable rawTicksOut;
   private final int reductionFactor;
   
   public EncoderResolutionReducer(String name, IntYoVariable rawTicksIn, int reductionFactor, YoVariableRegistry registry)
   {
      this.rawTicksIn = rawTicksIn;
      this.rawTicksOut = new IntYoVariable(name + "rawTicksOut", registry);
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
   
   public IntYoVariable getRawTicksOut()
   {
      return rawTicksOut;
   }
}
