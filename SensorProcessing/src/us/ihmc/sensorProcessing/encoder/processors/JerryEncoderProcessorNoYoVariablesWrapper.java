package us.ihmc.sensorProcessing.encoder.processors;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;

public class JerryEncoderProcessorNoYoVariablesWrapper extends JerryEncoderProcessorNoYoVariables implements EncoderProcessor
{
   private final IntegerYoVariable rawTicks;
   private final DoubleYoVariable time;
   
   public JerryEncoderProcessorNoYoVariablesWrapper(String name, IntegerYoVariable rawTicks, DoubleYoVariable time, double distancePerTick, double dt)
   {
      super(dt, distancePerTick);
      
      this.rawTicks = rawTicks;
      this.time = time;
   }
   
   public void update()
   {      
      super.update(rawTicks.getIntegerValue(), time.getDoubleValue());
   }

   public void initialize()
   {
      this.update();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   public String getName()
   {
      return "JerryEncoderProcessorNoYoVariables";
   }

   public String getDescription()
   {
      return getName();
   }

}

