package us.ihmc.sensorProcessing.encoder.processors;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

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

