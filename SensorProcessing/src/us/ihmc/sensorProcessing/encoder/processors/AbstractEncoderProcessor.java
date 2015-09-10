package us.ihmc.sensorProcessing.encoder.processors;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;


public abstract class AbstractEncoderProcessor implements EncoderProcessor
{
   protected final YoVariableRegistry registry;
   protected final IntegerYoVariable rawTicks;
   protected final DoubleYoVariable time;

   protected final DoubleYoVariable processedTicks;
   protected final DoubleYoVariable processedTickRate;

   private final DoubleYoVariable processedPosition;
   private final DoubleYoVariable processedVelocity;

   private final String name;
   
   public AbstractEncoderProcessor(String name, IntegerYoVariable rawTicks, DoubleYoVariable time, double distancePerTick, YoVariableRegistry registry)
   {
      this.name = name;
      
      this.registry = registry;
      this.rawTicks = rawTicks;
      this.time = time;
      this.processedTicks = new DoubleYoVariable(name + "ProcTicks", registry);
      this.processedTickRate = new DoubleYoVariable(name + "ProcTickRate", registry);
      this.processedPosition = new DoubleYoVariable(name + "ProcPos", registry);
      this.processedVelocity = new DoubleYoVariable(name + "ProcVel", registry);

      processedTicks.addVariableChangedListener(new MultiplicationVariableChangedListener(processedPosition, distancePerTick));
      processedTickRate.addVariableChangedListener(new MultiplicationVariableChangedListener(processedVelocity, distancePerTick));
   }

   public final double getQ()
   {
      return processedPosition.getDoubleValue();
   }

   public final double getQd()
   {
      return processedVelocity.getDoubleValue();
   }

   public abstract void initialize();

   public abstract void update();

   private static final class MultiplicationVariableChangedListener implements VariableChangedListener
   {
      private final YoVariable output;
      private final double multiplicationFactor;

      public MultiplicationVariableChangedListener(YoVariable output, double multiplicationFactor)
      {
         this.output = output;
         this.multiplicationFactor = multiplicationFactor;
      }

      public void variableChanged(YoVariable variable)
      {
         output.setValueFromDouble(variable.getValueAsDouble() * multiplicationFactor);
      }
   }
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return name;
   }
}
