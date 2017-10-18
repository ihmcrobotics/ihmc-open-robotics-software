package us.ihmc.sensorProcessing.encoder.processors;

import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;


public abstract class AbstractEncoderProcessor implements EncoderProcessor
{
   protected final YoVariableRegistry registry;
   protected final YoInteger rawTicks;
   protected final YoDouble time;

   protected final YoDouble processedTicks;
   protected final YoDouble processedTickRate;

   private final YoDouble processedPosition;
   private final YoDouble processedVelocity;

   private final String name;
   
   public AbstractEncoderProcessor(String name, YoInteger rawTicks, YoDouble time, double distancePerTick, YoVariableRegistry registry)
   {
      this.name = name;
      
      this.registry = registry;
      this.rawTicks = rawTicks;
      this.time = time;
      this.processedTicks = new YoDouble(name + "ProcTicks", registry);
      this.processedTickRate = new YoDouble(name + "ProcTickRate", registry);
      this.processedPosition = new YoDouble(name + "ProcPos", registry);
      this.processedVelocity = new YoDouble(name + "ProcVel", registry);

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

      public void notifyOfVariableChange(YoVariable variable)
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
