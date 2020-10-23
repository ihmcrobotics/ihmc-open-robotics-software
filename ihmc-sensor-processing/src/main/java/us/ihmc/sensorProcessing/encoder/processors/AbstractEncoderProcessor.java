package us.ihmc.sensorProcessing.encoder.processors;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;


public abstract class AbstractEncoderProcessor implements EncoderProcessor
{
   protected final YoRegistry registry;
   protected final YoInteger rawTicks;
   protected final YoDouble time;

   protected final YoDouble processedTicks;
   protected final YoDouble processedTickRate;

   private final YoDouble processedPosition;
   private final YoDouble processedVelocity;

   private final String name;
   
   public AbstractEncoderProcessor(String name, YoInteger rawTicks, YoDouble time, double distancePerTick, YoRegistry registry)
   {
      this.name = name;
      
      this.registry = registry;
      this.rawTicks = rawTicks;
      this.time = time;
      this.processedTicks = new YoDouble(name + "ProcTicks", registry);
      this.processedTickRate = new YoDouble(name + "ProcTickRate", registry);
      this.processedPosition = new YoDouble(name + "ProcPos", registry);
      this.processedVelocity = new YoDouble(name + "ProcVel", registry);

      processedTicks.addListener(new MultiplicationVariableChangedListener(processedPosition, distancePerTick));
      processedTickRate.addListener(new MultiplicationVariableChangedListener(processedVelocity, distancePerTick));
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

   private static final class MultiplicationVariableChangedListener implements YoVariableChangedListener
   {
      private final YoVariable output;
      private final double multiplicationFactor;

      public MultiplicationVariableChangedListener(YoVariable output, double multiplicationFactor)
      {
         this.output = output;
         this.multiplicationFactor = multiplicationFactor;
      }

      public void changed(YoVariable variable)
      {
         output.setValueFromDouble(variable.getValueAsDouble() * multiplicationFactor);
      }
   }
   
   public YoRegistry getYoRegistry()
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
