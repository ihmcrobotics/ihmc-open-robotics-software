package us.ihmc.robotDataLogger.example;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This class shows a simple example to create a YoVariableServer with two registries that update at different rates.
 * 
 * Note that only a main registry is necessary for the YoVariableServer to work.
 * 
 * @author Jesper Smith
 *
 */
public class ExampleServer
{
   public enum SomeEnum
   {
      A, B, C, D, E, F;
   }
   

   private static final int variablesPerType = 200;
   private static final double dt = 0.001;
   private static final int mainRegistryUpdatesPerSecondRegistryUpdates = 10;
   private static final DataServerSettings logSettings = new DataServerSettings(true);

   private final Random random = new Random(127L);
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoVariableRegistry secondRegistry = new YoVariableRegistry(getClass().getSimpleName() + "Second");
   private final YoVariableServer yoVariableServer;
   
   private final List<YoVariable<?>> mainChangingVariables = new ArrayList<>();
   private final List<YoVariable<?>> secondChangingVariables = new ArrayList<>();

   private long timestamp = 0;
   private long counter = 0;

   public ExampleServer()
   {
      // Create variables for both registries
      createVariables("Main", variablesPerType, registry, mainChangingVariables);
      createVariables("Second", variablesPerType, secondRegistry, secondChangingVariables);

      // Create server
      yoVariableServer = new YoVariableServer(getClass(), null, logSettings, dt);
      // Add main registry to server
      yoVariableServer.setMainRegistry(registry , null, null);
      // Add second registry to server
      yoVariableServer.addRegistry(secondRegistry, null);
      
   }

   public void start()
   {
      // Start the server before the loop and after all registires are added to the server.
      yoVariableServer.start();

      LogTools.info("Starting to loop.");
      
      // Testing only. Sending main registry a few times before sending the second registry. This is helpfull to test merging packets
      for(int i = 0; i < 3; i++)
      {
         timestamp += Conversions.secondsToNanoseconds(dt);
         yoVariableServer.update(timestamp);
      }
      
      
      while (true)
      {
         // Increase timestamp and update variables
         timestamp += Conversions.secondsToNanoseconds(dt);
         
         // Adjust timestamp by +- 0.25 * dt to simulate jitter
         long dtFactor = Conversions.secondsToNanoseconds(dt)/2;
         long jitteryTimestamp = timestamp + ((long)((random.nextDouble() - 0.5) * dtFactor));

         
         updateVariables(mainChangingVariables);
         
         // Send main registry
         yoVariableServer.update(jitteryTimestamp);
         
         
         if(counter % mainRegistryUpdatesPerSecondRegistryUpdates == 0)
         {
            // Update second registry variables
            updateVariables(secondChangingVariables);
            
            // Send second registry.
            // If the timestamps match between the main and secondary registry, they get merged by the receiver.
            yoVariableServer.update(jitteryTimestamp, secondRegistry);
         }
         
         counter++;
         
         // Wait to not crash the network
         ThreadTools.sleepSeconds(dt);
      }
   }

   private void createVariables(String prefix, int variablesPerType, YoVariableRegistry registry, List<YoVariable<?>> allChangingVariables)
   {
      for (int i = 0; i < variablesPerType; i++)
      {
         new YoBoolean(prefix + "Boolean" + i, registry);
         new YoDouble(prefix + "Double" + i, registry);
         new YoInteger(prefix + "Integer" + i, registry);
         new YoLong(prefix + "Long" + i, registry);
         new YoEnum<>(prefix + "Enum" + i, registry, SomeEnum.class, random.nextBoolean());
      }
      
      
      allChangingVariables.addAll(registry.getAllVariablesIncludingDescendants());
      
      
      YoDouble input = new YoDouble(prefix + "Input", registry);
      YoDouble output = new YoDouble(prefix + "Output", registry);
      input.addVariableChangedListener((v) -> output.set(input.getValue()));
      
      
   }

   private void updateVariables(List<YoVariable<?>> allChangingVariables)
   {
      for (int varIdx = 0; varIdx < allChangingVariables.size(); varIdx++)
      {
         updateVariable(allChangingVariables.get(varIdx));
      }
   }

   private void updateVariable(YoVariable<?> variable)
   {
      if (variable instanceof YoBoolean)
      {
         ((YoBoolean) variable).set(random.nextBoolean());
      }
      else if (variable instanceof YoDouble)
      {
         ((YoDouble) variable).set(random.nextDouble());
      }
      else if (variable instanceof YoInteger)
      {
         ((YoInteger) variable).set(random.nextInt());
      }
      else if (variable instanceof YoLong)
      {
         ((YoLong) variable).set(random.nextLong());
      }
      else if (variable instanceof YoEnum<?>)
      {
         int enumSize = ((YoEnum<?>) variable).getEnumSize();
         ((YoEnum<?>) variable).set(random.nextInt(enumSize));
      }
      else
      {
         throw new RuntimeException("Implement this case for " + variable.getClass().getSimpleName() + ".");
      }
   }

   public static void main(String[] args)
   {
      LogTools.info("Starting " + ExampleServer.class.getSimpleName());
      ExampleServer exampleServer = new ExampleServer();
      exampleServer.start();
   }
}
