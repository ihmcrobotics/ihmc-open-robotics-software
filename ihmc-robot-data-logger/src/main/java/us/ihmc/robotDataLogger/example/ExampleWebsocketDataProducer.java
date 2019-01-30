package us.ihmc.robotDataLogger.example;

import java.io.IOException;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotDataLogger.dataBuffers.RegistrySendBufferBuilder;
import us.ihmc.robotDataLogger.interfaces.RegistryPublisher;
import us.ihmc.robotDataLogger.rtps.CustomLogDataPublisherType;
import us.ihmc.robotDataLogger.util.PeriodicGCFreeNonRealtimeThreadSchedulerFactory;
import us.ihmc.robotDataLogger.websocket.server.WebsocketDataProducer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class ExampleWebsocketDataProducer
{
   private final YoVariableRegistry registry = new YoVariableRegistry("root");
   private final YoInteger counter = new YoInteger("counter", registry);

   private final RegistryPublisher publisher;
   private final WebsocketDataProducer producer;

   public ExampleWebsocketDataProducer() throws IOException
   {
      PeriodicGCFreeNonRealtimeThreadSchedulerFactory schedulerFactory = new PeriodicGCFreeNonRealtimeThreadSchedulerFactory();

      producer = new WebsocketDataProducer(null, null, null, false);

      RegistrySendBufferBuilder builder = new RegistrySendBufferBuilder(registry, null, null);
      builder.getVariables().addAll(registry.getAllVariablesIncludingDescendants());
      builder.build(0);
      
      CustomLogDataPublisherType type = new CustomLogDataPublisherType(builder.getNumberOfVariables(), builder.getNumberOfVariables());
      publisher = producer.createRegistryPublisher(type, schedulerFactory, builder);

   }

   public void run() throws IOException
   {
      producer.announce();
      publisher.start();
      long timestamp = 0;
      while (true)
      {
         publisher.update(timestamp++);
         counter.increment();
         ThreadTools.sleep(1);
      }
   }

   public static void main(String[] args) throws IOException
   {
      new ExampleWebsocketDataProducer().run();
   }
}
