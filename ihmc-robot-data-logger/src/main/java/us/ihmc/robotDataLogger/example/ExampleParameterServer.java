package us.ihmc.robotDataLogger.example;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.example.ExampleServer.SomeEnum;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.yoVariables.listener.ParameterChangedListener;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.EnumParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ExampleParameterServer
{
   private static final double dt = 0.001;
   private static final DataServerSettings logSettings = new DataServerSettings(false, false);

   private final YoVariableServer yoVariableServer;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private long timestamp = 0;

   public ExampleParameterServer()
   {
      createVariables(5, registry);
      yoVariableServer = new YoVariableServer(getClass(), null, logSettings, dt);
      yoVariableServer.setMainRegistry(registry, null, null);
      new DefaultParameterReader().readParametersInRegistry(registry);
      ParameterChangedListener changedPrinter = p -> System.out.println(p.getName() + " changed to " + p.getValueAsString());
      registry.getAllParameters().forEach(p -> p.addParameterChangedListener(changedPrinter ));
   }

   public void start()
   {
      yoVariableServer.start();
      while (true)
      {
         timestamp += Conversions.secondsToNanoseconds(dt);
         yoVariableServer.update(timestamp);
         ThreadTools.sleepSeconds(dt);
      }
   }

   private void createVariables(int variablesPerType, YoVariableRegistry parent)
   {
      for (int i = 0; i < variablesPerType; i++)
      {
         YoVariableRegistry registry = new YoVariableRegistry("Registry" + i);
         new BooleanParameter("BooleanParameter" + i, registry);
         new DoubleParameter("DoubleParameter" + i, registry);
         new IntegerParameter("IntegerParameter" + i, registry);
         new EnumParameter<>("EnumParameter" + i, registry, SomeEnum.class, true);
         parent.addChild(registry);
      }
   }

   public static void main(String[] args)
   {
      LogTools.info("Starting " + ExampleParameterServer.class.getSimpleName());
      ExampleParameterServer exampleServer = new ExampleParameterServer();
      exampleServer.start();
   }
}
