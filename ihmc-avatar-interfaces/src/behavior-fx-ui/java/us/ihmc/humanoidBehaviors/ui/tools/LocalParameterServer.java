package us.ihmc.humanoidBehaviors.ui.tools;

import us.ihmc.humanoidBehaviors.tools.thread.ExceptionPrintingThreadScheduler;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

public class LocalParameterServer
{
   private final YoVariableRegistry registry;

   public static YoVariableRegistry create(Class<?> clazz)
   {
      LocalParameterServer localParameterServer = new LocalParameterServer(clazz);
      return localParameterServer.getRegistry();
   }

   private LocalParameterServer(Class<?> clazz, )
   {
      registry = new YoVariableRegistry("vrui");
      ParameterLoaderHelper.loadParameters(getClass(), ClassLoader.getSystemClassLoader().getResourceAsStream("vrParameters.xml"), registry);
      YoVariableServer yoVariableServer = new YoVariableServer(robotModel.getSimpleRobotName() + getClass().getSimpleName(),
                                                               null,
                                                               new DataServerSettings(false),
                                                               0.01);
      yoVariableServer.setMainRegistry(registry, null, null);
      try
      {
         yoVariableServer.start();
      }
      catch(Throwable e)
      {
         e.printStackTrace();
      }
      ExceptionPrintingThreadScheduler scheduler = new ExceptionPrintingThreadScheduler(getClass().getSimpleName() + "YoVariableServer");
      AtomicLong timestamp = new AtomicLong();
      scheduler.schedule(() -> yoVariableServer.update(timestamp.getAndAdd(10000)), 17, TimeUnit.MILLISECONDS); // 60Hz should be plenty
   }

   private YoVariableRegistry getRegistry()
   {
      return registry;
   }
}
