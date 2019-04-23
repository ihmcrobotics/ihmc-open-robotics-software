package us.ihmc.humanoidBehaviors.ui.tools;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

public class LocalParameterServer
{
   private static final long PERIOD_MS = 17; // 60Hz should be plenty

   private final YoVariableRegistry registry;
   private final Class<?> clazz;
   private final String lowerCaseName;
   private final int port;

   public static YoVariableRegistry create(Class<?> clazz, int port)
   {
      LocalParameterServer localParameterServer = new LocalParameterServer(clazz, port);
      return localParameterServer.getRegistry();
   }

   public LocalParameterServer(Class<?> clazz, int port)
   {
      this.clazz = clazz;
      this.port = port;
      lowerCaseName = clazz.getSimpleName().toLowerCase();
      registry = new YoVariableRegistry(lowerCaseName);
   }

   public void start()
   {
      LogTools.info("Starting YoVariableServer on port {}", port);
      ParameterLoaderHelper.loadParameters(getClass(), clazz.getClassLoader().getResourceAsStream(lowerCaseName + "Parameters.xml"), registry);
      YoVariableServer yoVariableServer = new YoVariableServer(clazz.getSimpleName(),
                                                               null,
                                                               new DataServerSettings(false,
                                                                                      DataServerSettings.DEFAULT_AUTODISCOVERABLE,
                                                                                      port,
                                                                                      null),
                                                               0.01);
      yoVariableServer.setMainRegistry(registry, null, null);
      ExceptionTools.handle(() -> yoVariableServer.start(), DefaultExceptionHandler.PRINT_STACKTRACE);
      ExceptionHandlingThreadScheduler scheduler = new ExceptionHandlingThreadScheduler(clazz.getSimpleName() + "YoVariableServer");
      AtomicLong timestamp = new AtomicLong();
      scheduler.schedule(() -> yoVariableServer.update(timestamp.getAndAdd(10000)), PERIOD_MS, TimeUnit.MILLISECONDS);
   }

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }
}
