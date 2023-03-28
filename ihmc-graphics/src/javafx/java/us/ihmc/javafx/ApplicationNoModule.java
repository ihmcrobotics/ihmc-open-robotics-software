package us.ihmc.javafx;

import java.lang.reflect.InvocationTargetException;

import com.sun.javafx.application.LauncherImpl;

import javafx.application.Application;
import javafx.application.Application.Parameters;
import javafx.application.HostServices;
import javafx.application.Preloader.PreloaderNotification;
import javafx.stage.Stage;

/**
 * Workaround to create a JavaFX {@link Application} without the use of the modules.
 */
public abstract class ApplicationNoModule
{
   private ApplicationImpl impl;

   public ApplicationNoModule()
   {
   }

   public void init() throws Exception
   {
   }

   public abstract void start(Stage primaryStage) throws Exception;

   public void stop() throws Exception
   {
   }

   public final HostServices getHostServices()
   {
      if (impl == null)
         impl = new ApplicationImpl(false);
      return impl.getHostServices();
   }

   public final Parameters getParameters()
   {
      if (impl == null)
         impl = new ApplicationImpl(false);
      return impl.getParameters();
   }

   public final void notifyPreloader(PreloaderNotification info)
   {
      if (impl == null)
         impl = new ApplicationImpl(false);
      impl.notifyPreloader(info);
   }

   public static String getUserAgentStylesheet()
   {
      return Application.getUserAgentStylesheet();
   }

   public static void setUserAgentStylesheet(String url)
   {
      Application.setUserAgentStylesheet(url);
   }

   public static class ApplicationImpl extends Application
   {
      private static Class<? extends ApplicationNoModule> classToInstatiate;
      private ApplicationNoModule appToManage;

      private ApplicationImpl(boolean dummyParameter)
      {
      }

      public ApplicationImpl()
            throws InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException, NoSuchMethodException, SecurityException
      {
         if (classToInstatiate != null)
            appToManage = classToInstatiate.getConstructor().newInstance();
         classToInstatiate = null;
         appToManage.impl = this;
      }

      @Override
      public void init() throws Exception
      {
         appToManage.init();
      }

      @Override
      public void start(Stage primaryStage) throws Exception
      {
         appToManage.start(primaryStage);
      }

      @Override
      public void stop() throws Exception
      {
         appToManage.stop();
      }
   }

   public static void launch(Class<? extends ApplicationNoModule> appClass, String... args)
   {
      ApplicationImpl.classToInstatiate = appClass;
      LauncherImpl.launchApplication(ApplicationImpl.class, args);
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   public static void launch(String... args)
   {
      // Figure out the right class to call
      StackTraceElement[] cause = Thread.currentThread().getStackTrace();

      boolean foundThisMethod = false;
      String callingClassName = null;
      for (StackTraceElement se : cause)
      {
         // Skip entries until we get to the entry for this class
         String className = se.getClassName();
         String methodName = se.getMethodName();
         if (foundThisMethod)
         {
            callingClassName = className;
            break;
         }
         else if (ApplicationNoModule.class.getName().equals(className) && "launch".equals(methodName))
         {

            foundThisMethod = true;
         }
      }

      if (callingClassName == null)
      {
         throw new RuntimeException("Error: unable to determine Application class");
      }

      try
      {
         Class theClass = Class.forName(callingClassName, false, Thread.currentThread().getContextClassLoader());

         if (ApplicationNoModule.class.isAssignableFrom(theClass))
         {
            Class<? extends ApplicationNoModule> appClass = theClass;
            launch(appClass, args);
         }
         else
         {
            throw new RuntimeException("Error: " + theClass + " is not a subclass of javafx.application.Application");
         }
      }
      catch (RuntimeException ex)
      {
         throw ex;
      }
      catch (Exception ex)
      {
         throw new RuntimeException(ex);
      }
   }
}
