package us.ihmc.javafx;

import java.io.IOException;
import java.net.URL;
import java.util.concurrent.Callable;
import java.util.concurrent.CountDownLatch;

import org.apache.commons.lang3.mutable.MutableObject;

import com.sun.javafx.application.PlatformImpl;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.fxml.FXMLLoader;
import javafx.stage.Stage;
import us.ihmc.log.LogTools;

public class JavaFXMissingTools
{
   public static void startup()
   {
      PlatformImpl.startup(() ->
      {
         // Just to make sure JavaFX thread has started. 
      });
   }

   public static void runNFramesLater(int numberOfFramesToWait, Runnable runnable)
   {
      new AnimationTimer()
      {
         int counter = 0;

         @Override
         public void handle(long now)
         {
            if (counter++ > numberOfFramesToWait)
            {
               runnable.run();
               stop();
            }
         }
      }.start();
   }

   public static void runApplication(ApplicationNoModule application)
   {
      runApplication(application, null);
   }

   public static void runApplication(ApplicationNoModule application, Runnable initialize)
   {
      Runnable runnable = () ->
      {
         try
         {
            application.start(new Stage());
            if (initialize != null)
               initialize.run();
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      };

      PlatformImpl.startup(() ->
      {
         Platform.runLater(runnable);
      });
      PlatformImpl.setImplicitExit(false);
   }

   public static void runApplication(Application application)
   {
      runApplication(application, null);
   }

   public static void runApplication(Application application, Runnable initialize)
   {
      Runnable runnable = () ->
      {
         try
         {
            application.start(new Stage());
            if (initialize != null)
               initialize.run();
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      };

      PlatformImpl.startup(() ->
      {
         Platform.runLater(runnable);
      });
      PlatformImpl.setImplicitExit(false);
   }

   /**
    * - Controller class name must match the .fxml file name. - Must not set fx:contoller in FXML file
    * - This method expects .fxml to be in same package path as controller class
    */
   public static <T> T loadFromFXML(Object controller)
   {
      FXMLLoader loader = new FXMLLoader();
      loader.setController(controller);
      URL resource = controller.getClass().getResource(controller.getClass().getSimpleName() + ".fxml");
      if (resource == null)
      {
         throw new RuntimeException("Cannot find " + controller.getClass().getSimpleName() + ".fxml, please check the resource path.");
      }
      loader.setLocation(resource);

      try
      {
         return loader.load();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static void runAndWait(final Runnable runnable)
   {
      if (Platform.isFxApplicationThread())
      {
         try
         {
            runnable.run();
         }
         catch (Throwable t)
         {
            LogTools.error("Exception in runnable");
            t.printStackTrace();
         }
      }
      else
      {
         final CountDownLatch doneLatch = new CountDownLatch(1);

         Platform.runLater(() ->
         {
            try
            {
               runnable.run();
            }
            finally
            {
               doneLatch.countDown();
            }
         });

         try
         {
            doneLatch.await();
         }
         catch (InterruptedException ex)
         {
            ex.printStackTrace();
         }
      }
   }

   public static <R> R runAndWait(final Callable<R> callable)
   {
      if (Platform.isFxApplicationThread())
      {
         try
         {
            return callable.call();
         }
         catch (Throwable t)
         {
            LogTools.error("Exception in callable");
            t.printStackTrace();
            return null;
         }
      }
      else
      {
         final CountDownLatch doneLatch = new CountDownLatch(1);
         final MutableObject<R> result = new MutableObject<>();

         Platform.runLater(() ->
         {
            try
            {
               result.setValue(callable.call());
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
            finally
            {
               doneLatch.countDown();
            }
         });

         try
         {
            doneLatch.await();
            return result.getValue();
         }
         catch (InterruptedException ex)
         {
            ex.printStackTrace();
            return null;
         }
      }
   }
}
