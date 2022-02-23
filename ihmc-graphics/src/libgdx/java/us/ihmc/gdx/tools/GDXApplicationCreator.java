package us.ihmc.gdx.tools;

import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;

public class GDXApplicationCreator
{
   public static void launchGDXApplication(Lwjgl3ApplicationAdapter applicationAdapter, Class<?> clazz)
   {
      launchGDXApplication(applicationAdapter, clazz.getSimpleName(), 1100, 800);
   }

   public static Lwjgl3ApplicationConfiguration getDefaultConfiguration(Class<?> clazz)
   {
      return getDefaultConfiguration(clazz.getSimpleName(), 1100, 800);
   }

   public static Lwjgl3ApplicationConfiguration getDefaultConfiguration(String title, double width, double height)
   {
      Lwjgl3ApplicationConfiguration applicationConfiguration = new Lwjgl3ApplicationConfiguration();
      applicationConfiguration.setTitle(title);
      applicationConfiguration.setWindowedMode((int) width, (int) height);
      // TODO: These options are work in progress. Not sure what is the best setting for everyone.
      if (Boolean.parseBoolean(System.getProperty("gdx.free.spin")))
      {
         applicationConfiguration.setIdleFPS(Integer.MAX_VALUE);
         applicationConfiguration.setForegroundFPS(Integer.MAX_VALUE);
      }
      else
      {
         applicationConfiguration.setIdleFPS(30); // probably need to implement pause before idle FPS does anything
         applicationConfiguration.setForegroundFPS(240);
      }
      applicationConfiguration.useVsync(false); // vsync on seems to limit FPS to 30 so keep off
      applicationConfiguration.setBackBufferConfig(8, 8, 8, 8, 16, 0, 4);
      applicationConfiguration.useOpenGL3(true, 3, 3);
      applicationConfiguration.setWindowIcon("icons/gdx-icon16.png", "icons/gdx-icon32.png", "icons/gdx-icon48.png");
      return applicationConfiguration;
   }

   public static void launchGDXApplication(Lwjgl3ApplicationAdapter applicationAdapter, String title, double width, double height)
   {
      launchGDXApplication(getDefaultConfiguration(title, width, height), applicationAdapter, title);
   }

   public static void launchGDXApplication(Lwjgl3ApplicationConfiguration applicationConfiguration, Lwjgl3ApplicationAdapter applicationAdapter, String title)
   {
      ThreadTools.startAThread(() ->
      {
         new Lwjgl3Application(applicationAdapter, applicationConfiguration);
      }, title);
   }
}
