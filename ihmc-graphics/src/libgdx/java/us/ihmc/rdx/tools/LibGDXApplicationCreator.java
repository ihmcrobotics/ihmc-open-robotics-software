package us.ihmc.rdx.tools;

import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;

public class LibGDXApplicationCreator
{
   public static final int DEFAULT_WINDOW_WIDTH = 800;
   public static final int DEFAULT_WINDOW_HEIGHT = 600;

   public static void launchGDXApplication(Lwjgl3ApplicationAdapter applicationAdapter, Class<?> clazz)
   {
      launchGDXApplication(applicationAdapter, clazz.getSimpleName(), DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);
   }

   public static Lwjgl3ApplicationConfiguration getDefaultConfiguration(Class<?> clazz)
   {
      return getDefaultConfiguration(clazz.getSimpleName(), DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);
   }

   public static Lwjgl3ApplicationConfiguration getDefaultConfiguration(String title)
   {
      return getDefaultConfiguration(title, DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);
   }

   public static Lwjgl3ApplicationConfiguration getDefaultConfiguration(String title, double width, double height)
   {
      Lwjgl3ApplicationConfiguration applicationConfiguration = new Lwjgl3ApplicationConfiguration();
      applicationConfiguration.setTitle(title);
      applicationConfiguration.setWindowedMode((int) width, (int) height);
      // TODO: These options are work in progress. Not sure what is the best setting for everyone.
      if (Boolean.parseBoolean(System.getProperty("rdx.free.spin")))
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
      applicationConfiguration.setOpenGLEmulation(Lwjgl3ApplicationConfiguration.GLEmulation.GL30, 4, 6);
      applicationConfiguration.setWindowIcon("icons/rdx-icon16.png", "icons/rdx-icon32.png", "icons/rdx-icon48.png");
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
