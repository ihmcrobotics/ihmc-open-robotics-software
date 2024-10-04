package us.ihmc.rdx.tools;

import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
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
      applicationConfiguration.setBackBufferConfig(8, 8, 8, 8, 16, 0, 4);
      applicationConfiguration.setOpenGLEmulation(Lwjgl3ApplicationConfiguration.GLEmulation.GL30, 4, 6);
      applicationConfiguration.setWindowIcon("icons/rdx-icon16.png", "icons/rdx-icon32.png", "icons/rdx-icon48.png");
      return applicationConfiguration;
   }

   /**
    * Launches a LibGDX application with the specified adapter, window title, width, and height.
    * Will block until a shutdown is requested.
    *
    * @param applicationAdapter the adapter for the application
    * @param title the title of the application window
    * @param width the width of the application window
    * @param height the height of the application window
    */
   public static void launchGDXApplication(Lwjgl3ApplicationAdapter applicationAdapter, String title, double width, double height)
   {
      launchGDXApplication(getDefaultConfiguration(title, width, height), applicationAdapter);
   }

   /**
    * Launches a LibGDX application using the specified configuration and adapter.
    * Will block until a shutdown is requested.
    *
    * @param applicationConfiguration the configuration for the application
    * @param applicationAdapter the adapter for the application
    */
   public static void launchGDXApplication(Lwjgl3ApplicationConfiguration applicationConfiguration, Lwjgl3ApplicationAdapter applicationAdapter)
   {
      new Lwjgl3Application(applicationAdapter, applicationConfiguration);
   }
}
