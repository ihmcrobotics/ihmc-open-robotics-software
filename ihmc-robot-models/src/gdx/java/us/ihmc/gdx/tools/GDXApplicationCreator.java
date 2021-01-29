package us.ihmc.gdx.tools;

import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;

public class GDXApplicationCreator
{
   public static void launchGDXApplication(Lwjgl3ApplicationAdapter application, Class<?> clazz)
   {
      launchGDXApplication(application, clazz.getSimpleName(), 1100, 800);
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
      applicationConfiguration.useVsync(true);
      applicationConfiguration.setBackBufferConfig(8, 8, 8, 8, 16, 0, 4);
      applicationConfiguration.useOpenGL3(true, 3, 2);
      return applicationConfiguration;
   }

   public static void launchGDXApplication(Lwjgl3ApplicationAdapter application, String title, double width, double height)
   {
      launchGDXApplication(getDefaultConfiguration(title, width, height), application, title);
   }

   public static void launchGDXApplication(Lwjgl3ApplicationConfiguration applicationConfiguration,
                                           Lwjgl3ApplicationAdapter application,
                                           String title)
   {
      ThreadTools.startAThread(() -> new Lwjgl3Application(application, applicationConfiguration), title);
   }
}
