package us.ihmc.gdx;

import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import us.ihmc.commons.thread.ThreadTools;

public class GDXApplicationCreator
{
   public static void launchGDXApplication(Lwjgl3ApplicationAdapter application, Class<?> clazz)
   {
      launchGDXApplication(application, clazz.getSimpleName(), 1100, 800);
   }

   public static void launchGDXApplication(Lwjgl3ApplicationAdapter application, String title, double width, double height)
   {
      Lwjgl3ApplicationConfiguration applicationConfiguration = new Lwjgl3ApplicationConfiguration();
      applicationConfiguration.setTitle(title);
      applicationConfiguration.setWindowedMode((int) width, (int) height);
      applicationConfiguration.useVsync(true);
      applicationConfiguration.setBackBufferConfig(8, 8, 8, 8, 16, 0, 4);
      applicationConfiguration.useOpenGL3(true, 3, 2);

      ThreadTools.startAThread(() -> new Lwjgl3Application(application, applicationConfiguration), title);
   }
}
