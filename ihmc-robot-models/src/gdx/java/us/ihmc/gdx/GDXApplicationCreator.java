package us.ihmc.gdx;

import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;

public class GDXApplicationCreator
{
   public static Lwjgl3Application launchGDXApplication(Lwjgl3ApplicationAdapter application,
                                                        String title,
                                                        double width,
                                                        double height)
   {
      Lwjgl3ApplicationConfiguration applicationConfiguration = new Lwjgl3ApplicationConfiguration();
      applicationConfiguration.setTitle("GDX3DDemo");
      applicationConfiguration.setWindowedMode((int) width, (int) height);
      applicationConfiguration.useVsync(true);

      return new Lwjgl3Application(application, applicationConfiguration);
   }
}
