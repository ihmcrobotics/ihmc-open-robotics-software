package us.ihmc.gdx.simulation.environment;

import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;

public class GDXEnvironmentBuilderUI extends Lwjgl3ApplicationAdapter
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/libgdx/resources",
                                                              "Environment Builder");
   private final GDXEnvironmentBuilderPanel environmentBuilderUI = new GDXEnvironmentBuilderPanel();

   public GDXEnvironmentBuilderUI()
   {
      baseUI.getImGuiDockingSetup().addWindow(environmentBuilderUI.getWindowName(), environmentBuilderUI::renderImGuiWindow);
      baseUI.launchGDXApplication(this);
   }

   @Override
   public void create()
   {
      baseUI.create();

      environmentBuilderUI.create(baseUI);
      baseUI.getSceneManager().addRenderableProvider(environmentBuilderUI);
   }

   @Override
   public void render()
   {
      baseUI.pollVREvents();

      environmentBuilderUI.handleVREvents();

      baseUI.renderBeforeOnScreenUI();

      baseUI.renderEnd();
   }

   @Override
   public void dispose()
   {
      baseUI.dispose();
   }

   public static void main(String[] args)
   {
      new GDXEnvironmentBuilderUI();
   }
}
