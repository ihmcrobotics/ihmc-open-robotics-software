package us.ihmc.gdx.simulation.environment;

import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGui3DViewInputDebugger;

public class GDXEnvironmentBuilderUI extends Lwjgl3ApplicationAdapter
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/libgdx/resources",
                                                              "Environment Builder");
   private final GDXEnvironment environment = new GDXEnvironment();
   private final ImGui3DViewInputDebugger inputDebugger = new ImGui3DViewInputDebugger();

   public GDXEnvironmentBuilderUI()
   {
      baseUI.getImGuiPanelManager().addPanel(environment.getWindowName(), environment::renderImGuiWidgets);
      baseUI.getImGuiPanelManager().addPanel(GDX3DSceneTools.TUNING_WINDOW_NAME, GDX3DSceneTools::renderTuningSliders);
      baseUI.launchGDXApplication(this);
   }

   @Override
   public void create()
   {
      baseUI.create();

      inputDebugger.create(baseUI);
      baseUI.getImGuiPanelManager().addPanel(inputDebugger.getWindowName(), inputDebugger::render);

      environment.create(baseUI);
   }

   @Override
   public void render()
   {
      baseUI.pollVREvents();

      baseUI.renderBeforeOnScreenUI();

      baseUI.renderEnd();
   }

   @Override
   public void dispose()
   {
      environment.destroy();
      baseUI.dispose();
   }

   public static void main(String[] args)
   {
      new GDXEnvironmentBuilderUI();
   }
}
