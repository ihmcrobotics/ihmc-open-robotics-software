package us.ihmc.gdx.simulation.environment;

import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGui3DViewInputDebugger;

public class GDXEnvironmentBuilderUI extends Lwjgl3ApplicationAdapter
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/libgdx/resources",
                                                              "Environment Builder");
   private final GDXEnvironment environment = new GDXEnvironment(baseUI.get3DSceneManager());
   private final ImGui3DViewInputDebugger inputDebugger = new ImGui3DViewInputDebugger();

   public GDXEnvironmentBuilderUI()
   {
      baseUI.getImGuiPanelManager().addPanel(environment.getWindowName(), environment::renderImGuiWidgets);
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
