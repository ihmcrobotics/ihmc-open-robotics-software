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
   private final GDXEnvironmentBuilderPanel environmentBuilderUI = new GDXEnvironmentBuilderPanel();
   private final GDXEnvironment environment = new GDXEnvironment();
   private final ImGui3DViewInputDebugger inputDebugger = new ImGui3DViewInputDebugger();

   public GDXEnvironmentBuilderUI()
   {
      baseUI.getImGuiDockingSetup().addWindow(environmentBuilderUI.getWindowName(), environmentBuilderUI::renderImGuiWindow);
      baseUI.launchGDXApplication(this);
   }

   @Override
   public void create()
   {
      baseUI.create();

      inputDebugger.create(baseUI);
      baseUI.getImGuiDockingSetup().addWindow(inputDebugger.getWindowName(), inputDebugger::render);

      environmentBuilderUI.create(baseUI);
      baseUI.getSceneManager().addRenderableProvider(environmentBuilderUI);

      environment.create(baseUI);
      baseUI.getImGuiDockingSetup().addWindow(environment.getWindowName(), environment::render);
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
