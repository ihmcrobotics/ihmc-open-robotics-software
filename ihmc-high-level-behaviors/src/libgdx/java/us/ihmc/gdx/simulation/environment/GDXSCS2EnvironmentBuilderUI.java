package us.ihmc.gdx.simulation.environment;

import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGui3DViewInputDebugger;

public class GDXSCS2EnvironmentBuilderUI extends Lwjgl3ApplicationAdapter
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/libgdx/resources",
                                                              "SCS 2 Environment Builder");
   private final GDXSCS2EnvironmentBuilder environmentBuilder = new GDXSCS2EnvironmentBuilder(baseUI.getPrimary3DPanel());
   private final ImGui3DViewInputDebugger inputDebugger = new ImGui3DViewInputDebugger();

   public GDXSCS2EnvironmentBuilderUI()
   {
      baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getWindowName(), environmentBuilder::renderImGuiWidgets);
      baseUI.launchGDXApplication(this);
   }

   @Override
   public void create()
   {
      baseUI.create();

      inputDebugger.create(baseUI.getPrimary3DPanel());
      baseUI.getImGuiPanelManager().addPanel(inputDebugger.getWindowName(), inputDebugger::render);

      environmentBuilder.create(baseUI);
   }

   @Override
   public void render()
   {
      environmentBuilder.update();

      baseUI.renderBeforeOnScreenUI();
      baseUI.renderEnd();
   }

   @Override
   public void dispose()
   {
      environmentBuilder.destroy();
      baseUI.dispose();
   }

   public static void main(String[] args)
   {
      new GDXSCS2EnvironmentBuilderUI();
   }
}
