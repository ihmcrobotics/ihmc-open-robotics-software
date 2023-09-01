package us.ihmc.rdx.simulation.environment;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.ImGui3DViewInputDebugger;

public class RDXSCS2EnvironmentBuilderUI extends Lwjgl3ApplicationAdapter
{
   private final RDXBaseUI baseUI = new RDXBaseUI("SCS 2 Environment Builder");
   private final RDXSCS2EnvironmentBuilder environmentBuilder = new RDXSCS2EnvironmentBuilder(baseUI.getPrimary3DPanel());
   private final ImGui3DViewInputDebugger inputDebugger = new ImGui3DViewInputDebugger();

   public RDXSCS2EnvironmentBuilderUI()
   {
      baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getWindowName(), environmentBuilder::renderImGuiWidgets);
      baseUI.launchRDXApplication(this);
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
      new RDXSCS2EnvironmentBuilderUI();
   }
}
