package us.ihmc.rdx.simulation.environment;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.ImGui3DViewInputDebugger;

public class RDXEnvironmentBuilderUI extends Lwjgl3ApplicationAdapter
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Environment Builder");
   private final RDXEnvironmentBuilder environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
   private final ImGui3DViewInputDebugger inputDebugger = new ImGui3DViewInputDebugger();

   public RDXEnvironmentBuilderUI()
   {
      baseUI.getImGuiPanelManager().addPanel(environmentBuilder);
      baseUI.launchRDXApplication(this);
   }

   @Override
   public void create()
   {
      baseUI.create();

      inputDebugger.create(baseUI.getPrimary3DPanel());
      baseUI.getImGuiPanelManager().addPanel(inputDebugger.getWindowName(), inputDebugger::render);

      environmentBuilder.create();
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
      baseUI.dispose();
   }

   public static void main(String[] args)
   {
      new RDXEnvironmentBuilderUI();
   }
}
