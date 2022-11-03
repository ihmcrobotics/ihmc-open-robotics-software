package us.ihmc.rdx.simulation;

import imgui.ImGui;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2BulletSimulationSession;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.examples.simulations.bullet.FallingBoxBulletSimulation;
import us.ihmc.scs2.examples.simulations.bullet.FallingSphereExperimentalBulletSimulation;

public class RDXSCS2BulletSimulationDemos
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private RDXSCS2BulletSimulationSession scs2SimulationSession;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   public RDXSCS2BulletSimulationDemos()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);

            baseUI.getImGuiPanelManager().addPanel("Session Selection", this::renderSessionSelectionPanelImGuiWidgets);

            scs2SimulationSession = new RDXSCS2BulletSimulationSession();
            scs2SimulationSession.create(baseUI);
            scs2SimulationSession.startSession(FallingBoxBulletSimulation.createSession());
            baseUI.getImGuiPanelManager().addPanel(scs2SimulationSession.getControlPanel());
         }

         @Override
         public void render()
         {
            scs2SimulationSession.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderSessionSelectionPanelImGuiWidgets()
         {
            if (ImGui.button(labels.get("Falling Box")))
            {
               scs2SimulationSession.startSession(FallingBoxBulletSimulation.createSession());
            }
            if (ImGui.button(labels.get("Falling Sphere")))
            {
               scs2SimulationSession.startSession(FallingSphereExperimentalBulletSimulation.createSession());
            }

         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXSCS2BulletSimulationDemos();
   }
}
