package us.ihmc.rdx.simulation;

import imgui.ImGui;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2SimulationSession;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.examples.simulations.bullet.*;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletFlyingBallSimulationTest;

public class RDXSCS2BulletSimulationDemos
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXSCS2SimulationSession scs2SimulationSession;
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

            scs2SimulationSession = new RDXSCS2SimulationSession(baseUI);
            scs2SimulationSession.startSession(FallingBoxBulletSimulation.createSession());
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
            if (ImGui.button(labels.get("Box")))
            {
               scs2SimulationSession.startSession(BoxExperimentalBulletSimulation.createSession());
            }
            if (ImGui.button(labels.get("Box teetering edge to edge")))
            {
               scs2SimulationSession.startSession(BoxTeeteringEdgeToEdgeExperimentalBulletSimulation.createSession());
            }
            if (ImGui.button(labels.get("Colliding spheres no gravity")))
            {
               scs2SimulationSession.startSession(CollidingSpheresNoGravityExperimentalBulletSimulation.createSession());
            }
            if (ImGui.button(labels.get("Connected shapes")))
            {
               scs2SimulationSession.startSession(ConnectedShapesExperimentalBulletSimulation.createSession());
            }
            if (ImGui.button(labels.get("Falling box")))
            {
               scs2SimulationSession.startSession(FallingBoxBulletSimulation.createSession());
            }
            if (ImGui.button(labels.get("Falling sphere")))
            {
               scs2SimulationSession.startSession(FallingSphereExperimentalBulletSimulation.createSession());
            }
            if (ImGui.button(labels.get("Flying ball")))
            {
               scs2SimulationSession.startSession(new BulletFlyingBallSimulationTest().createSession());
            }
            if (ImGui.button(labels.get("Mobile")))
            {
               scs2SimulationSession.startSession(MobileBulletSimulation.createSession());
            }
            if (ImGui.button(labels.get("Newton's cradle")))
            {
               scs2SimulationSession.startSession(NewtonsCradleExperimentalBulletSimulation.createSession());
            }
            if (ImGui.button(labels.get("Prismatic spring")))
            {
               scs2SimulationSession.startSession(PrismaticSpringBulletSimulation.createSession());
            }
            if (ImGui.button(labels.get("Spring box with intertia and collision offsets")))
            {
               scs2SimulationSession.startSession(SingleBoxWithInertiaAndCollisionOffsetsBulletSimulation.createSession());
            }
            if (ImGui.button(labels.get("Sliding box")))
            {
               scs2SimulationSession.startSession(SlidingBoxExperimentalBulletSimulation.createSession());
            }
            if (ImGui.button(labels.get("Stack of boxes")))
            {
               scs2SimulationSession.startSession(StackOfBoxesExperimentalBulletSimulation.createSession());
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
