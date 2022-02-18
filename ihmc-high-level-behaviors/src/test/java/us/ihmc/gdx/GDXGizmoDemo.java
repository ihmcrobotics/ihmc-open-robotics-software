package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXFootstepPlannerGoalGizmo;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

public class GDXGizmoDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();
   private final GDXFootstepPlannerGoalGizmo footstepRingGizmo = new GDXFootstepPlannerGoalGizmo();

   public GDXGizmoDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.get3DSceneManager().addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));

            poseGizmo.create(baseUI.get3DSceneManager().getCamera3D());
            baseUI.addImGui3DViewInputProcessor(poseGizmo::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(poseGizmo);
            baseUI.getImGuiPanelManager().addPanel(poseGizmo.createTunerPanel(GDXGizmoDemo.class.getSimpleName()));

            footstepRingGizmo.create(baseUI.get3DSceneManager().getCamera3D());
            baseUI.addImGui3DViewInputProcessor(footstepRingGizmo::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(footstepRingGizmo);
            baseUI.getImGuiPanelManager().addPanel(footstepRingGizmo.createTunerPanel(GDXGizmoDemo.class.getSimpleName()));
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
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXGizmoDemo();
   }
}
