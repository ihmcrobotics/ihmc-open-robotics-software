package us.ihmc.gdx.ui.gizmo;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.affordances.GDXInteractableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class GDXFrameGizmoDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   public GDXFrameGizmoDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.get3DSceneManager().addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));

            RigidBodyTransform transform = new RigidBodyTransform();
            ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), transform);
            transform.getTranslation().addX(0.5);
            GDXInteractableReferenceFrame interactableReferenceFrame = new GDXInteractableReferenceFrame();
            interactableReferenceFrame.create(referenceFrame, transform, 1.0, baseUI.get3DSceneManager().getCamera3D());
            baseUI.addImGui3DViewPickCalculator(interactableReferenceFrame::calculate3DViewPick);
            baseUI.addImGui3DViewInputProcessor(interactableReferenceFrame::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(interactableReferenceFrame::getVirtualRenderables);

            RigidBodyTransform transform2 = new RigidBodyTransform();
            transform2.getTranslation().add(0.5, 0.5, 0.5);
            ReferenceFrame referenceFrame2 = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(referenceFrame, transform2);
            GDXInteractableReferenceFrame interactableReferenceFrame2 = new GDXInteractableReferenceFrame();
            interactableReferenceFrame2.create(referenceFrame2, transform2, 1.0, baseUI.get3DSceneManager().getCamera3D());
            baseUI.addImGui3DViewPickCalculator(interactableReferenceFrame2::calculate3DViewPick);
            baseUI.addImGui3DViewInputProcessor(interactableReferenceFrame2::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(interactableReferenceFrame2::getVirtualRenderables);

            GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo(interactableReferenceFrame2.getSelectablePose3DGizmo().getPoseGizmo().getGizmoFrame());
            poseGizmo.create(baseUI.get3DSceneManager().getCamera3D());
            baseUI.addImGui3DViewPickCalculator(poseGizmo::calculate3DViewPick);
            baseUI.addImGui3DViewInputProcessor(poseGizmo::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(poseGizmo);
            baseUI.getImGuiPanelManager().addPanel(poseGizmo.createTunerPanel(GDXFrameGizmoDemo.class.getSimpleName()));
            poseGizmo.getTransformToParent().getTranslation().add(0.5, 0.5, 0.5);

            GDXPathControlRingGizmo footstepRingGizmo
                  = new GDXPathControlRingGizmo(interactableReferenceFrame2.getSelectablePose3DGizmo().getPoseGizmo().getGizmoFrame());
            footstepRingGizmo.create(baseUI.get3DSceneManager().getCamera3D());
            baseUI.addImGui3DViewPickCalculator(footstepRingGizmo::calculate3DViewPick);
            baseUI.addImGui3DViewInputProcessor(footstepRingGizmo::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(footstepRingGizmo);
            baseUI.getImGuiPanelManager().addPanel(footstepRingGizmo.createTunerPanel(GDXFrameGizmoDemo.class.getSimpleName()));
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
      new GDXFrameGizmoDemo();
   }
}
