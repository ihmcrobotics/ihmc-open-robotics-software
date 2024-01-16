package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class RDXFrameGizmoDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();

   public RDXFrameGizmoDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));

            RigidBodyTransform transform = new RigidBodyTransform();
            ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), transform);
            transform.getTranslation().addX(0.5);
            RDXInteractableReferenceFrame interactableReferenceFrame = new RDXInteractableReferenceFrame();
            interactableReferenceFrame.create(referenceFrame, transform, 1.0, baseUI.getPrimary3DPanel());
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(interactableReferenceFrame::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableReferenceFrame::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(interactableReferenceFrame::getVirtualRenderables);

            RigidBodyTransform transform2 = new RigidBodyTransform();
            transform2.getTranslation().add(0.5, 0.5, 0.5);
            ReferenceFrame referenceFrame2 = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(referenceFrame, transform2);
            RDXInteractableReferenceFrame interactableReferenceFrame2 = new RDXInteractableReferenceFrame();
            interactableReferenceFrame2.create(referenceFrame2, transform2, 1.0, baseUI.getPrimary3DPanel());
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(interactableReferenceFrame2::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableReferenceFrame2::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(interactableReferenceFrame2::getVirtualRenderables);

            RDXPose3DGizmo poseGizmo = new RDXPose3DGizmo(interactableReferenceFrame2.getSelectablePose3DGizmo().getPoseGizmo().getGizmoFrame());
            poseGizmo.create(baseUI.getPrimary3DPanel());
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(poseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(poseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(poseGizmo);
            baseUI.getImGuiPanelManager().addPanel(poseGizmo.createTunerPanel(RDXFrameGizmoDemo.class.getSimpleName()));
            poseGizmo.getTransformToParent().getTranslation().add(0.5, 0.5, 0.5);

            RDXPathControlRingGizmo footstepRingGizmo
                  = new RDXPathControlRingGizmo(interactableReferenceFrame2.getSelectablePose3DGizmo().getPoseGizmo().getGizmoFrame());
            footstepRingGizmo.createAndSetupDefault(baseUI);
            baseUI.getImGuiPanelManager().addPanel(footstepRingGizmo.createTunerPanel(RDXFrameGizmoDemo.class.getSimpleName()));
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
      new RDXFrameGizmoDemo();
   }
}
