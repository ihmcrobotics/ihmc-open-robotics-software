package us.ihmc.rdx.ui.gizmo;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.rdx.RDXFocusBasedCamera;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

public class RDXGizmoDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();

   private RDXFocusBasedCamera focusBasedCamera;
   private final RDXPose3DGizmo poseGizmo = new RDXPose3DGizmo();
   private final RDXPathControlRingGizmo footstepRingGizmo = new RDXPathControlRingGizmo();
   private ModelInstance clockHandTip;
   private ModelInstance clockCenter;
   private RDXReferenceFrameGraphic keyboardTransformationFrameGraphic;

   public RDXGizmoDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            focusBasedCamera = baseUI.getPrimary3DPanel().getCamera3D();

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));

            clockHandTip = new ModelInstance(RDXModelBuilder.createSphere(0.1f, Color.RED));
            baseUI.getPrimaryScene().addModelInstance(clockHandTip);
            clockCenter = new ModelInstance(RDXModelBuilder.createSphere(0.1f, Color.BLUE));
            baseUI.getPrimaryScene().addModelInstance(clockCenter);

            poseGizmo.create(baseUI.getPrimary3DPanel());
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(poseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(poseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(poseGizmo);
            baseUI.getImGuiPanelManager().addPanel(poseGizmo.createTunerPanel(RDXGizmoDemo.class.getSimpleName()));

            poseGizmo.getTransformToParent().getTranslation().set(-1.0, -2.0, 0.1);

            footstepRingGizmo.create(baseUI.getPrimary3DPanel());
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(footstepRingGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(footstepRingGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(footstepRingGizmo);
            baseUI.getImGuiPanelManager().addPanel(footstepRingGizmo.createTunerPanel(RDXGizmoDemo.class.getSimpleName()));

            footstepRingGizmo.getTransformToParent().getTranslation().set(2.0, 1.0, 0.0);

            keyboardTransformationFrameGraphic = new RDXReferenceFrameGraphic(0.3, Color.SKY);
            baseUI.getPrimaryScene().addRenderableProvider(keyboardTransformationFrameGraphic);
         }

         @Override
         public void render()
         {
            LibGDXTools.toLibGDX(poseGizmo.getClockFaceDragAlgorithm().getClockHandTipInWorld(), clockHandTip.transform);
            LibGDXTools.toLibGDX(poseGizmo.getClockFaceDragAlgorithm().getClockCenter(), clockCenter.transform);

            keyboardTransformationFrameGraphic.getFramePose3D().setToZero();
            keyboardTransformationFrameGraphic.getFramePose3D().getOrientation().setToYawOrientation(focusBasedCamera.getFocusPointPose().getYaw());
            keyboardTransformationFrameGraphic.updateFromFramePose();

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
      new RDXGizmoDemo();
   }
}
