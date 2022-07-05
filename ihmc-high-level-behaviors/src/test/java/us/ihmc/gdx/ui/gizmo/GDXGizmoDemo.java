package us.ihmc.gdx.ui.gizmo;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.GDXReferenceFrameGraphic;

public class GDXGizmoDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   private GDXFocusBasedCamera focusBasedCamera;
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();
   private final GDXPathControlRingGizmo footstepRingGizmo = new GDXPathControlRingGizmo();
   private ModelInstance clockHandTip;
   private ModelInstance clockCenter;
   private GDXReferenceFrameGraphic keyboardTransformationFrameGraphic;

   public GDXGizmoDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            focusBasedCamera = baseUI.getPrimary3DPanel().getCamera3D();

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(GDXModelBuilder.createCoordinateFrame(0.3)));

            clockHandTip = new ModelInstance(GDXModelBuilder.createSphere(0.1f, Color.RED));
            baseUI.getPrimaryScene().addModelInstance(clockHandTip);
            clockCenter = new ModelInstance(GDXModelBuilder.createSphere(0.1f, Color.BLUE));
            baseUI.getPrimaryScene().addModelInstance(clockCenter);

            poseGizmo.create(focusBasedCamera);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(poseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(poseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(poseGizmo);
            baseUI.getImGuiPanelManager().addPanel(poseGizmo.createTunerPanel(GDXGizmoDemo.class.getSimpleName()));

            poseGizmo.getTransformToParent().getTranslation().set(-1.0, -2.0, 0.1);

            footstepRingGizmo.create(focusBasedCamera);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(footstepRingGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(footstepRingGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(footstepRingGizmo);
            baseUI.getImGuiPanelManager().addPanel(footstepRingGizmo.createTunerPanel(GDXGizmoDemo.class.getSimpleName()));

            footstepRingGizmo.getTransformToParent().getTranslation().set(2.0, 1.0, 0.0);

            keyboardTransformationFrameGraphic = new GDXReferenceFrameGraphic(0.3, Color.SKY);
            baseUI.getPrimaryScene().addRenderableProvider(keyboardTransformationFrameGraphic);
         }

         @Override
         public void render()
         {
            GDXTools.toGDX(poseGizmo.getClockFaceDragAlgorithm().getClockHandTipInWorld(), clockHandTip.transform);
            GDXTools.toGDX(poseGizmo.getClockFaceDragAlgorithm().getClockCenter(), clockCenter.transform);

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
      new GDXGizmoDemo();
   }
}
