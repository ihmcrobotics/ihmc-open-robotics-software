package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

public class RDXOusterBlackflyProjectionTest
{

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXPose3DGizmo ousterPointInWorldGizmo = new RDXPose3DGizmo(ReferenceFrame.getWorldFrame());
   private RigidBodyTransform userTransformToParent;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showWorldFrame = new ImBoolean(true);
   private final ImBoolean showUserFrame = new ImBoolean(true);
   private final ImDouble yaw = new ImDouble();
   private final ImDouble pitch = new ImDouble();
   private final ImDouble roll = new ImDouble();
   private final ImDouble focalLength = new ImDouble(0.0027);
   private RDXReferenceFrameGraphic userReferenceFrameGraphic;
   private final FramePoint3D ousterFramePoint = new FramePoint3D();
   private final FramePoint3D pointOnCMOSPlane = new FramePoint3D();
   private ReferenceFrame userReferenceFrame;
   private ModelInstance worldFrameGraphic;
   private ModelInstance pointOnCMOSGraphic;
   private RigidBodyTransform tempTransform = new RigidBodyTransform();

   public RDXOusterBlackflyProjectionTest()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            LibGDXTools.setOpacity(worldFrameGraphic, 0.6f);

            userTransformToParent = new RigidBodyTransform();
            yaw.set(Math.toRadians(-90.0));
            pitch.set(Math.toRadians(0.0));
            roll.set(Math.toRadians(-90.0));
            userTransformToParent.getRotation().setYawPitchRoll(yaw.get(), pitch.get(), roll.get());

            userReferenceFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("userReferenceFrame",
                                                                                                 ReferenceFrame.getWorldFrame(), userTransformToParent);
            userReferenceFrameGraphic = new RDXReferenceFrameGraphic(0.2);
            userReferenceFrameGraphic.setToReferenceFrame(userReferenceFrame);

            ousterPointInWorldGizmo.create(baseUI.getPrimary3DPanel());
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(ousterPointInWorldGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(ousterPointInWorldGizmo::process3DViewInput);
            ousterPointInWorldGizmo.setResizeAutomatically(false);

            pointOnCMOSGraphic = RDXModelBuilder.createSphere(0.01f, Color.RED);

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
            baseUI.getImGuiPanelManager().addPanel("Reference Frames", this::renderImGuiWidgets);
         }

         private void renderImGuiWidgets()
         {
            ImGui.checkbox(labels.get("Show World Frame"), showWorldFrame);
            ImGui.checkbox(labels.get("Show User Frame"), showUserFrame);
            boolean input = false;
            input |= ImGui.inputDouble(labels.get("Yaw"), yaw);
            input |= ImGui.inputDouble(labels.get("Pitch"), pitch);
            input |= ImGui.inputDouble(labels.get("Roll"), roll);

            ImGui.inputDouble(labels.get("Focal length"), focalLength);

            if (input)
            {
               userTransformToParent.getRotation().setYawPitchRoll(Math.toRadians(yaw.get()), Math.toRadians(pitch.get()), Math.toRadians(roll.get()));
               userReferenceFrame.update();
               userReferenceFrameGraphic.setToReferenceFrame(userReferenceFrame);
            }
         }

         @Override
         public void render()
         {
            ousterFramePoint.setIncludingFrame(ReferenceFrame.getWorldFrame(), ousterPointInWorldGizmo.getPose().getPosition());
//            ousterFramePoint.changeFrame(userReferenceFrame);


            // XY plane
            double yaw2 = EuclidGeometryTools.angleFromFirstToSecondVector2D(1.0, 0.0, ousterFramePoint.getX(), ousterFramePoint.getY());
            // XZ plane
            double pitch2 = EuclidGeometryTools.angleFromFirstToSecondVector2D(1.0, 0.0, ousterFramePoint.getX(), -ousterFramePoint.getZ());

            pointOnCMOSPlane.setIncludingFrame(ReferenceFrame.getWorldFrame(), -focalLength.get(), -focalLength.get() * yaw2, focalLength.get() * pitch2);
//            pointOnCMOSPlane.setIncludingFrame(userReferenceFrame, 0.0, 0.0, -focalLength.get());
//            pointOnCMOSPlane.changeFrame(ReferenceFrame.getWorldFrame());
            LibGDXTools.toLibGDX(pointOnCMOSPlane, pointOnCMOSGraphic.transform);

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            if (showWorldFrame.get())
               worldFrameGraphic.getRenderables(renderables, pool);
            if (showUserFrame.get())
               userReferenceFrameGraphic.getRenderables(renderables, pool);
            ousterPointInWorldGizmo.getRenderables(renderables, pool);
            pointOnCMOSGraphic.getRenderables(renderables, pool);
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
      new RDXOusterBlackflyProjectionTest();
   }
}
