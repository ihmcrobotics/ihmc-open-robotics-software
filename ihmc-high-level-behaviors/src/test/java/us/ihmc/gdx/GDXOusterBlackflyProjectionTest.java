package us.ihmc.gdx;

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
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.ui.graphics.GDXReferenceFrameGraphic;

public class GDXOusterBlackflyProjectionTest
{

   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private final GDXPose3DGizmo ousterPointInWorldGizmo = new GDXPose3DGizmo(ReferenceFrame.getWorldFrame());
   private RigidBodyTransform userTransformToParent;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showWorldFrame = new ImBoolean(true);
   private final ImBoolean showUserFrame = new ImBoolean(true);
   private final ImDouble yaw = new ImDouble();
   private final ImDouble pitch = new ImDouble();
   private final ImDouble roll = new ImDouble();
   private final ImDouble focalLength = new ImDouble(0.0027);
   private GDXReferenceFrameGraphic userReferenceFrameGraphic;
   private final FramePoint3D ousterFramePoint = new FramePoint3D();
   private final FramePoint3D pointOnCMOSPlane = new FramePoint3D();
   private ReferenceFrame userReferenceFrame;
   private ModelInstance worldFrameGraphic;
   private ModelInstance pointOnCMOSGraphic;
   private RigidBodyTransform tempTransform = new RigidBodyTransform();

   public GDXOusterBlackflyProjectionTest()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            worldFrameGraphic = new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3));
            GDXTools.setTransparency(worldFrameGraphic, 0.6f);

            userTransformToParent = new RigidBodyTransform();
            yaw.set(Math.toRadians(-90.0));
            pitch.set(Math.toRadians(0.0));
            roll.set(Math.toRadians(-90.0));
            userTransformToParent.getRotation().setYawPitchRoll(yaw.get(), pitch.get(), roll.get());

            userReferenceFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("userReferenceFrame",
                                                                                                 ReferenceFrame.getWorldFrame(), userTransformToParent);
            userReferenceFrameGraphic = new GDXReferenceFrameGraphic(0.2);
            userReferenceFrameGraphic.setToReferenceFrame(userReferenceFrame);

            ousterPointInWorldGizmo.create(baseUI.get3DSceneManager().getCamera3D());
            baseUI.addImGui3DViewPickCalculator(ousterPointInWorldGizmo::calculate3DViewPick);
            baseUI.addImGui3DViewInputProcessor(ousterPointInWorldGizmo::process3DViewInput);
            ousterPointInWorldGizmo.setResizeAutomatically(false);

            pointOnCMOSGraphic = GDXModelPrimitives.createSphere(0.01f, Color.RED);

            baseUI.get3DSceneManager().addRenderableProvider(this::getRenderables);
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
            GDXTools.toGDX(pointOnCMOSPlane, pointOnCMOSGraphic.transform);

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
      new GDXOusterBlackflyProjectionTest();
   }
}
