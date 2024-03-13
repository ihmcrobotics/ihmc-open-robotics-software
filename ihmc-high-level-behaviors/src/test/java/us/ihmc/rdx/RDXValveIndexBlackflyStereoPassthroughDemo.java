package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImDouble;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.colorVision.stereo.DualBlackflyUDPReceiver;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.ImageDimensions;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.perception.RDXProjectionSphere;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

public class RDXValveIndexBlackflyStereoPassthroughDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final SideDependentList<RDXProjectionSphere> projectionSpheres = new SideDependentList<>(RDXProjectionSphere::new);
   private final ImDouble pupillaryDistance = new ImDouble(0.0);

   private final FramePose3D leftEyePose = new FramePose3D();
   private final FramePose3D rightEyePose = new FramePose3D();
   private ReferenceFrame projectionOriginFrame;

   private final ImDouble projectionZOffset = new ImDouble(1.701657);
   private final SideDependentList<RDXReferenceFrameGraphic> eyeFrameGraphics = new SideDependentList<>();

   private final DualBlackflyUDPReceiver dualBlackflyUDPReceiver = new DualBlackflyUDPReceiver();

   private long lastFrameUpdateTime;

   public RDXValveIndexBlackflyStereoPassthroughDemo()
   {
      for (RDXProjectionSphere projectionSphere : projectionSpheres)
      {
         projectionSphere.setProjectionScaleX(0.284862);
         projectionSphere.setProjectionScaleY(0.394807);
      }

      projectionSpheres.get(RobotSide.LEFT).setPrinciplePointX(-0.013812);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            super.create();

            dualBlackflyUDPReceiver.start();

            // Setup projection spheres
            projectionSpheres.get(RobotSide.LEFT).create();
            projectionSpheres.get(RobotSide.RIGHT).create();
            baseUI.getPrimaryScene().addRenderableProvider(RDXValveIndexBlackflyStereoPassthroughDemo.this::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Projection controls", RDXValveIndexBlackflyStereoPassthroughDemo.this::renderControls);

            baseUI.create();

            // Setup VR frames after BaseUI is created
            projectionOriginFrame = new ReferenceFrame("sphericalProjectionOrigin", ReferenceFrame.getWorldFrame())
            {
               @Override
               protected void updateTransformToParent(RigidBodyTransform transformToParent)
               {
                  ReferenceFrame headsetFrame = baseUI.getVRManager().getContext().getHeadset().getXForwardZUpHeadsetFrame();

                  transformToParent.getTranslation().set(headsetFrame.getTransformToRoot().getTranslation());
                  transformToParent.getRotation().set(headsetFrame.getTransformToRoot().getRotation());

                  transformToParent.getTranslation().setZ(projectionZOffset.get());
                  //                  transformToParent.getRotation().setToYawOrientation(robotZUpFrame.getTransformToRoot().getRotation().getYaw());
               }
            };
            leftEyePose.setReferenceFrame(projectionOriginFrame);
            rightEyePose.setReferenceFrame(projectionOriginFrame);
         }

         @Override
         public void render()
         {
            super.render();

            boolean hadNewFrame = false;

            if (System.currentTimeMillis() - lastFrameUpdateTime > 20)
            {
               lastFrameUpdateTime = System.currentTimeMillis();

               for (RobotSide side : RobotSide.values)
               {
                  byte[] imageData = dualBlackflyUDPReceiver.getImageBuffers().get(side);

                  if (imageData != null)
                  {
                     ImageDimensions imageDimensions = dualBlackflyUDPReceiver.getImageDimensions().get(side);
                     BytePointer imageDataPointer = new BytePointer(imageData);

                     Mat mat = new Mat(imageDimensions.getImageHeight(), imageDimensions.getImageWidth(), opencv_core.CV_8UC1);
                     mat.data(imageDataPointer);

                     Pixmap pixmap = new Pixmap(mat.cols(), mat.rows(), Pixmap.Format.RGBA8888);
                     BytePointer rgba8888BytePointer = new BytePointer(pixmap.getPixels());
                     Mat rgba8Mat = new Mat(mat.rows(), mat.cols(), opencv_core.CV_8UC4, rgba8888BytePointer);
                     opencv_imgproc.cvtColor(mat, rgba8Mat, opencv_imgproc.COLOR_BayerBG2RGBA);
                     Texture texture = new Texture(new PixmapTextureData(pixmap, null, false, false));

                     projectionSpheres.get(side).updateTexture(texture, 1.0f);

                     rgba8Mat.close();
                     pixmap.dispose();
                     mat.close();
                     imageDataPointer.close();

                     hadNewFrame = true;
                  }
               }
            }

            if (baseUI.getVRManager().isVRReady())
               for (RDXReferenceFrameGraphic eyeFrameGraphic : eyeFrameGraphics)
                  if (eyeFrameGraphic != null)
                     eyeFrameGraphic.updateFromLastGivenFrame();

            // Sync (or mirror settings)
            double principlePointX = projectionSpheres.get(RobotSide.LEFT).getPrinciplePointX();
            double principlePointY = projectionSpheres.get(RobotSide.LEFT).getPrinciplePointY();
            projectionSpheres.get(RobotSide.RIGHT).setPrinciplePointX(-principlePointX);
            projectionSpheres.get(RobotSide.RIGHT).setPrinciplePointY(principlePointY);

            if (hadNewFrame)
            {
               leftEyePose.getTranslation().setY(pupillaryDistance.get() / 2);
               rightEyePose.getTranslation().setY(-pupillaryDistance.get() / 2);

               projectionOriginFrame.update();
               RigidBodyTransform leftEyePoseWorld = new RigidBodyTransform(leftEyePose);
               leftEyePoseWorld.preMultiply(leftEyePose.getReferenceFrame().getTransformToRoot());
               RigidBodyTransform rightEyePoseWorld = new RigidBodyTransform(rightEyePose);
               rightEyePoseWorld.preMultiply(rightEyePose.getReferenceFrame().getTransformToRoot());
               LibGDXTools.toLibGDX(leftEyePoseWorld, projectionSpheres.get(RobotSide.LEFT).getModelInstance().transform);
               LibGDXTools.toLibGDX(rightEyePoseWorld, projectionSpheres.get(RobotSide.RIGHT).getModelInstance().transform);
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            super.dispose();

            dualBlackflyUDPReceiver.stop();

            baseUI.dispose();
         }
      });
   }

   private void renderControls()
   {
      ImGuiTools.sliderDouble("Projection Z offset", projectionZOffset, -4, 4);

      if (ImGuiTools.sliderDouble("Pupillary distance", pupillaryDistance, -2, 2))
      {

      }

      ImGui.separator();

      for (RDXProjectionSphere projectionSphere : projectionSpheres)
      {
         projectionSphere.renderImGuiWidgets();
      }
   }

   private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VR_EYE_RIGHT))
      {
         projectionSpheres.get(RobotSide.RIGHT).getRenderables(renderables, pool);
      }
      else if (sceneLevels.contains(RDXSceneLevel.VR_EYE_LEFT))
      {
         projectionSpheres.get(RobotSide.LEFT).getRenderables(renderables, pool);
      }
      else
      {
         projectionSpheres.get(RobotSide.RIGHT).getRenderables(renderables, pool);
         projectionSpheres.get(RobotSide.LEFT).getRenderables(renderables, pool);

         if (baseUI.getVRManager().isVRReady())
         {
            for (RobotSide side : RobotSide.values)
            {
               if (eyeFrameGraphics.get(side) != null)
               {
                  eyeFrameGraphics.get(side).getRenderables(renderables, pool);
               }
               else
               {
                  RDXReferenceFrameGraphic eyeFrameGraphic = new RDXReferenceFrameGraphic(0.5);
                  eyeFrameGraphic.setToReferenceFrame(baseUI.getVRManager().getContext().getEyes().get(side).getEyeXForwardZUpFrame());
                  eyeFrameGraphics.put(side, eyeFrameGraphic);
               }
            }
         }
      }
   }

   public static void main(String[] args)
   {
      Mat mat = new Mat(); // Get opencv loaded before things start
      mat.close();

      new RDXValveIndexBlackflyStereoPassthroughDemo();
   }
}
