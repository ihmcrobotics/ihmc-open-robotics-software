package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Queue;
import java.util.Set;

public class RDXValveIndexStereoPassthroughDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ValveIndexCameraReaderThread cameraReaderThread = new ValveIndexCameraReaderThread();
   private final SideDependentList<RDXProjectionSphere> projectionSpheres = new SideDependentList<>(RDXProjectionSphere::new);
   private final SideDependentList<RDXReferenceFrameGraphic> eyeFrameGraphics = new SideDependentList<>();

   private final FramePose3D leftEyePose = new FramePose3D();
   private final FramePose3D rightEyePose = new FramePose3D();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showEyeFrameGraphics = new ImBoolean(false);
   private final ImDouble sphereRadius = new ImDouble(20.0);
   private final ImDouble projectionScaleX = new ImDouble(1.0);
   private final ImDouble projectionScaleY = new ImDouble(1.0);
   private final ImDouble principlePointX = new ImDouble(0.0);
   private final ImDouble principlePointY = new ImDouble(0.5);

   public RDXValveIndexStereoPassthroughDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            super.create();

            cameraReaderThread.start();

            // Setup projection spheres
            projectionSpheres.get(RobotSide.LEFT).create();
            projectionSpheres.get(RobotSide.RIGHT).create();
            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Projection controls", this::renderControls);

            baseUI.create();
         }

         @Override
         public void render()
         {
            super.render();

            boolean hadNewFrame = false;

            for (RobotSide side : RobotSide.values)
            {
               Queue<Mat> eyeImageQueue = cameraReaderThread.getEyeImageQueue(side);

               Mat eyeImage = null;
               while (!eyeImageQueue.isEmpty())
               {
                  eyeImage = eyeImageQueue.remove();

                  if (!eyeImageQueue.isEmpty())
                  {
                     eyeImage.close();
                  }
               }

               if (eyeImage != null && eyeImage.address() != 0)
               {
                  Pixmap pixmap = new Pixmap(eyeImage.cols(), eyeImage.rows(), Pixmap.Format.RGBA8888);
                  BytePointer rgba8888BytePointer = new BytePointer(pixmap.getPixels());
                  Mat displayMat = new Mat(eyeImage.rows(), eyeImage.cols(), opencv_core.CV_8UC4, rgba8888BytePointer);
                  eyeImage.copyTo(displayMat);
                  Texture texture = new Texture(new PixmapTextureData(pixmap, null, false, false));

                  projectionSpheres.get(side).updateTexture(texture);

                  displayMat.close();
                  pixmap.dispose();
                  eyeImage.close();

                  hadNewFrame = true;
               }
            }

            // Sync (or mirror settings)
            double principlePointX = projectionSpheres.get(RobotSide.LEFT).getPrinciplePointX().get();
            double principlePointY = projectionSpheres.get(RobotSide.LEFT).getPrinciplePointY().get();
            projectionSpheres.get(RobotSide.RIGHT).getPrinciplePointX().set(-principlePointX);
            projectionSpheres.get(RobotSide.RIGHT).getPrinciplePointY().set(principlePointY);

            if (baseUI.getVRManager().isVRReady() && hadNewFrame)
            {
               for (RDXReferenceFrameGraphic eyeFrameGraphic : eyeFrameGraphics)
                  if (eyeFrameGraphic != null)
                     eyeFrameGraphic.updateFromLastGivenFrame();

               leftEyePose.setToZero(baseUI.getVRManager().getContext().getEyes().get(RobotSide.LEFT).getEyeXForwardZUpFrame());
               rightEyePose.setToZero(baseUI.getVRManager().getContext().getEyes().get(RobotSide.RIGHT).getEyeXForwardZUpFrame());

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

         private void renderControls()
         {
            ImGui.checkbox(labels.get("Show eye frame graphics"), showEyeFrameGraphics);

            boolean rebuildMesh = false;
            rebuildMesh |= ImGuiTools.volatileInputDouble(labels.get("Sphere radius"), sphereRadius);
            rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Projection scale X (Fx)"), projectionScaleX, 0.01, 2.0);
            rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Projection scale Y (Fy)"), projectionScaleY, 0.01, 2.0);
            rebuildMesh |= ImGuiTools.volatileInputDouble(labels.get("Principle point X (Cx)"), principlePointX);
            rebuildMesh |= ImGuiTools.volatileInputDouble(labels.get("Principle point Y (Cy)"), principlePointY);

            for (RobotSide side : RobotSide.values)
            {
               projectionSpheres.get(side).getSphereRadius().set(sphereRadius);
               projectionSpheres.get(side).getFocalLengthX().set(projectionScaleX);
               projectionSpheres.get(side).getFocalLengthY().set(projectionScaleY);
               projectionSpheres.get(side).getPrinciplePointX().set(principlePointX);
               projectionSpheres.get(side).getPrinciplePointY().set(principlePointY);
            }

            if (rebuildMesh)
            {
               for (RobotSide side : RobotSide.values)
               {
                  projectionSpheres.get(side).rebuildUVSphereMesh();
               }
            }
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
         {
            if (sceneLevels.contains(RDXSceneLevel.VR_EYE_RIGHT))
            {
               projectionSpheres.get(RobotSide.RIGHT).getRenderables(renderables, pool);
               if (showEyeFrameGraphics.get() && eyeFrameGraphics.get(RobotSide.RIGHT) != null)
               {
                  eyeFrameGraphics.get(RobotSide.RIGHT).getRenderables(renderables, pool);
               }
            }
            else if (sceneLevels.contains(RDXSceneLevel.VR_EYE_LEFT))
            {
               projectionSpheres.get(RobotSide.LEFT).getRenderables(renderables, pool);
               if (showEyeFrameGraphics.get() && eyeFrameGraphics.get(RobotSide.LEFT) != null)
               {
                  eyeFrameGraphics.get(RobotSide.LEFT).getRenderables(renderables, pool);
               }
            }
            else
            {
               projectionSpheres.get(RobotSide.RIGHT).getRenderables(renderables, pool);
               projectionSpheres.get(RobotSide.LEFT).getRenderables(renderables, pool);

               if (showEyeFrameGraphics.get() && baseUI.getVRManager().isVRReady())
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

         @Override
         public void dispose()
         {
            super.dispose();

            cameraReaderThread.stopRunning();

            try
            {
               cameraReaderThread.join();
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }

            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      Mat mat = new Mat(); // Get opencv loaded before things start
      mat.close();

      new RDXValveIndexStereoPassthroughDemo();
   }
}