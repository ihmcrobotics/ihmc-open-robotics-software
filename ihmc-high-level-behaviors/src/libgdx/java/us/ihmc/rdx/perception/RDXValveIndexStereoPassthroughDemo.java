package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
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
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Queue;
import java.util.Set;

/**
 * A self contained demo that allows experimentation with doing stereo passthrough with
 * the Valve Index HMD cameras.
 */
public class RDXValveIndexStereoPassthroughDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ValveIndexCameraReaderThread cameraReaderThread = new ValveIndexCameraReaderThread();
   private final SideDependentList<RDXProjectionSphere> projectionSpheres = new SideDependentList<>(RDXProjectionSphere::new);
   private final SideDependentList<RDXReferenceFrameGraphic> eyeFrameGraphics = new SideDependentList<>();
   private boolean renderLeftHandPanel;
   private RDX3DSituatedImGuiPanel leftHandPanel;
   private final FramePose3D leftHandPanelPose = new FramePose3D();

   private final FramePose3D leftEyePose = new FramePose3D();
   private final FramePose3D rightEyePose = new FramePose3D();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showEyeFrameGraphics = new ImBoolean(false);
   private final ImDouble sphereRadius = new ImDouble(20.0);
   private final ImDouble projectionScale = new ImDouble(0.45);
   private final ImDouble principlePointX = new ImDouble(0.0);
   private final ImDouble principlePointY = new ImDouble(0.5);

   public RDXValveIndexStereoPassthroughDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            cameraReaderThread.start();

            for (RobotSide side : RobotSide.values)
               projectionSpheres.get(side).create();
            updateProjectionSettings();
            for (RobotSide side : RobotSide.values)
               projectionSpheres.get(side).rebuildUVSphereMesh();
            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Projection controls", this::renderControls);

            leftHandPanel = new RDX3DSituatedImGuiPanel("Projection controls", this::renderControls);
            leftHandPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
            leftHandPanel.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));
            baseUI.getVRManager().getContext().addVRPickCalculator(leftHandPanel::calculateVRPick);
            baseUI.getVRManager().getContext().addVRInputProcessor(leftHandPanel::processVRInput);
            baseUI.getVRManager().getContext().addVRInputProcessor(this::processVRInput);
         }

         @Override
         public void render()
         {
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

            leftHandPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderControls()
         {
            ImGui.checkbox(labels.get("Show eye frame graphics"), showEyeFrameGraphics);

            boolean rebuildMesh = false;
            rebuildMesh |= ImGuiTools.volatileInputDouble(labels.get("Sphere radius"), sphereRadius);
            rebuildMesh |= ImGuiTools.sliderDouble(labels.get("Projection scale"), projectionScale, 0.01, 2.0);
            rebuildMesh |= ImGuiTools.volatileInputDouble(labels.get("Principle point X (Cx)"), principlePointX);
            rebuildMesh |= ImGuiTools.volatileInputDouble(labels.get("Principle point Y (Cy)"), principlePointY);

            updateProjectionSettings();

            if (rebuildMesh)
            {
               for (RobotSide side : RobotSide.values)
               {
                  projectionSpheres.get(side).rebuildUVSphereMesh();
               }
            }
         }

         private void updateProjectionSettings()
         {
            for (RobotSide side : RobotSide.values)
            {
               projectionSpheres.get(side).getSphereRadius().set(sphereRadius);
               projectionSpheres.get(side).getFocalLengthX().set(projectionScale);
               projectionSpheres.get(side).getFocalLengthY().set(projectionScale);
               projectionSpheres.get(side).getPrinciplePointX().set(principlePointX);
               projectionSpheres.get(side).getPrinciplePointY().set(principlePointY);
            }
         }

         public void processVRInput(RDXVRContext vrContext)
         {
            renderLeftHandPanel = vrContext.getHeadset().isConnected() && vrContext.getController(RobotSide.LEFT).isConnected();

            vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
            {
               leftHandPanelPose.setToZero(controller.getXForwardZUpControllerFrame());
               leftHandPanelPose.getOrientation().setYawPitchRoll(Math.PI / 2.0, 0.0, Math.PI / 4.0);
               leftHandPanelPose.getPosition().addY(-0.05);
               leftHandPanelPose.changeFrame(ReferenceFrame.getWorldFrame());
               leftHandPanel.updateDesiredPose(leftHandPanelPose::get);
            });
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
         {
            if (renderLeftHandPanel)
            {
               leftHandPanel.getRenderables(renderables, pool);
            }

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
            cameraReaderThread.stopRunning();

            try
            {
               cameraReaderThread.join();
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }

            leftHandPanel.dispose();

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
