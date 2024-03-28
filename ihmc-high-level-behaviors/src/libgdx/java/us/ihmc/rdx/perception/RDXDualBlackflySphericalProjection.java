package us.ihmc.rdx.perception;

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
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.avatar.colorVision.stereo.DualBlackflyUDPReceiver;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ImageDimensions;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class RDXDualBlackflySphericalProjection
{
   private final RDXBaseUI baseUI;
   private final SideDependentList<RDXProjectionSphere> projectionSpheres = new SideDependentList<>();
   private final ImDouble pupillaryDistance = new ImDouble(-0.040650);
   private final FramePose3D leftEyePose = new FramePose3D();
   private final FramePose3D rightEyePose = new FramePose3D();
   private final ReferenceFrame robotZUpFrame;
   private final SideDependentList<ReferenceFrame> projectionOriginFrames = new SideDependentList<>();
   private final DualBlackflyUDPReceiver dualBlackflyUDPReceiver = new DualBlackflyUDPReceiver();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private volatile boolean reconnecting = false;
   private Thread reconnectThread;

   private final ImDouble projectionZOffset = new ImDouble(1.4);
   private final ImDouble projectionXOffset = new ImDouble(1.0);
   private final ImDouble projectionYaw = new ImDouble(0.0);
   private final ImDouble projectionPitch = new ImDouble(0.0);
   private final ImDouble projectionRoll = new ImDouble(0.0);
   private final SideDependentList<RDXReferenceFrameGraphic> eyeFrameGraphics = new SideDependentList<>();

   private long lastFrameUpdateTime;

   private YoRegistry stereoVisionRegistry = new YoRegistry("stereoVision");

   private YoDouble yoPupilaryDistance;
   private YoDouble yoProjectionZOffset;
   private YoDouble yoProjectionXOffset;
   private YoDouble yoProjectionYaw;
   private YoDouble yoProjectionPitch;
   private YoDouble yoProjectionRoll;

   private YoDouble yoSphereRadius;
   private YoDouble yoFocalLengthX;
   private YoDouble yoFocalLengthY;
   private YoDouble yoPrinciplePointX;
   private YoDouble yoPrinciplePointY;

   private final YoVariableServer yoVariableServer;

   public RDXDualBlackflySphericalProjection(ReferenceFrame robotZUpFrame)
   {
      this.baseUI = RDXBaseUI.getInstance();
      this.robotZUpFrame = robotZUpFrame;

      yoVariableServer = new YoVariableServer(RDXDualBlackflySphericalProjection.class, null, new DataServerSettings(false, true), 1.0/120.0);
      yoVariableServer.setMainRegistry(stereoVisionRegistry, null);

      yoPupilaryDistance = new YoDouble("pupilaryDistance", stereoVisionRegistry);
      yoPupilaryDistance.set(pupillaryDistance.get());
      yoProjectionZOffset = new YoDouble("projectionZOffset", stereoVisionRegistry);
      yoProjectionZOffset.set(projectionZOffset.get());
      yoProjectionXOffset = new YoDouble("projectionXOffset", stereoVisionRegistry);
      yoProjectionXOffset.set(projectionXOffset.get());
      yoProjectionYaw = new YoDouble("projectionYaw", stereoVisionRegistry);
      yoProjectionPitch = new YoDouble("projectionPitch", stereoVisionRegistry);
      yoProjectionRoll = new YoDouble("projectionRoll", stereoVisionRegistry);

      yoSphereRadius = new YoDouble("sphereRadius", stereoVisionRegistry);
      yoSphereRadius.set(1.0);
      yoFocalLengthX = new YoDouble("focalLengthX", stereoVisionRegistry);
      yoFocalLengthX.set(0.665244);
      yoFocalLengthY = new YoDouble("focalLengthY", stereoVisionRegistry);
      yoFocalLengthY.set(0.713780);
      yoPrinciplePointX = new YoDouble("principlePointX", stereoVisionRegistry);
      yoPrinciplePointX.set(0.0);
      yoPrinciplePointY = new YoDouble("principlePointY", stereoVisionRegistry);
      yoPrinciplePointY.set(0.5);

      // Dex preferred params:
      yoPupilaryDistance.set(-0.002);
      yoProjectionZOffset.set(1.764);
      yoProjectionXOffset.set(-0.433);
      yoProjectionYaw.set(-0.03070866141732287);
      yoProjectionPitch.set(-0.015748031496062964);
      yoProjectionRoll.set(0.017);
      yoSphereRadius.set(5.7);
      yoFocalLengthX.set(0.386);
      yoFocalLengthY.set(0.622);
      yoPrinciplePointX.set(0.005);
      yoPrinciplePointY.set(0.437);

      projectionSpheres.put(RobotSide.LEFT, new RDXProjectionSphere());
      projectionSpheres.put(RobotSide.RIGHT, new RDXProjectionSphere());

      yoVariableServer.start();
      Executors.newSingleThreadScheduledExecutor().scheduleAtFixedRate(() -> yoVariableServer.update(System.nanoTime()), 0, 10, TimeUnit.MILLISECONDS);
   }

   public void renderControls()
   {
      ImGuiTools.sliderDouble("Projection Z offset", projectionZOffset, -4, 4);
      ImGuiTools.sliderDouble("Projection X offset", projectionXOffset, -4, 4);
      ImGuiTools.sliderDouble(labels.get("Pupillary distance"), pupillaryDistance, -0.5, 0.5);
      ImGui.separator();
      projectionSpheres.get(RobotSide.LEFT).renderImGuiWidgets();
      projectionSpheres.get(RobotSide.RIGHT).renderImGuiWidgets();

      // Sync (or mirror settings)
      pupillaryDistance.set(yoPupilaryDistance.getValue());
      projectionZOffset.set(yoProjectionZOffset.getValue());
      projectionXOffset.set(yoProjectionXOffset.getValue());
      projectionYaw.set(yoProjectionYaw.getValue());
      projectionPitch.set(yoProjectionPitch.getValue());
      projectionRoll.set(yoProjectionRoll.getValue());

      for (RobotSide side : RobotSide.values)
      {
         projectionSpheres.get(side).setRadius(yoSphereRadius.getValue());

         projectionSpheres.get(side).setFocalLengthX(yoFocalLengthX.getValue());
         projectionSpheres.get(side).setFocalLengthY(yoFocalLengthY.getValue());
      }

      // mirror principle point
      projectionSpheres.get(RobotSide.LEFT).setPrinciplePointX(yoPrinciplePointX.getValue());
      projectionSpheres.get(RobotSide.LEFT).setPrinciplePointY(yoPrinciplePointY.getValue());
      double principlePointX = yoPrinciplePointX.getValue();
      double principlePointY = yoPrinciplePointY.getValue();
      projectionSpheres.get(RobotSide.RIGHT).setPrinciplePointX(-principlePointX);
      projectionSpheres.get(RobotSide.RIGHT).setPrinciplePointY(principlePointY);
   }

   public boolean isConnectingOrConnected()
   {
      return isReconnecting() || isConnected();
   }

   public boolean isReconnecting()
   {
      return reconnecting;
   }

   public boolean isConnected()
   {
      return dualBlackflyUDPReceiver.connected();
   }

   private void startReconnectThread()
   {
      stopReconnectThread();

      reconnecting = true;

      reconnectThread = new Thread(() ->
      {
         do
         {
            RDXBaseUI.pushNotification("Dual Blackfly stereo client reconnecting...");

            dualBlackflyUDPReceiver.stop();
            dualBlackflyUDPReceiver.start();

            ThreadTools.sleep(5000);
         }
         while (reconnecting && !dualBlackflyUDPReceiver.connected());
      }, getClass().getName() + "-ReconnectThread");

      reconnectThread.start();
   }

   private void stopReconnectThread()
   {
      if (reconnectThread != null)
      {
         reconnecting = false;

         try
         {
            reconnectThread.join();
         }
         catch (InterruptedException e)
         {
            LogTools.error(e);
         }
      }
   }

   public void enable()
   {
      startReconnectThread();

      projectionSpheres.get(RobotSide.LEFT).create();
      projectionSpheres.get(RobotSide.RIGHT).create();
   }

   public void disable()
   {
      // Disable on a thread, so we don't hang the UI
      ThreadTools.startAThread(this::stopReconnectThread, getClass().getSimpleName() + "StopReconnect");

      dualBlackflyUDPReceiver.stop();
   }

   public void shutdown()
   {
      stopReconnectThread();

      dualBlackflyUDPReceiver.stop();
   }

   public void render()
   {
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

               // Draw a circle in the center
               int centerX = mat.cols() / 2;
               int centerY = mat.rows() / 2;
               int radius = 16;
               opencv_imgproc.circle(rgba8Mat, new Point(centerX, centerY), radius, new Scalar(0, 255, 0, 255), opencv_imgproc.CV_FILLED, 8, 0);

               Texture texture = new Texture(new PixmapTextureData(pixmap, null, false, false));

               projectionSpheres.get(side).updateTexture(texture, 0.5f);

               rgba8Mat.close();
               pixmap.dispose();
               mat.close();
               imageDataPointer.close();

               hadNewFrame = true;
            }
         }
      }

      if (baseUI.getVRManager().isVRReady() && hadNewFrame)
      {
         for (RDXReferenceFrameGraphic eyeFrameGraphic : eyeFrameGraphics)
            if (eyeFrameGraphic != null)
               eyeFrameGraphic.updateFromLastGivenFrame();

         for (RobotSide side : RobotSide.values)
         {
            ReferenceFrame projectionOriginFrame = projectionOriginFrames.get(side);

            if (projectionOriginFrame == null)
            {
               projectionOriginFrame = new ReferenceFrame("sphericalProjectionOrigin" + side.getCamelCaseNameForMiddleOfExpression(),
                                                          ReferenceFrame.getWorldFrame())
               {
                  @Override
                  protected void updateTransformToParent(RigidBodyTransform transformToParent)
                  {
                     ReferenceFrame eyeFrame = baseUI.getVRManager().getContext().getEyes().get(side).getEyeXForwardZUpFrame();

                     transformToParent.getTranslation().set(eyeFrame.getTransformToRoot().getTranslation());
                     transformToParent.getRotation().setToYawOrientation(robotZUpFrame.getTransformToRoot().getRotation().getYaw());
                     transformToParent.getTranslation().setZ(projectionZOffset.get());
                     transformToParent.getTranslation().setX(projectionXOffset.get());
                     transformToParent.getRotation().appendYawRotation(side.negateIfRightSide(projectionYaw.get()));
                     transformToParent.getRotation().appendPitchRotation(side.negateIfRightSide(projectionPitch.get()));
                     transformToParent.getRotation().appendRollRotation(side.negateIfRightSide(projectionRoll.get()));
                  }
               };

               projectionOriginFrames.set(side, projectionOriginFrame);
            }

            projectionOriginFrame.update();
         }

         leftEyePose.setToZero(projectionOriginFrames.get(RobotSide.LEFT));
         rightEyePose.setToZero(projectionOriginFrames.get(RobotSide.RIGHT));

         leftEyePose.getTranslation().setY(pupillaryDistance.get() / 2);
         rightEyePose.getTranslation().setY(-pupillaryDistance.get() / 2);

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

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
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
                  RDXReferenceFrameGraphic eyeFrameGraphic = new RDXReferenceFrameGraphic(0.8);
                  eyeFrameGraphic.setToReferenceFrame(baseUI.getVRManager().getContext().getEyes().get(side).getEyeXForwardZUpFrame());
                  eyeFrameGraphics.put(side, eyeFrameGraphic);
               }
            }
         }
      }
   }

   public DualBlackflyUDPReceiver getDualBlackflyUDPReceiver()
   {
      return dualBlackflyUDPReceiver;
   }

   public SideDependentList<RDXProjectionSphere> getProjectionSpheres()
   {
      return projectionSpheres;
   }
}
