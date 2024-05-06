package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.colorVision.stereo.DualBlackflyUDPReceiver;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.perception.ImageDimensions;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.perception.RDXProjectionHemisphere.FisheyeTextureCalculator;
import us.ihmc.rdx.perception.RDXProjectionHemisphere.FisheyeTextureCalculator.CameraOrientation;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BiConsumer;

public class RDXDualBlackflyProjection
{
   private enum RDXProjectionType
   {
      SPHERE, FLAT_RECTANGLE, HEMISPHERE;
   }

   private static final RDXProjectionType PROJECTION_TYPE = RDXProjectionType.HEMISPHERE;

   private final RDXBaseUI baseUI;
   private final SideDependentList<RDXProjectionShape> projectionShapes = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> projectionShapePoses = new SideDependentList<>(side -> new RigidBodyTransform());
   private final ReferenceFrame robotZUpFrame;
   private final SideDependentList<ReferenceFrame> robotCameraFrames = new SideDependentList<>();
   private final ReferenceFrame stereoMidPointFrame;

   private final DualBlackflyUDPReceiver dualBlackflyUDPReceiver = new DualBlackflyUDPReceiver();

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private volatile boolean reconnecting = false;
   private Thread reconnectThread;

   private final RDXDualBlackflyProjectionSettings projectionSettings = new RDXDualBlackflyProjectionSettings();

   private final SideDependentList<RDXReferenceFrameGraphic> eyeFrameGraphics = new SideDependentList<>();

   private final SideDependentList<AtomicReference<PixmapTextureData>> newTextureMapDatas = new SideDependentList<>(new AtomicReference<>(null),
                                                                                                                    new AtomicReference<>(null));
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedDaemonThreadFactory(getClass().getSimpleName()));
   private ScheduledFuture<?> textureUpdateFuture;

   private final SideDependentList<RigidBodyTransform> camerasToStereoMidPoint = new SideDependentList<>();

   public RDXDualBlackflyProjection(HumanoidReferenceFrames currentRobotFrames)
   {
      this.baseUI = RDXBaseUI.getInstance();
      this.robotZUpFrame = currentRobotFrames.getMidFootZUpGroundFrame();
      robotCameraFrames.set(currentRobotFrames::getSituationalAwarenessCameraFrame);

      RigidBodyTransform stereoMidPoint = new RigidBodyTransform();
      MovingReferenceFrame chestFrame = currentRobotFrames.getChestFrame();
      stereoMidPoint.interpolate(currentRobotFrames.getSituationalAwarenessCameraFrame(RobotSide.LEFT).getTransformToDesiredFrame(chestFrame),
                                 currentRobotFrames.getSituationalAwarenessCameraFrame(RobotSide.RIGHT).getTransformToDesiredFrame(chestFrame),
                                 0.5);
      stereoMidPointFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("StereoMidPoint", chestFrame, stereoMidPoint);
      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform cameraToStereoMidPoint = currentRobotFrames.getSituationalAwarenessCameraFrame(side).getTransformToDesiredFrame(chestFrame);
         cameraToStereoMidPoint.invert();
         cameraToStereoMidPoint.multiply(stereoMidPoint);
         cameraToStereoMidPoint.invert();
         camerasToStereoMidPoint.set(side, cameraToStereoMidPoint);
      }

      projectionSettings.cameraXOffsetCalibration.set(camerasToStereoMidPoint.get(RobotSide.LEFT).getTranslationX());
      projectionSettings.cameraYOffsetCalibration.set(camerasToStereoMidPoint.get(RobotSide.LEFT).getTranslationY());
      projectionSettings.cameraZOffsetCalibration.set(camerasToStereoMidPoint.get(RobotSide.LEFT).getTranslationZ());
      projectionSettings.cameraYawOffsetCalibration.set(camerasToStereoMidPoint.get(RobotSide.LEFT).getRotation().getYaw());
      projectionSettings.cameraPitchOffsetCalibration.set(camerasToStereoMidPoint.get(RobotSide.LEFT).getRotation().getPitch());
      projectionSettings.cameraRollOffsetCalibration.set(camerasToStereoMidPoint.get(RobotSide.LEFT).getRotation().getRoll());

      RigidBodyTransform diff = new RigidBodyTransform();
      diff.set(camerasToStereoMidPoint.get(RobotSide.LEFT));
      diff.multiply(camerasToStereoMidPoint.get(RobotSide.RIGHT));

      for (RobotSide side : RobotSide.values)
      {
         projectionShapes.put(side, switch (PROJECTION_TYPE)
         {
            case SPHERE -> new RDXProjectionSphere();
            case FLAT_RECTANGLE -> new RDXProjectionRectangle();
            case HEMISPHERE -> new RDXProjectionHemisphere();
         });
      }
   }

   public void renderControls()
   {
      projectionSettings.renderProjectionControls();

      ImGui.separator();
      projectionShapes.get(RobotSide.LEFT).renderImGuiWidgets();

      ImGuiTools.separatorText("Save / Load");
      projectionSettings.renderIOControls(projectionShapes.get(RobotSide.LEFT));

      updateParameters();
   }

   private void updateParameters()
   {
      if (PROJECTION_TYPE == RDXProjectionType.FLAT_RECTANGLE)
      {
         RDXProjectionRectangle leftProjectionRectangle = (RDXProjectionRectangle) projectionShapes.get(RobotSide.LEFT);
         RDXProjectionRectangle rightProjectionRectangle = (RDXProjectionRectangle) projectionShapes.get(RobotSide.RIGHT);

         //            rightProjectionRectangle.setShapeWidth(leftProjectionRectangle.getShapeWidth());
         rightProjectionRectangle.setTextureWidthResolution(leftProjectionRectangle.getNumberOfPixelsWidth());
         //            rightProjectionRectangle.setAspectRatio(leftProjectionRectangle.getAspectRatio());
      }
      else if (PROJECTION_TYPE == RDXProjectionType.HEMISPHERE)
      {
         RDXProjectionHemisphere leftProjectionHemisphere = (RDXProjectionHemisphere) projectionShapes.get(RobotSide.LEFT);
         RDXProjectionHemisphere rightProjectionHemisphere = (RDXProjectionHemisphere) projectionShapes.get(RobotSide.RIGHT);

         rightProjectionHemisphere.setXRadius(leftProjectionHemisphere.getXRadius());
         rightProjectionHemisphere.setYRadius(leftProjectionHemisphere.getYRadius());
         rightProjectionHemisphere.setZRadius(leftProjectionHemisphere.getZRadius());
         rightProjectionHemisphere.setLatitudeResolution(leftProjectionHemisphere.getLatitudeResolution());
         rightProjectionHemisphere.setLongitudeResolution(leftProjectionHemisphere.getLongitudeResolution());
         rightProjectionHemisphere.setFieldOfView(leftProjectionHemisphere.getFieldOfView());
      }

      if (PROJECTION_TYPE == RDXProjectionType.HEMISPHERE)
      {
         for (RobotSide side : RobotSide.values)
         {
            RDXProjectionHemisphere projectionHemisphere = (RDXProjectionHemisphere) projectionShapes.get(side);
            BiConsumer<Point3DReadOnly[], TexCoord2f[]> textureCoordinateCalculator = projectionHemisphere.getTextureCoordinateCalculator();

            if (!(textureCoordinateCalculator instanceof FisheyeTextureCalculator))
            {
//                              BlackflyLensProperties properties = side == RobotSide.LEFT ?
//                                    BlackflyLensProperties.BFLY_U3_23S6C_FE185C086HA_1_LEFT_MSA_CALIBRATED :
//                                    BlackflyLensProperties.BFLY_U3_23S6C_FE185C086HA_1_RIGHT_MSA_CALIBRATED;
               BlackflyLensProperties properties = BlackflyLensProperties.BFS_U3_27S5C_FE185C086HA_1;
               textureCoordinateCalculator = new FisheyeTextureCalculator(CameraOrientation.X_DEPTH_POSITIVE,
                                                                          properties.getFocalLengthXForUndistortion(),
                                                                          properties.getFocalLengthYForUndistortion(),
                                                                          properties.getPrincipalPointXForUndistortion(),
                                                                          properties.getPrincipalPointYForUndistortion(),
                                                                          properties.getK1ForUndistortion(),
                                                                          properties.getK2ForUndistortion(),
                                                                          properties.getK3ForUndistortion(),
                                                                          properties.getK4ForUndistortion(),
                                                                          Math.toRadians(185.0));
               projectionHemisphere.setTextureCoordinateCalculator(textureCoordinateCalculator);
            }
         }
      }

      camerasToStereoMidPoint.get(RobotSide.LEFT)
                             .getTranslation()
                             .set(projectionSettings.cameraXOffsetCalibration.get(),
                                  projectionSettings.cameraYOffsetCalibration.get(),
                                  projectionSettings.cameraZOffsetCalibration.get());
      camerasToStereoMidPoint.get(RobotSide.LEFT)
                             .getRotation()
                             .setYawPitchRoll(projectionSettings.cameraYawOffsetCalibration.get(),
                                              projectionSettings.cameraPitchOffsetCalibration.get(),
                                              projectionSettings.cameraRollOffsetCalibration.get());
      camerasToStereoMidPoint.get(RobotSide.RIGHT).setAndInvert(camerasToStereoMidPoint.get(RobotSide.LEFT));
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
      textureUpdateFuture = executorService.scheduleAtFixedRate(() ->
      {
         for (RobotSide side : RobotSide.values)
         {
            byte[] imageByteArray = dualBlackflyUDPReceiver.getImageBuffers().get(side);

            if (imageByteArray != null)
            {

               newTextureMapDatas.get(side).set(generateNewTextureMapData(imageByteArray, dualBlackflyUDPReceiver.getImageDimensions().get(side)));
            }
         }
      }, 0, 20, java.util.concurrent.TimeUnit.MILLISECONDS);
   }

   public void disable()
   {
      // Disable on a thread, so we don't hang the UI
      ThreadTools.startAThread(this::stopReconnectThread, getClass().getSimpleName() + "StopReconnect");

      dualBlackflyUDPReceiver.stop();
      textureUpdateFuture.cancel(false);
   }

   public void shutdown()
   {
      stopReconnectThread();

      dualBlackflyUDPReceiver.stop();
   }

   public void render()
   {
      boolean hadNewFrame = false;

      updateParameters();

      for (RobotSide side : RobotSide.values)
      {
         projectionShapes.get(side).updateMeshLazy(camerasToStereoMidPoint.get(side));
         PixmapTextureData newTextureMapData = newTextureMapDatas.get(side).getAndSet(null);
         if (newTextureMapData != null)
         {
            projectionShapes.get(side).updateTexture(new Texture(newTextureMapData), 1.0f);
            hadNewFrame = true;
         }
      }

      if (baseUI.getVRManager().isVRReady() && hadNewFrame)
      {
         for (RDXReferenceFrameGraphic eyeFrameGraphic : eyeFrameGraphics)
            if (eyeFrameGraphic != null)
               eyeFrameGraphic.updateFromLastGivenFrame();
      }

      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform pose = projectionShapePoses.get(side);
         if (projectionSettings.projectionShapesLockOnRobot.get())
         {
            pose.set(stereoMidPointFrame.getTransformToRoot());
         }
         else if (baseUI.getVRManager().isVRReady())
         {
            RDXVRContext context = baseUI.getVRManager().getContext();
            //            RigidBodyTransform eyePose = context.getEyes().get(side).getEyeXForwardZUpFrame().getTransformToRoot();
            RigidBodyTransform eyePose = context.getHeadset().getXForwardZUpHeadsetFrame().getTransformToRoot();
            pose.getTranslation().set(eyePose.getTranslation());
            pose.getRotation().setToYawOrientation(eyePose.getRotation().getYaw());
            //            pose.getRotation().setToYawOrientation(robotZUpFrame.getTransformToRoot().getRotation().getYaw());
         }
         else
         {
            pose.setToZero();
         }

         pose.appendTranslation(projectionSettings.projectionXOffsetCalibration.get(),
                                side.negateIfLeftSide(projectionSettings.projectionYOffsetCalibration.get()),
                                projectionSettings.projectionZOffsetCalibration.get());
         pose.getRotation().appendYawRotation(side.negateIfRightSide(projectionSettings.projectionYawOffsetCalibration.get()));
         pose.getRotation().appendPitchRotation(side.negateIfRightSide(projectionSettings.projectionPitchOffsetCalibration.get()));
         pose.getRotation().appendRollRotation(side.negateIfRightSide(projectionSettings.projectionRollOffsetCalibration.get()));

         LibGDXTools.toLibGDX(pose, projectionShapes.get(side).getModelInstance().transform);
      }

      baseUI.renderBeforeOnScreenUI();
      baseUI.renderEnd();
   }

   private PixmapTextureData generateNewTextureMapData(byte[] imageByteArray, ImageDimensions imageDimensions)
   {
      BytePointer imageDataBytePointer = new BytePointer(imageByteArray);
      Mat imageMat = new Mat(imageDimensions.getImageHeight(), imageDimensions.getImageWidth(), opencv_core.CV_8UC1, imageDataBytePointer);

      Pixmap imagePixmap = new Pixmap(imageMat.cols(), imageMat.rows(), Format.RGBA8888);
      BytePointer imageRGBABytePointer = new BytePointer(imagePixmap.getPixels());
      Mat imageRGBAMat = new Mat(imageMat.rows(), imageMat.cols(), opencv_core.CV_8UC4, imageRGBABytePointer);
      opencv_imgproc.cvtColor(imageMat, imageRGBAMat, opencv_imgproc.COLOR_BayerBG2RGBA);

      // TODO Consider the following for anti-aliasing
      //      Pixmap imageBigAssPixmap = new Pixmap(2 * imageMat.cols(), 2 * imageMat.rows(), Format.RGBA8888);
      //      BytePointer imageBigAssRGBABytePointer = new BytePointer(imageBigAssPixmap.getPixels());
      //      Mat imageBigAssRGBAMat = new Mat(2 * imageMat.rows(), 2 * imageMat.cols(), opencv_core.CV_8UC4, imageBigAssRGBABytePointer);
      //      opencv_imgproc.resize(imageRGBAMat, imageBigAssRGBAMat, imageBigAssRGBAMat.size());

      PixmapTextureData pixmapTextureData = new PixmapTextureData(imagePixmap, null, false, true);

      imageDataBytePointer.close();
      imageMat.close();

      return pixmapTextureData;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VR_EYE_RIGHT))
      {
         projectionShapes.get(RobotSide.RIGHT).getRenderables(renderables, pool);
      }
      else if (sceneLevels.contains(RDXSceneLevel.VR_EYE_LEFT))
      {
         projectionShapes.get(RobotSide.LEFT).getRenderables(renderables, pool);
      }
      else
      {
         if (projectionSettings.showRightProjectionShape.get())
            projectionShapes.get(RobotSide.RIGHT).getRenderables(renderables, pool);
         if (projectionSettings.showLeftProjectionShape.get())
            projectionShapes.get(RobotSide.LEFT).getRenderables(renderables, pool);

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
}
