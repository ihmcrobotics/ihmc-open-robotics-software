package us.ihmc.rdx.perception;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractorCUDA;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXSensorSimulator;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXGridMapGraphic;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public class RDXRapidHeightMapExtractorCUDADemo
{
   private static final float FOV = 70.0f;
   private static final float MIN_RANGE = 0.2f;
   private static final float MAX_RANGE = 20.0f;
   private static final int NOISE = 2;

   private final Throttler throttler = new Throttler().setFrequency(30.0);
   private final RDXSensorSimulator sensorSimulator;
   private RDXPose3DGizmo sensorPoseGizmo;
   private RDXEnvironmentBuilder environmentBuilder;
   private RapidHeightMapExtractorCUDA extractor;
   private ZUpFrame sensorZUpFrame;
   private final RigidBodyTransform sensorToSensorZUp = new RigidBodyTransform();
   private final RigidBodyTransform sensorZUpToWorld = new RigidBodyTransform();
   private GpuMat heightMapImage;
   private final RDXGridMapGraphic heightMapGraphic = new RDXGridMapGraphic();
   private final BytePointer compressedCroppedHeightMapPointer = new BytePointer();
   private final ImageMessage croppedHeightMapImageMessage = new ImageMessage();
   private final FramePose3D cameraPoseForHeightMap = new FramePose3D();
   private float pixelScalingFactor = 10000.0f;
   private boolean heightMapMessageGenerated = false;
   private final RigidBodyTransform zUpToWorldTransform = new RigidBodyTransform();
   private HeightMapData heightMapData;
   private final HeightMapMessage heightMapMessage = new HeightMapMessage();

   public RDXRapidHeightMapExtractorCUDADemo()
   {
      sensorSimulator = new RDXSensorSimulator(1280, 720, FOV, MIN_RANGE, MAX_RANGE, NOISE);

      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create(RDXSceneLevel.GROUND_TRUTH, RDXSceneLevel.MODEL, RDXSceneLevel.VIRTUAL);

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            environmentBuilder.loadEnvironment("HarderTerrain.json");
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder);

            sensorSimulator.create(baseUI.getPrimaryScene());
            sensorSimulator.enableColor(true);
            sensorSimulator.enableDepth(true);

            sensorPoseGizmo = new RDXPose3DGizmo();
            sensorPoseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.getTransformToParent().getTranslation().set(0.3, 0.0, 1.5);
            sensorPoseGizmo.getTransformToParent().getRotation().setYawPitchRoll(0.0, Math.toRadians(38.0), 0.0);
            sensorPoseGizmo.update();

            RigidBodyTransform leftSoleToGizmo = new RigidBodyTransform();
            leftSoleToGizmo.getTranslation().set(0.0, 0.1, -0.5);
            RigidBodyTransform rightSoleToGizmo = new RigidBodyTransform();
            rightSoleToGizmo.getTranslation().set(0.0, -0.1, -0.5);
            ReferenceFrame leftFootSoleFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("leftFootSoleFrame",
                                                                                                                 sensorPoseGizmo.getGizmoFrame(),
                                                                                                                 leftSoleToGizmo);
            ReferenceFrame rightFootSoleFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("rightFootSoleFrame",
                                                                                                                  sensorPoseGizmo.getGizmoFrame(),
                                                                                                                  rightSoleToGizmo);
            sensorZUpFrame = new ZUpFrame(sensorPoseGizmo.getGizmoFrame(), "sensorZUpFrame");

            CameraIntrinsics intrinsics = sensorSimulator.getDepthImage().getIntrinsicsCopy();

            heightMapImage = new GpuMat(intrinsics.getWidth(), intrinsics.getHeight(), opencv_core.CV_16UC1);

            extractor = new RapidHeightMapExtractorCUDA(leftFootSoleFrame,
                                                        rightFootSoleFrame,
                                                        heightMapImage,
                                                        intrinsics,
                                                        1,
                                                        new HeightMapParameters("GPU"));

            baseUI.getPrimaryScene().addRenderableProvider(heightMapGraphic, RDXSceneLevel.MODEL);
         }

         @Override
         public void render()
         {
            if (throttler.run())
            {
               // "Grab" the image
               sensorSimulator.render(sensorPoseGizmo.getTransformToParent());

               sensorZUpFrame.update();
               sensorPoseGizmo.getGizmoFrame().getTransformToDesiredFrame(sensorToSensorZUp, sensorZUpFrame);
               sensorZUpFrame.getTransformToDesiredFrame(sensorZUpToWorld, ReferenceFrame.getWorldFrame());

               heightMapImage.upload(sensorSimulator.getDepthImage().getCpuImageMat());

               extractor.update(sensorPoseGizmo.getTransformToParent(), sensorToSensorZUp, sensorZUpToWorld);

               heightMapData = extractor.getHeightMapData();

               Mat croppedHeightMapImage = extractor.getTerrainMapData().getHeightMap();

               cameraPoseForHeightMap.setToZero(sensorPoseGizmo.getGizmoFrame());
               cameraPoseForHeightMap.changeFrame(ReferenceFrame.getWorldFrame());

               pixelScalingFactor = croppedHeightMapImageMessage.getDepthDiscretization();
               zUpToWorldTransform.set(croppedHeightMapImageMessage.getOrientation(), croppedHeightMapImageMessage.getPosition());

               //               PerceptionMessageTools.convertToHeightMapImage(croppedHeightMapImageMessage,
               //                                                              heightMapImage,
               //                                                              incomingCompressedImageBuffer,
               //                                                              incomingCompressedImageBytePointer,
               //                                                              compressedBytesMat);

               HeightMapParameters heightMapParameters = new HeightMapParameters("GPU");
               PerceptionMessageTools.convertToHeightMapData(sensorSimulator.getDepthImage().getCpuImageMat(),
                                                             heightMapData,
                                                             croppedHeightMapImageMessage.getPosition(),
                                                             (float) heightMapParameters.getGlobalWidthInMeters(),
                                                             (float) heightMapParameters.getGlobalCellSizeInMeters(),
                                                             heightMapParameters);
               HeightMapMessageTools.toMessage(heightMapData, heightMapMessage);

               //               List<RDXMultiColorMeshBuilder> multiColorMeshBuilders = RDXHeightMapGraphicNew.generateHeightCells(heightsProvided,
               //                                                                                                                  keysProvider,
               //                                                                                                                  numberOfOccupiedCells,
               //                                                                                                                  gridResolutionXY,
               //                                                                                                                  gridSizeXy,
               //                                                                                                                  gridCenterX,
               //                                                                                                                  gridCenterY,
               //                                                                                                                  groundHeight);
               //               heightMapGraphic.setLatestMeshBuilder(multiColorMeshBuilders);

               heightMapGraphic.generateMeshesAsync(heightMapMessage);
               heightMapGraphic.update();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            heightMapGraphic.destroy();
            extractor.destroy();
            environmentBuilder.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXRapidHeightMapExtractorCUDADemo();
   }
}
