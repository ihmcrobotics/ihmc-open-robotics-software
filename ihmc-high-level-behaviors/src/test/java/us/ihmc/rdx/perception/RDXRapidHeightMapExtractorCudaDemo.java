package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractorCuda;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXSensorSimulator;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.referenceFrames.ZUpFrame;

public class RDXRapidHeightMapExtractorCudaDemo
{
   private static final float FOV = 70.0f;
   private static final float MIN_RANGE = 0.2f;
   private static final float MAX_RANGE = 20.0f;

   private final Throttler throttler = new Throttler().setFrequency(30.0);
   private final RDXSensorSimulator sensorSimulator;
   private RDXPose3DGizmo sensorPoseGizmo;
   private RDXEnvironmentBuilder environmentBuilder;
   private RapidHeightMapExtractorCuda extractor;
   private ZUpFrame sensorZUpFrame;
   private final RigidBodyTransform sensorToSensorZUp = new RigidBodyTransform();
   private final RigidBodyTransform sensorZUpToWorld = new RigidBodyTransform();
   private GpuMat heightMapImage;

   public RDXRapidHeightMapExtractorCudaDemo()
   {
      sensorSimulator = new RDXSensorSimulator(1280, 720, FOV, MIN_RANGE, MAX_RANGE);

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

            extractor = new RapidHeightMapExtractorCuda(leftFootSoleFrame, rightFootSoleFrame);
            extractor.setDepthIntrinsics(intrinsics);
            extractor.create(heightMapImage, 1);
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
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            extractor.destroy();
            environmentBuilder.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXRapidHeightMapExtractorCudaDemo();
   }
}
