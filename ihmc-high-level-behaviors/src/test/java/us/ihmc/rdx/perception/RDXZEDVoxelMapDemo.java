package us.ihmc.rdx.perception;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.voxelMap.VoxelMapExtractor;
import us.ihmc.perception.voxelMap.VoxelMap;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.RDXVoxelMapRenderer;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;

public class RDXZEDVoxelMapDemo
{
   private static final int MAP_SIZE = 50;      // voxels per axis → 5 m span at 10 cm resolution
   private static final float VOXEL_SIZE = 0.1f;

   private ZEDImageSensor zedImageSensor;
   private VoxelMapExtractor voxelMapExtractor;
   private RDXPose3DGizmo mapOriginGizmo;
   private final RDXVoxelMapRenderer voxelMapRenderer = new RDXVoxelMapRenderer();
   private final RDXRawImagePointCloudVisualizer pointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud");
   private final RepeatingTaskThread zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::grabAndExtract);
   private volatile VoxelMap latestVoxelMap;
   // Written on the render thread, snapshot-copied on the grab thread — one frame stale is fine
   private final Pose3D mapOrigin = new Pose3D();

   public RDXZEDVoxelMapDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            mapOriginGizmo = new RDXPose3DGizmo();
            mapOriginGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
            baseUI.getPrimaryScene().addRenderableProvider(mapOriginGizmo);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(mapOriginGizmo::calculate3DViewPick);

            zedImageSensor = new ZEDImageSensor(0, 0, ZEDModelData.ZED_2I, zed.SL_INPUT_TYPE_USB, zed.SL_DEPTH_MODE_NEURAL, zed.SL_RESOLUTION_VGA, 15);
            zedImageSensor.run(true);

            voxelMapExtractor = new VoxelMapExtractor(MAP_SIZE, VOXEL_SIZE);
            voxelMapRenderer.create(MAP_SIZE * MAP_SIZE * MAP_SIZE);
            baseUI.getPrimaryScene().addRenderableProvider(voxelMapRenderer);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudVisualizer);

            baseUI.getImGuiPanelManager().addPanel("Point Cloud", pointCloudVisualizer::renderImGuiWidgets);

            zedGrabThread.startRepeating();
         }

         @Override
         public void render()
         {
            mapOrigin.set(mapOriginGizmo.getTransformToParent());

            VoxelMap map = latestVoxelMap;
            if (map != null)
               voxelMapRenderer.update(map);

            pointCloudVisualizer.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            zedGrabThread.blockingKill();
            zedImageSensor.close();
            voxelMapExtractor.close();
            voxelMapRenderer.dispose();
            pointCloudVisualizer.destroy();
            baseUI.dispose();
         }
      });
   }

   private void grabAndExtract() throws InterruptedException
   {
      zedImageSensor.waitForGrab();

      RawImage depthImage = zedImageSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
      RawImage colorImage = zedImageSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);

      // Both setDepthImage and setColorImage call get() internally; the visualizer owns those refs
      pointCloudVisualizer.setDepthImage(depthImage);
      pointCloudVisualizer.setColorImage(colorImage);

      // Snapshot the gizmo pose so the extractor sees a consistent transform per frame
      Pose3D framePose = new Pose3D();
      framePose.set(mapOrigin);
      // getVoxelMap calls get()/release() internally; we release our ref below
      latestVoxelMap = voxelMapExtractor.getVoxelMap(framePose, depthImage);

      depthImage.release();
      colorImage.release();
   }

   public static void main(String[] args)
   {
      new RDXZEDVoxelMapDemo();
   }
}
