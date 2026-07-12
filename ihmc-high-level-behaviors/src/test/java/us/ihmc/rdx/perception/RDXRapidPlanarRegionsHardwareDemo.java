package us.ihmc.rdx.perception;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractionThread;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2FramePlanarRegionsVisualizer;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class RDXRapidPlanarRegionsHardwareDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private RDXPose3DGizmo zedPoseGizmo = new RDXPose3DGizmo();
   private RDXRawImagePointCloudVisualizer pointCloudVisualizer;
   private RDXRapidRegionsUI rapidRegionsUI;
   private RDXROS2FramePlanarRegionsVisualizer planarRegionsVisualizer;
   private final ZEDImageSensor zedImageSensor;
   private final RepeatingTaskThread zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::zedGrabThread);
   private final ROS2Node ros2Node = new ROS2Node("rapid_regions_node");
   private final RapidPlanarRegionsExtractionThread rapidPlanarRegionsExtractionThread;

   public RDXRapidPlanarRegionsHardwareDemo()
   {
      zedImageSensor = new ZEDImageSensor(0, ZEDModelData.ZED_2, zed.SL_INPUT_TYPE_USB);
      zedImageSensor.run(true);

      BlockingQueue<RawImage> rapidRegionsDepthQueue = new LinkedBlockingQueue<>(ImageSensor.DEFAULT_IMAGE_QUEUE_CAPACITY);
      zedImageSensor.registerImageQueue(rapidRegionsDepthQueue, ZEDImageSensor.DEPTH_IMAGE_KEY);

      rapidPlanarRegionsExtractionThread = new RapidPlanarRegionsExtractionThread(ros2Node, rapidRegionsDepthQueue);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            pointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud");
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudVisualizer);
            baseUI.getImGuiPanelManager().addPanel("Point Cloud", pointCloudVisualizer::renderImGuiWidgets);

            planarRegionsVisualizer = new RDXROS2FramePlanarRegionsVisualizer("Planar Regions", ros2Node, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS);
            planarRegionsVisualizer.setActive(true);
            baseUI.getPrimaryScene().addRenderableProvider(planarRegionsVisualizer);
            baseUI.getImGuiPanelManager().addPanel("Planar Regions", planarRegionsVisualizer::renderImGuiWidgets);

            rapidPlanarRegionsExtractionThread.startRepeating();
            zedGrabThread.startRepeating();

            robotInteractableReferenceFrame = new RDXInteractableReferenceFrame();
            robotInteractableReferenceFrame.create(ReferenceFrame.getWorldFrame(), 0.15, baseUI.getPrimary3DPanel());
            robotInteractableReferenceFrame.getTransformToParent().getTranslation().add(2.2, 0.0, 1.0);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(robotInteractableReferenceFrame::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(robotInteractableReferenceFrame::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            zedPoseGizmo = new RDXPose3DGizmo(robotInteractableReferenceFrame.getRepresentativeReferenceFrame());
            zedPoseGizmo.create(baseUI.getPrimary3DPanel());
            zedPoseGizmo.setResizeAutomatically(false);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(zedPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(zedPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(zedPoseGizmo, RDXSceneLevel.VIRTUAL);
            zedPoseGizmo.getTransformToParent().appendPitchRotation(Math.toRadians(10.0));
         }

         @Override
         public void render()
         {
            if (rapidRegionsUI == null && rapidPlanarRegionsExtractionThread.getExtractor() != null)
            {
               rapidRegionsUI = new RDXRapidRegionsUI();
               rapidRegionsUI.create(rapidPlanarRegionsExtractionThread.getExtractor());
               baseUI.getImGuiPanelManager().addPanel(rapidRegionsUI.getPanel());
               baseUI.getLayoutManager().reloadLayout();
            }

            pointCloudVisualizer.update();
            planarRegionsVisualizer.update();
            if (rapidRegionsUI != null)
               rapidRegionsUI.render();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            zedGrabThread.blockingKill();
            rapidPlanarRegionsExtractionThread.kill();
            zedImageSensor.close();
            pointCloudVisualizer.destroy();
            planarRegionsVisualizer.destroy();
            rapidRegionsUI.destroy();
            ros2Node.close();

            baseUI.dispose();
         }
      });
   }

   private void zedGrabThread() throws InterruptedException
   {
      zedImageSensor.waitForGrab();

      RawImage colorImage = zedImageSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      RawImage depthImage = zedImageSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

      pointCloudVisualizer.setColorImage(colorImage);
      pointCloudVisualizer.setDepthImage(depthImage);

      colorImage.release();
      depthImage.release();
   }

   public static void main(String[] args)
   {
      new RDXRapidPlanarRegionsHardwareDemo();
   }
}
