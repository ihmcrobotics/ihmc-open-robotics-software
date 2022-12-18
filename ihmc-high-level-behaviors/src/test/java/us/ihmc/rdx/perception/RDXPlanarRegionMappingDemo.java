package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.ros2.ROS2Node;

public class RDXPlanarRegionMappingDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "filtered_map_node");

   private RDXGPUPlanarRegionExtractionUI planarRegionExtractionUI;
   private PlanarRegionMappingUIPanel planarRegionMappingUI;
   private PlanarRegionMappingManager mappingManager;
   private BytedecoImage depth32FC1Image;

   public RDXPlanarRegionMappingDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            //Color: [fx:901.3026, fy:901.8400, cx:635.2337, cy:350.9427, h:720, w:1280]
            //Depth: [fx:730.7891, fy:731.0859, cx:528.6094, cy:408.1602, h:768, w:1024]

            depth32FC1Image = new BytedecoImage(1024, 768, opencv_core.CV_32FC1);

            planarRegionExtractionUI = new RDXGPUPlanarRegionExtractionUI();
            planarRegionExtractionUI.create(1024,
                                            768,
                                            depth32FC1Image.getBackingDirectByteBuffer(),
                                            730.7891,
                                            731.0859,
                                            528.6094,
                                            408.1602,
                                            ReferenceFrame.getWorldFrame());

            planarRegionExtractionUI.getRender3DPlanarRegions().set(false);
            planarRegionExtractionUI.getRender3DBoundaries().set(false);
            planarRegionExtractionUI.getRender3DGrownBoundaries().set(false);

            mappingManager = new PlanarRegionMappingManager(planarRegionExtractionUI, depth32FC1Image, true);

            baseUI.getImGuiPanelManager().addPanel(planarRegionExtractionUI.getPanel());
            baseUI.getPrimaryScene().addRenderableProvider(planarRegionExtractionUI::getVirtualRenderables, RDXSceneLevel.VIRTUAL);

            planarRegionMappingUI = new PlanarRegionMappingUIPanel("Filtered Map", mappingManager);
            baseUI.getImGuiPanelManager().addPanel(planarRegionMappingUI.getImGuiPanel());
            baseUI.getPrimaryScene().addRenderableProvider(planarRegionMappingUI::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
         }

         @Override
         public void render()
         {
            if (planarRegionMappingUI.isCaptured())
            {
               LogTools.info("Filtered Map Panel Captured: {}", planarRegionMappingUI.isCaptured());
               mappingManager.setCaptured(true);
               planarRegionMappingUI.setCaptured(false);
            }

            planarRegionMappingUI.renderPlanarRegions();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            super.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPlanarRegionMappingDemo();
   }
}
