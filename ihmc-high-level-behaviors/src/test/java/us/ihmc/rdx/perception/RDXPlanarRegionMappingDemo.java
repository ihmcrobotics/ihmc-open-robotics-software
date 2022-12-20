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

            mappingManager = new PlanarRegionMappingManager(true);

            baseUI.getImGuiPanelManager().addPanel(mappingManager.getGpuPlanarRegionExtractionUI().getPanel());
            baseUI.getPrimaryScene().addRenderableProvider(mappingManager.getGpuPlanarRegionExtractionUI()::getVirtualRenderables, RDXSceneLevel.VIRTUAL);

            planarRegionMappingUI = new PlanarRegionMappingUIPanel("Filtered Map", mappingManager);
            baseUI.getImGuiPanelManager().addPanel(planarRegionMappingUI.getImGuiPanel());
            baseUI.getPrimaryScene().addRenderableProvider(planarRegionMappingUI::getVirtualRenderables, RDXSceneLevel.VIRTUAL);

            baseUI.getPerspectiveManager().reloadPerspective();
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
