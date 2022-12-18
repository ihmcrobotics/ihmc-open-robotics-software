package us.ihmc.rdx.perception;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.ros2.ROS2Node;

public class RDXPlanarRegionMappingDemo
{
   private PlanarRegionMappingManager mapHandler;
   private PlanarRegionMappingUIPanel planarRegionMappingUI;
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");

   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "filtered_map_node");

   public RDXPlanarRegionMappingDemo()
   {
      mapHandler = new PlanarRegionMappingManager(ros2Node, true);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            planarRegionMappingUI = new PlanarRegionMappingUIPanel("Filtered Map", mapHandler);
            baseUI.getImGuiPanelManager().addPanel(planarRegionMappingUI.getImGuiPanel());
            baseUI.getPrimaryScene().addRenderableProvider(planarRegionMappingUI::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
         }

         @Override
         public void render()
         {
            if (planarRegionMappingUI.isCaptured())
            {
               LogTools.info("Filtered Map Panel Captured: {}", planarRegionMappingUI.isCaptured());
               mapHandler.setCaptured(true);
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
