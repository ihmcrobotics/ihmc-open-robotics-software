package us.ihmc.rdx.perception;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.ros2.ROS2Node;

import java.io.File;

public class RDXPlanarRegionMappingDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "filtered_map_node");

   private static final File regionLogDirectory = new File(
         System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private PlanarRegionMappingManager mappingManager;
   private PlanarRegionMappingUIPanel planarRegionMappingUI;

   private final String perceptionLogDirectory = System.getProperty("user.home") + "/.ihmc/logs/perception/";
   private final String logFileName = "20230117_161540_PerceptionLog.hdf5";

   public RDXPlanarRegionMappingDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            // To Run With Perception Logs (HDF5)
            mappingManager = new PlanarRegionMappingManager(perceptionLogDirectory + logFileName, false);

            // To Run with Planar Region Logs (PRLLOG)
            //mappingManager = new PlanarRegionMappingManager(regionLogDirectory , false);

            // To Run in Live Mode (ROS2)
            //mappingManager = new PlanarRegionMappingManager("Nadia", ros2Node, false);

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

            //rapidRegionsUIPanel.renderImGuiWidgets();

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
