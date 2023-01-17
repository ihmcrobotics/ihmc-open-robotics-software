package us.ihmc.rdx.perception;

import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsCustomizer;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
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

   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private RDXRapidRegionsUIPanel rapidRegionsUIPanel;
   private RapidPlanarRegionsExtractor rapidRegionsExtractor;
   private RapidPlanarRegionsCustomizer rapidRegionCustomizer;

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

            depth32FC1Image = new BytedecoImage(1024, 768, opencv_core.CV_32FC1);

            /* L515 Parameters
                  Color: [fx:901.3026, fy:901.8400, cx:635.2337, cy:350.9427, h:720, w:1280]
                  Depth: [fx:730.7891, fy:731.0859, cx:528.6094, cy:408.1602, h:768, w:1024]
             */
            openCLManager = new OpenCLManager();
            openCLManager.create();
            openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");

            rapidRegionsExtractor = new RapidPlanarRegionsExtractor();
            rapidRegionCustomizer = new RapidPlanarRegionsCustomizer();

            rapidRegionsExtractor.create(openCLManager, openCLProgram, 1024, 768,
                                         730.7891,
                                         731.0859,
                                         528.6094,
                                         408.1602);

            mappingManager = new PlanarRegionMappingManager(rapidRegionsExtractor, rapidRegionCustomizer, true);

            rapidRegionsUIPanel = new RDXRapidRegionsUIPanel();
            rapidRegionsUIPanel.create(rapidRegionsExtractor);
            rapidRegionsUIPanel.getEnabled().set(true);
            baseUI.getImGuiPanelManager().addPanel(rapidRegionsUIPanel.getPanel());
            baseUI.getPrimaryScene().addRenderableProvider(rapidRegionsUIPanel, RDXSceneLevel.VIRTUAL);

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

            if(mappingManager.isModified())
            {
               rapidRegionsUIPanel.render3DGraphics(mappingManager.getPlanarRegionsListWithPose().getPlanarRegionsList(), ReferenceFrame.getWorldFrame());
               mappingManager.setModified(false);
            }

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
