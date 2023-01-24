package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.tools.MocapTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXLineMeshModel;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.Activator;

import java.io.File;

public class RDXPlanarRegionMappingDemo
{
   private Activator nativesLoadedActivator;
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "filtered_map_node");

   private static final File regionLogDirectory = new File(
         System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);

   private PlanarRegionMappingManager mappingManager;
   private PlanarRegionMappingUIPanel planarRegionMappingUI;

   private final RDXLineMeshModel mocapGraphic = new RDXLineMeshModel(0.02f, Color.YELLOW);
   private final RDXLineMeshModel rootJointGraphic = new RDXLineMeshModel(0.02f, Color.RED);

   private final String perceptionLogDirectory = System.getProperty("user.home") + "/.ihmc/logs/perception/";
   private final String logFileName = "20230117_161540_PerceptionLog.hdf5";

   private final RDXPlanarRegionsGraphic mapPlanarRegionsGraphic = new RDXPlanarRegionsGraphic();

   private boolean graphicsInitialized = false;

   public RDXPlanarRegionMappingDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
            baseUI.create();


            // To Run With Perception Logs (HDF5)
            mappingManager = new PlanarRegionMappingManager(perceptionLogDirectory + logFileName, true);


            // To Run with Planar Region Logs (PRLLOG)
            //mappingManager = new PlanarRegionMappingManager(regionLogDirectory , false);

            // To Run in Live Mode (ROS2)
            //mappingManager = new PlanarRegionMappingManager("Nadia", ros2Node, false);

            mapPlanarRegionsGraphic.generateMeshes(mappingManager.pollMapRegions());
            mapPlanarRegionsGraphic.update();
            mapPlanarRegionsGraphic.setupTooltip(baseUI.getPrimary3DPanel(), "");

            planarRegionMappingUI = new PlanarRegionMappingUIPanel("Filtered Map", mappingManager);
            baseUI.getImGuiPanelManager().addPanel(planarRegionMappingUI.getImGuiPanel());
            baseUI.getPrimaryScene().addRenderableProvider(mapPlanarRegionsGraphic::getRenderables, RDXSceneLevel.VIRTUAL);

            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(mapPlanarRegionsGraphic::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(mapPlanarRegionsGraphic::process3DViewInput);

            baseUI.getPerspectiveManager().reloadPerspective();
         }


         public void renderPlanarRegions()
         {
            if (mappingManager.pollIsModified() && mappingManager.hasPlanarRegionsToRender())
            {
               LogTools.info("Calling Update on Graphic ------------------------------------------------------------+++++++++++++++++++++++++++++++++++++++++++++++++++");
               mapPlanarRegionsGraphic.clear();
               mapPlanarRegionsGraphic.generateMeshes(mappingManager.pollMapRegions());
               mapPlanarRegionsGraphic.update();
            }
         }

         @Override
         public void render()
         {
            if(nativesLoadedActivator.poll())
            {
               if(nativesLoadedActivator.isNewlyActivated())
               {
                  baseUI.getPrimaryScene().addRenderableProvider(mocapGraphic, RDXSceneLevel.VIRTUAL);
                  baseUI.getPrimaryScene().addRenderableProvider(rootJointGraphic, RDXSceneLevel.VIRTUAL);
               }

               if(!mappingManager.getSensorPositionBuffer().isEmpty() && !mappingManager.getMocapPositionBuffer().isEmpty() && !graphicsInitialized)
               {
                  MocapTools.adjustMocapPositionsByOffset(mappingManager.getMocapPositionBuffer(),
                                                          mappingManager.getSensorPositionBuffer().get(0));

                  mocapGraphic.generateMeshes(mappingManager.getMocapPositionBuffer(), 10);
                  mocapGraphic.update();

                  rootJointGraphic.generateMeshes(mappingManager.getSensorPositionBuffer(), 5);
                  rootJointGraphic.update();

                  graphicsInitialized = true;
               }

               if (planarRegionMappingUI.isCaptured())
               {
                  LogTools.info("Filtered Map Panel Captured: {}", planarRegionMappingUI.isCaptured());
                  mappingManager.setCaptured(true);
                  planarRegionMappingUI.setCaptured(false);
               }

               //rapidRegionsUIPanel.renderImGuiWidgets();

               renderPlanarRegions();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            ros2Node.destroy();
            super.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPlanarRegionMappingDemo();
   }
}
