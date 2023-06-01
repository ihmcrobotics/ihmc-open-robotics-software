package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.PlanarRegionMappingHandler;
import us.ihmc.perception.tools.MocapTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXLineGraphic;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.util.ArrayList;

public class RDXPlanarRegionMappingDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "filtered_map_node");

   private static final File regionLogDirectory = new File(IHMCCommonPaths.LOGS_DIRECTORY + "/");

   private PlanarRegionMappingHandler mappingManager;
   private RDXPlanarRegionMappingUI mappingUI;

   private final RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();

   private final RDXLineGraphic mocapGraphic = new RDXLineGraphic(0.02f, Color.YELLOW);
   private final RDXLineGraphic rootJointGraphic = new RDXLineGraphic(0.02f, Color.RED);

   private final String perceptionLogFile = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20230517_114430_PerceptionLog_900_ms.hdf5").toString();

   private final RDXPlanarRegionsGraphic mapPlanarRegionsGraphic = new RDXPlanarRegionsGraphic();
   private final ArrayList<ModelInstance> poseModels = new ArrayList<>();
   private ModelInstance modelInstance;

   private final FramePose3D framePose = new FramePose3D(ReferenceFrame.getWorldFrame());
   private final FramePose3D framePreviousPose = new FramePose3D(ReferenceFrame.getWorldFrame());

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private RDXLineGraphic keyframeTrajectoryGraphic = new RDXLineGraphic(0.02f, Color.WHITE);

   private boolean graphicsInitialized = false;

   public RDXPlanarRegionMappingDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            // To Run With Perception Logs (HDF5)
            mappingManager = new PlanarRegionMappingHandler(perceptionLogFile, true);

            pointCloudRenderer.create(mappingManager.getRapidRegionsExtractor().getImageHeight() * mappingManager.getRapidRegionsExtractor().getImageWidth());

            // To Run with Planar Region Logs (PRLLOG)
            //mappingManager = new PlanarRegionMappingManager(regionLogDirectory , false);

            // To Run in Live Mode (ROS2)
            //mappingManager = new PlanarRegionMappingManager("Nadia", ros2Node, false);

            mapPlanarRegionsGraphic.generateMeshes(mappingManager.pollMapRegions());
            mapPlanarRegionsGraphic.update();
            mapPlanarRegionsGraphic.setupTooltip(baseUI.getPrimary3DPanel(), "");

            mappingUI = new RDXPlanarRegionMappingUI("Filtered Map", mappingManager);
            baseUI.getImGuiPanelManager().addPanel(mappingUI.getImGuiPanel());

            baseUI.getPrimaryScene().addRenderableProvider(mappingUI, RDXSceneLevel.VIRTUAL);
            baseUI.getPrimaryScene().addRenderableProvider(mapPlanarRegionsGraphic::getRenderables, RDXSceneLevel.VIRTUAL);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer::getRenderables, RDXSceneLevel.VIRTUAL);

            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(mapPlanarRegionsGraphic::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(mapPlanarRegionsGraphic::process3DViewInput);

            baseUI.getPrimaryScene().addRenderableProvider(mocapGraphic, RDXSceneLevel.VIRTUAL);
            baseUI.getPrimaryScene().addRenderableProvider(rootJointGraphic, RDXSceneLevel.VIRTUAL);
         }

         public void renderPlanarRegions()
         {
            if (mappingManager.pollIsModified() && mappingManager.hasPlanarRegionsToRender())
            {
               mapPlanarRegionsGraphic.clear();
               mapPlanarRegionsGraphic.generateMeshes(mappingManager.pollMapRegions());
               mapPlanarRegionsGraphic.update();

               RigidBodyTransform transform = mappingManager.pollKeyframePose();
               if (transform != null)
               {
                  framePose.set(transform);
                  modelInstance = RDXModelBuilder.createCoordinateFrameInstance(0.04, Color.GREEN);

                  //                  keyframeTrajectoryGraphic.generateMeshes();

                  LibGDXTools.toLibGDX(framePose, tempTransform, modelInstance.transform);
                  poseModels.add(modelInstance);
               }
               framePreviousPose.set(framePose);

               for (ModelInstance model : poseModels)
               {
                  baseUI.getPrimaryScene().addRenderableProvider(model, RDXSceneLevel.VIRTUAL);
               }

               if (mappingUI.getPointCloudRenderEnabled())
               {
                  pointCloudRenderer.setPointsToRender(mappingManager.getRapidRegionsExtractor().getDebugger().getDebugPoints(), Color.GRAY);
                  pointCloudRenderer.updateMesh();
               }
            }
         }

         @Override
         public void render()
         {

            if (!mappingManager.getMocapPositionBuffer().isEmpty() && !graphicsInitialized)
            {
               MocapTools.adjustMocapPositionsByOffset(mappingManager.getMocapPositionBuffer(), mappingManager.getSensorPositionBuffer().get(0));

               if (!mappingManager.getMocapPositionBuffer().isEmpty() && !graphicsInitialized)
               {
                  MocapTools.adjustMocapPositionsByOffset(mappingManager.getMocapPositionBuffer(), mappingManager.getSensorPositionBuffer().get(0));

                  mocapGraphic.generateMeshes(mappingManager.getMocapPositionBuffer(), 10);
                  mocapGraphic.update();
               }

               if (!mappingManager.getSensorPositionBuffer().isEmpty())
               {
                  rootJointGraphic.generateMeshes(mappingManager.getSensorPositionBuffer(), 5);
                  rootJointGraphic.update();
               }

               graphicsInitialized = true;

               if (mappingUI.isCaptured())
               {
                  LogTools.info("Filtered Map Panel Captured: {}", mappingUI.isCaptured());
                  mappingManager.setCaptured(true);
                  mappingUI.setCaptured(false);
               }

               //rapidRegionsUIPanel.renderImGuiWidgets();

               renderPlanarRegions();
            }

            if (!mappingManager.getSensorPositionBuffer().isEmpty())
            {
               rootJointGraphic.generateMeshes(mappingManager.getSensorPositionBuffer(), 5);
               rootJointGraphic.update();
            }

            graphicsInitialized = true;

            renderPlanarRegions();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            ros2Node.destroy();
            mappingManager.destroy();
            super.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPlanarRegionMappingDemo();
   }
}
