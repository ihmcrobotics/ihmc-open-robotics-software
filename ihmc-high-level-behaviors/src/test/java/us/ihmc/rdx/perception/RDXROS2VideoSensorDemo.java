package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2BigVideoVisualizer;
import us.ihmc.rdx.ui.graphics.RDXGeneralToolsPanel;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class RDXROS2VideoSensorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();

   private RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
   private RDXEnvironmentBuilder environmentBuilder;

   public RDXROS2VideoSensorDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {

         private RDXGeneralToolsPanel globalVisualizersPanel;

         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("FlatGround.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);

            PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;
            globalVisualizersPanel = new RDXGeneralToolsPanel();

            RDXROS2BigVideoVisualizer videoVisualizer = new RDXROS2BigVideoVisualizer("Video",
                                                                                      pubSubImplementation,
                                                                                      PerceptionAPI.BIG_VIDEO);
            videoVisualizer.setSubscribed(true);
            globalVisualizersPanel.addVisualizer(videoVisualizer);

            globalVisualizersPanel.create();
            baseUI.getImGuiPanelManager().addPanel(globalVisualizersPanel);

            // https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/opengl-perspective-projection-matrix
            double publishRateHz = 60.0;
            double verticalFOV = 55.0;
            int imageWidth = 640;
            int imageHeight = 480;
            // range should be as small as possible because depth precision is nonlinear
            // it gets drastically less precise father away
            double minRange = 0.105;
            double maxRange = 5.0;
            highLevelDepthSensorSimulator = new RDXHighLevelDepthSensorSimulator("Stepping L515",
                                                                                 sensorPoseGizmo.getGizmoFrame(),
                                                                                 () -> 0L,
                                                                                 verticalFOV,
                                                                                 imageWidth,
                                                                                 imageHeight,
                                                                                 minRange,
                                                                                 maxRange,
                                                                                 0.03,
                                                                                 0.05,
                                                                                 true,
                                                                                 publishRateHz);
            highLevelDepthSensorSimulator.setupForROS2Color(pubSubImplementation, PerceptionAPI.BIG_VIDEO);
            baseUI.getImGuiPanelManager().addPanel(highLevelDepthSensorSimulator);
            highLevelDepthSensorSimulator.setSensorEnabled(true);
            highLevelDepthSensorSimulator.getLowLevelSimulator().setDepthEnabled(false);
            highLevelDepthSensorSimulator.setPublishPointCloudROS2(false);
            highLevelDepthSensorSimulator.setRenderPointCloudDirectly(false);
            highLevelDepthSensorSimulator.setPublishDepthImageROS1(false);
            highLevelDepthSensorSimulator.setDebugCoordinateFrame(false);
            highLevelDepthSensorSimulator.setRenderColorVideoDirectly(true);
            highLevelDepthSensorSimulator.setRenderDepthVideoDirectly(false);
            highLevelDepthSensorSimulator.setPublishColorImageROS1(false);
            highLevelDepthSensorSimulator.setPublishColorImageROS2(true);
            baseUI.getPrimaryScene().addRenderableProvider(highLevelDepthSensorSimulator::getRenderables);
         }

         @Override
         public void render()
         {
            highLevelDepthSensorSimulator.render(baseUI.getPrimaryScene());
            globalVisualizersPanel.update();

            for (RDXEnvironmentObject allObject : environmentBuilder.getAllObjects())
            {
               allObject.getRealisticModelInstance().setDiffuseColor(highLevelDepthSensorSimulator.getPointColorFromPicker());
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
            highLevelDepthSensorSimulator.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXROS2VideoSensorDemo();
   }
}
