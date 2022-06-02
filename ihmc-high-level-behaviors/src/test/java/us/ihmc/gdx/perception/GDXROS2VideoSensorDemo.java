package us.ihmc.gdx.perception;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.ui.graphics.live.GDXROS2BigVideoVisualizer;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXGlobalVisualizersPanel;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class GDXROS2VideoSensorDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   private GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private GDXEnvironmentBuilder environmentBuilder;

   public GDXROS2VideoSensorDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {

         private ImGuiGDXGlobalVisualizersPanel globalVisualizersPanel;

         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.get3DSceneManager());
            environmentBuilder.create(baseUI);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.get3DSceneManager().addRenderableProvider(environmentBuilder::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
            baseUI.get3DSceneManager().addRenderableProvider(environmentBuilder::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            environmentBuilder.loadEnvironment("DepthSensorZeroTest.json");

            sensorPoseGizmo.create(baseUI.get3DSceneManager().getCamera3D());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(sensorPoseGizmo, GDXSceneLevel.VIRTUAL);

            PubSubImplementation pubSubImplementation = PubSubImplementation.INTRAPROCESS;
            globalVisualizersPanel = new ImGuiGDXGlobalVisualizersPanel(false);
            GDXROS2BigVideoVisualizer videoVisualizer = new GDXROS2BigVideoVisualizer("Video",
                                                                                      pubSubImplementation,
                                                                                      ROS2Tools.BIG_VIDEO);
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
            highLevelDepthSensorSimulator = new GDXHighLevelDepthSensorSimulator("Stepping L515",
                                                                                 sensorPoseGizmo.getGizmoFrame(),
                                                                                 () -> 0L,
                                                                                 verticalFOV,
                                                                                 imageWidth,
                                                                                 imageHeight,
                                                                                 minRange,
                                                                                 maxRange,
                                                                                 publishRateHz);
            highLevelDepthSensorSimulator.setupForROS2Color(pubSubImplementation, ROS2Tools.BIG_VIDEO);
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
            baseUI.get3DSceneManager().addRenderableProvider(highLevelDepthSensorSimulator, GDXSceneLevel.VIRTUAL);
         }

         @Override
         public void render()
         {
            highLevelDepthSensorSimulator.render(baseUI.get3DSceneManager());
            globalVisualizersPanel.update();

            for (GDXEnvironmentObject allObject : environmentBuilder.getAllObjects())
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
      new GDXROS2VideoSensorDemo();
   }
}
