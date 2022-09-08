package us.ihmc.gdx.perception;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.ui.graphics.GDX3DSituatedImagePanel;
import us.ihmc.gdx.ui.graphics.live.GDXROS2BigVideoVisualizer;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXGlobalVisualizersPanel;
import us.ihmc.pubsub.DomainFactory;

public class GDXARDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private GDXEnvironmentBuilder environmentBuilder;
   private ImGuiGDXGlobalVisualizersPanel globalVisualizersPanel;
   private GDX3DPanel arPanel;
   private GDX3DSituatedImagePanel situatedImage3DPanel;

   public GDXARDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.getPrimaryScene().getSceneLevelsToRender().remove(GDXSceneLevel.MODEL);
            environmentBuilder.loadEnvironment("LookAndStepHard.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, GDXSceneLevel.VIRTUAL);

            DomainFactory.PubSubImplementation pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
            globalVisualizersPanel = new ImGuiGDXGlobalVisualizersPanel(false);
            GDXROS2BigVideoVisualizer videoVisualizer = new GDXROS2BigVideoVisualizer("Video", pubSubImplementation, ROS2Tools.BIG_VIDEO);
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
                                                                                 0.03,
                                                                                 0.05,
                                                                                 true,
                                                                                 publishRateHz);
            highLevelDepthSensorSimulator.setupForROS2Color(pubSubImplementation, ROS2Tools.BIG_VIDEO);
            baseUI.getImGuiPanelManager().addPanel(highLevelDepthSensorSimulator);
            highLevelDepthSensorSimulator.setSensorEnabled(true);
            highLevelDepthSensorSimulator.setPublishPointCloudROS2(false);
            highLevelDepthSensorSimulator.setRenderPointCloudDirectly(true);
            highLevelDepthSensorSimulator.setPublishDepthImageROS1(false);
            highLevelDepthSensorSimulator.setDebugCoordinateFrame(false);
            highLevelDepthSensorSimulator.setRenderColorVideoDirectly(true);
            highLevelDepthSensorSimulator.setRenderDepthVideoDirectly(false);
            highLevelDepthSensorSimulator.setPublishColorImageROS1(false);
            highLevelDepthSensorSimulator.setPublishColorImageROS2(true);
            highLevelDepthSensorSimulator.setUseSensorColor(true);
            baseUI.getPrimaryScene().addRenderableProvider(highLevelDepthSensorSimulator::getRenderables);

            highLevelDepthSensorSimulator.getLowLevelSimulator().getCamera().update();

            situatedImage3DPanel = new GDX3DSituatedImagePanel();

            baseUI.getPrimaryScene().addRenderableProvider(situatedImage3DPanel::getRenderables, GDXSceneLevel.VIRTUAL);

            sensorPoseGizmo.getTransformToParent().getTranslation().set(0.2, 0.0, 1.0);
            sensorPoseGizmo.getTransformToParent().getRotation().setToPitchOrientation(Math.toRadians(45.0));

            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            // TODO: Make a new scene for this panel
            arPanel = new GDX3DPanel("AR View", 2, false);

            baseUI.add3DPanel(arPanel);
            arPanel.getCamera3D().setInputEnabled(false);
            arPanel.getCamera3D().setVerticalFieldOfView(verticalFOV);
         }

         @Override
         public void render()
         {
            boolean flipY = true;
            situatedImage3DPanel.create(highLevelDepthSensorSimulator.getLowLevelSimulator().getFrameBufferColorTexture(),
                                        highLevelDepthSensorSimulator.getLowLevelSimulator().getCamera().frustum,
                                        highLevelDepthSensorSimulator.getSensorFrame(),
                                        flipY);

            arPanel.getCamera3D().setPose(highLevelDepthSensorSimulator.getSensorFrame().getTransformToWorldFrame());
            highLevelDepthSensorSimulator.render(baseUI.getPrimaryScene());
            globalVisualizersPanel.update();

            situatedImage3DPanel.setPoseToReferenceFrame(highLevelDepthSensorSimulator.getSensorFrame());

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
      new GDXARDemo();
   }
}
