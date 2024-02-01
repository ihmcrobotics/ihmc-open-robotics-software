package us.ihmc.rdx.perception;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDX3DSituatedImagePanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2BigVideoVisualizer;
import us.ihmc.rdx.ui.graphics.RDXGlobalVisualizersPanel;
import us.ihmc.pubsub.DomainFactory;

public class RDXARDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
   private RDXEnvironmentBuilder environmentBuilder;
   private RDXGlobalVisualizersPanel globalVisualizersPanel;
   private RDX3DPanel arPanel;
   private RDX3DSituatedImagePanel situatedImage3DPanel;

   public RDXARDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.getPrimaryScene().getSceneLevelsToRender().remove(RDXSceneLevel.MODEL);
            environmentBuilder.loadEnvironment("LookAndStepHard.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);

            DomainFactory.PubSubImplementation pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
            globalVisualizersPanel = new RDXGlobalVisualizersPanel();

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

            situatedImage3DPanel = new RDX3DSituatedImagePanel();

            baseUI.getPrimaryScene().addRenderableProvider(situatedImage3DPanel::getRenderables);

            sensorPoseGizmo.getTransformToParent().getTranslation().set(0.2, 0.0, 1.0);
            sensorPoseGizmo.getTransformToParent().getRotation().setToPitchOrientation(Math.toRadians(45.0));

            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            // TODO: Make a new scene for this panel
            arPanel = new RDX3DPanel("AR View", false);

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
      new RDXARDemo();
   }
}
