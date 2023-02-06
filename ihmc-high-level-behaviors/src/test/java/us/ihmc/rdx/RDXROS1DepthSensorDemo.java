package us.ihmc.rdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.live.RDXROS1VideoVisualizer;
import us.ihmc.rdx.ui.visualizers.RDXGlobalVisualizersPanel;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

public class RDXROS1DepthSensorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass().getSimpleName());
   private RDXGlobalVisualizersPanel globalVisualizersUI;
   private RDXHighLevelDepthSensorSimulator l515;
   private final RDXPose3DGizmo poseGizmo = new RDXPose3DGizmo();

   public RDXROS1DepthSensorDemo()
   {
      RosMainNode ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "depth_sensor_demo");

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
            baseUI.getPrimaryScene().addModelInstance(new DepthSensorDemoObjectsModel().newInstance());

            poseGizmo.create(baseUI.getPrimary3DPanel());
            poseGizmo.setResizeAutomatically(false);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(poseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(poseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(poseGizmo, RDXSceneLevel.VIRTUAL);
            poseGizmo.getTransformToParent().appendTranslation(0.0, 0.0, 0.5);
            poseGizmo.getTransformToParent().appendPitchRotation(Math.PI / 6.0);

            double publishRateHz = 5.0;
            double verticalFOV = 55.0;
            int imageWidth = 640;
            int imageHeight = 480;
            double minRange = 0.105;
            double maxRange = 5.0;
            l515 = new RDXHighLevelDepthSensorSimulator("Stepping L515",
                                                        poseGizmo.getGizmoFrame(),
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
            l515.setupForROS1Depth(ros1Node, RosTools.MAPSENSE_DEPTH_IMAGE, RosTools.MAPSENSE_DEPTH_CAMERA_INFO);
            l515.setupForROS1Color(ros1Node, RosTools.L515_VIDEO, RosTools.L515_COLOR_CAMERA_INFO);
            globalVisualizersUI = new RDXGlobalVisualizersPanel();
            globalVisualizersUI.addVisualizer(new RDXROS1VideoVisualizer("L515 Depth Video", RosTools.L515_DEPTH));
            globalVisualizersUI.addVisualizer(new RDXROS1VideoVisualizer("L515 Color Video", RosTools.L515_VIDEO));


            baseUI.getImGuiPanelManager().addPanel(l515);
            baseUI.getImGuiPanelManager().addPanel(globalVisualizersUI);

            l515.setSensorEnabled(true);
            l515.setRenderPointCloudDirectly(true);
            l515.setPublishDepthImageROS1(true);
            l515.setDebugCoordinateFrame(true);
            l515.setRenderColorVideoDirectly(true);
            l515.setRenderDepthVideoDirectly(true);
            l515.setPublishColorImageROS1(true);

            baseUI.getPrimaryScene().addRenderableProvider(l515::getRenderables);
            globalVisualizersUI.create();

            ros1Node.execute();
         }

         @Override
         public void render()
         {
            globalVisualizersUI.update();
            l515.render(baseUI.getPrimaryScene());
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            l515.dispose();
            globalVisualizersUI.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXROS1DepthSensorDemo();
   }
}
