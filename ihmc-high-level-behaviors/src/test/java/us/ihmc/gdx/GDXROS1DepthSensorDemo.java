package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.ui.graphics.live.GDXROS1VideoVisualizer;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXGlobalVisualizersPanel;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

public class GDXROS1DepthSensorDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              getClass().getSimpleName());
   private ImGuiGDXGlobalVisualizersPanel globalVisualizersUI;
   private GDXHighLevelDepthSensorSimulator l515;
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();

   public GDXROS1DepthSensorDemo()
   {
      RosMainNode ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "depth_sensor_demo");

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(GDXModelBuilder.createCoordinateFrame(0.3)));
            baseUI.getPrimaryScene().addModelInstance(new DepthSensorDemoObjectsModel().newInstance());

            poseGizmo.create(baseUI.getPrimary3DPanel());
            poseGizmo.setResizeAutomatically(false);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(poseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(poseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(poseGizmo, GDXSceneLevel.VIRTUAL);
            poseGizmo.getTransformToParent().appendTranslation(0.0, 0.0, 0.5);
            poseGizmo.getTransformToParent().appendPitchRotation(Math.PI / 6.0);

            double publishRateHz = 5.0;
            double verticalFOV = 55.0;
            int imageWidth = 640;
            int imageHeight = 480;
            double minRange = 0.105;
            double maxRange = 5.0;
            l515 = new GDXHighLevelDepthSensorSimulator("Stepping L515",
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
            globalVisualizersUI = new ImGuiGDXGlobalVisualizersPanel();
            globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("L515 Depth Video", RosTools.L515_DEPTH));
            globalVisualizersUI.addVisualizer(new GDXROS1VideoVisualizer("L515 Color Video", RosTools.L515_VIDEO));


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
      new GDXROS1DepthSensorDemo();
   }
}
