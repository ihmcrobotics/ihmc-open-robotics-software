package us.ihmc.gdx.perception;

import imgui.ImGui;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.simulation.sensors.GDXSimulatedSensorFactory;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.GDXInteractableReferenceFrame;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.visualizers.GDXHeightMapGraphic;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMap;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapParameters;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapUpdater;
import us.ihmc.tools.thread.Activator;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

public class GDXGPUHeightMapBodyPathPlanningDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private Activator nativesLoadedActivator;
   private GDXHighLevelDepthSensorSimulator ouster;
   private GDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private GDXPose3DGizmo ousterPoseGizmo = new GDXPose3DGizmo();
   private GDXEnvironmentBuilder environmentBuilder;
   private GDXHeightMapGraphic heightMapGraphic;
   private SimpleGPUHeightMapUpdater simpleGPUHeightMapUpdater;
   private RosMainNode ros1Node;
   private GDXPose3DGizmo heightMapPoseGizmo;

   public GDXGPUHeightMapBodyPathPlanningDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

            baseUI.create();

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.get3DSceneManager());
            environmentBuilder.create(baseUI);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.get3DSceneManager().addRenderableProvider(environmentBuilder::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
            baseUI.get3DSceneManager().addRenderableProvider(environmentBuilder::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            environmentBuilder.loadEnvironment("CurvingBlockPathForBodyPath.json");

            robotInteractableReferenceFrame = new GDXInteractableReferenceFrame();
            robotInteractableReferenceFrame.create(ReferenceFrame.getWorldFrame(), 0.15, baseUI.get3DSceneManager().getCamera3D());
            robotInteractableReferenceFrame.getTransformToParent().getTranslation().add(0.0, 0.0, 1.7);
            baseUI.addImGui3DViewInputProcessor(robotInteractableReferenceFrame::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(robotInteractableReferenceFrame::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            ousterPoseGizmo = new GDXPose3DGizmo(robotInteractableReferenceFrame.getRepresentativeReferenceFrame());
            ousterPoseGizmo.create(baseUI.get3DSceneManager().getCamera3D());
            ousterPoseGizmo.setResizeAutomatically(false);
            baseUI.addImGui3DViewPickCalculator(ousterPoseGizmo::calculate3DViewPick);
            baseUI.addImGui3DViewInputProcessor(ousterPoseGizmo::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(ousterPoseGizmo, GDXSceneLevel.VIRTUAL);
            ousterPoseGizmo.getTransformToParent().appendPitchRotation(Math.toRadians(60.0));

            ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "height_map_planning_demo_ui");
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  ouster = GDXSimulatedSensorFactory.createOusterLidar(ousterPoseGizmo.getGizmoFrame(), () -> 0L);
                  ouster.setupForROS1PointCloud(ros1Node, RosTools.OUSTER_POINT_CLOUD);
                  baseUI.getImGuiPanelManager().addPanel(ouster);
                  ouster.setSensorEnabled(true);
                  ouster.setRenderPointCloudDirectly(true);
                  ouster.setPublishPointCloudROS1(true);
                  baseUI.get3DSceneManager().addRenderableProvider(ouster, GDXSceneLevel.VIRTUAL);

                  heightMapPoseGizmo = new GDXPose3DGizmo();
                  heightMapPoseGizmo.create(baseUI.get3DSceneManager().getCamera3D());
                  heightMapPoseGizmo.getTransformToParent().getTranslation().set(1.7, 0.0, 0.0);
                  heightMapPoseGizmo.setResizeAutomatically(false);
                  baseUI.addImGui3DViewPickCalculator(heightMapPoseGizmo::calculate3DViewPick);
                  baseUI.addImGui3DViewInputProcessor(heightMapPoseGizmo::process3DViewInput);
                  baseUI.get3DSceneManager().addRenderableProvider(heightMapPoseGizmo, GDXSceneLevel.VIRTUAL);

                  SimpleGPUHeightMapParameters parameters = new SimpleGPUHeightMapParameters();
                  ImGuiStoredPropertySetTuner heightMapParameterTuner = new ImGuiStoredPropertySetTuner("Height Map Parameters");
                  heightMapParameterTuner.create(parameters, SimpleGPUHeightMapParameters.keys, () -> { });
                  baseUI.getImGuiPanelManager().addPanel(heightMapParameterTuner);
                  simpleGPUHeightMapUpdater = new SimpleGPUHeightMapUpdater(parameters);
                  simpleGPUHeightMapUpdater.create(ouster.getLowLevelSimulator().getImageWidth(),
                                                   ouster.getLowLevelSimulator().getImageHeight(),
                                                   ouster.getLowLevelSimulator().getMetersDepthFloatBuffer(),
                                                   ouster.getDepthCameraIntrinsics().getFx(),
                                                   ouster.getDepthCameraIntrinsics().getFy(),
                                                   ouster.getDepthCameraIntrinsics().getCx(),
                                                   ouster.getDepthCameraIntrinsics().getCy());

                  heightMapGraphic = new GDXHeightMapGraphic();
                  baseUI.get3DSceneManager().addRenderableProvider(heightMapGraphic, GDXSceneLevel.VIRTUAL);
                  baseUI.getImGuiPanelManager().addPanel("Height Map", this::renderHeightMapImGuiWidgets);

                  ros1Node.execute();
                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               ouster.render(baseUI.get3DSceneManager());
               RigidBodyTransform heightMapToWorld = heightMapPoseGizmo.getGizmoFrame().getTransformToWorldFrame();
               RigidBodyTransform sensorTransformToWorld = ousterPoseGizmo.getGizmoFrame().getTransformToWorldFrame();
               simpleGPUHeightMapUpdater.computeFromDepthMap(sensorTransformToWorld);
               SimpleGPUHeightMap heightMap = simpleGPUHeightMapUpdater.getHeightMap();

               heightMapGraphic.getTransformToWorld().set(new RigidBodyTransform());
               heightMapGraphic.generateMeshesAsync(heightMap.buildMessage());
               heightMapGraphic.update();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderHeightMapImGuiWidgets()
         {
            ImGui.checkbox(labels.get("Render ground plane"), heightMapGraphic.getRenderGroundPlane());
         }

         @Override
         public void dispose()
         {
            ros1Node.shutdown();
            environmentBuilder.destroy();
            ouster.dispose();
            simpleGPUHeightMapUpdater.destroy();
            heightMapGraphic.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXGPUHeightMapBodyPathPlanningDemo();
   }
}
