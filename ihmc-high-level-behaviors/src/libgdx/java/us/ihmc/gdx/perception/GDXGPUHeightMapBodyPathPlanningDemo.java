package us.ihmc.gdx.perception;

import controller_msgs.msg.dds.HeightMapMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.bodyPath.AStarBodyPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.simulation.sensors.GDXSimulatedSensorFactory;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.GDXInteractableReferenceFrame;
import us.ihmc.gdx.ui.affordances.GDXSelectablePose3DGizmo;
import us.ihmc.gdx.ui.graphics.GDXBodyPathPlanGraphic;
import us.ihmc.gdx.visualizers.GDXHeightMapGraphic;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMap;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapParameters;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapUpdater;
import us.ihmc.robotics.heightMap.HeightMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.Activator;
import us.ihmc.utilities.ros.RosMainNode;

import java.util.List;

public class GDXGPUHeightMapBodyPathPlanningDemo
{
   private final GDXImGuiBasedUI baseUI;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private Activator nativesLoadedActivator;
   private GDXHighLevelDepthSensorSimulator ouster;
   private GDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private GDXSelectablePose3DGizmo ousterPoseGizmo;
   private GDXEnvironmentBuilder environmentBuilder;
   private GDXHeightMapGraphic heightMapGraphic;
   private SimpleGPUHeightMapParameters simpleGPUHeightMapParameters;
   private SimpleGPUHeightMapUpdater simpleGPUHeightMapUpdater;
   private RosMainNode ros1Node;
   private GDXSelectablePose3DGizmo heightMapPoseGizmo;
   private final Stopwatch bodyPathPlannerStopwatch = new Stopwatch();
   private AStarBodyPathPlanner bodyPathPlanner;
   private GDXBodyPathPlanGraphic bodyPathPlanGraphic;
   private boolean planBodyPath = false;
   private GDXSelectablePose3DGizmo startPoseGizmo;
   private GDXSelectablePose3DGizmo goalPoseGizmo;
   private final FramePose3D startFramePose = new FramePose3D();
   private final FramePose3D goalFramePose = new FramePose3D();
   private final ImBoolean updateHeightMap = new ImBoolean(true);
   private HeightMapMessage heightMapMessage;

   public GDXGPUHeightMapBodyPathPlanningDemo(GDXImGuiBasedUI baseUI, DRCRobotModel robotModel)
   {
      this.baseUI = baseUI;
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
            baseUI.create();

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("CurvingBlockPathForBodyPath.json");

            robotInteractableReferenceFrame = new GDXInteractableReferenceFrame();
            robotInteractableReferenceFrame.create(ReferenceFrame.getWorldFrame(), 0.15, baseUI.getPrimary3DPanel());
            robotInteractableReferenceFrame.getTransformToParent().getTranslation().add(0.0, 0.0, 1.7);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(robotInteractableReferenceFrame::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(robotInteractableReferenceFrame::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            ousterPoseGizmo = new GDXSelectablePose3DGizmo(robotInteractableReferenceFrame.getRepresentativeReferenceFrame());
            ousterPoseGizmo.create(baseUI.getPrimary3DPanel());
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(ousterPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(input -> ousterPoseGizmo.process3DViewInput(input));
            baseUI.getPrimaryScene().addRenderableProvider(ousterPoseGizmo::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            ousterPoseGizmo.getPoseGizmo().getTransformToParent().appendPitchRotation(Math.toRadians(60.0));

//            ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "height_map_planning_demo_ui");
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  ouster = GDXSimulatedSensorFactory.createOusterLidar(ousterPoseGizmo.getPoseGizmo().getGizmoFrame(), () -> 0L);
//                  ouster.setupForROS1PointCloud(ros1Node, RosTools.OUSTER_POINT_CLOUD);
                  baseUI.getImGuiPanelManager().addPanel(ouster);
                  ouster.setSensorEnabled(true);
                  ouster.setRenderPointCloudDirectly(true);
//                  ouster.setPublishPointCloudROS1(true);
                  baseUI.getPrimaryScene().addRenderableProvider(ouster::getRenderables);

                  heightMapPoseGizmo = new GDXSelectablePose3DGizmo();
                  heightMapPoseGizmo.create(baseUI.getPrimary3DPanel());
                  heightMapPoseGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(1.7, 0.0, 0.0);
                  baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(heightMapPoseGizmo::calculate3DViewPick);
                  baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(input -> heightMapPoseGizmo.process3DViewInput(input));
                  baseUI.getPrimaryScene().addRenderableProvider(heightMapPoseGizmo::getVirtualRenderables, GDXSceneLevel.VIRTUAL);

                  simpleGPUHeightMapParameters = new SimpleGPUHeightMapParameters();
                  ImGuiStoredPropertySetTuner heightMapParameterTuner = new ImGuiStoredPropertySetTuner("Height Map Parameters");
                  heightMapParameterTuner.create(simpleGPUHeightMapParameters, SimpleGPUHeightMapParameters.keys, () -> { });
                  baseUI.getImGuiPanelManager().addPanel(heightMapParameterTuner);
                  simpleGPUHeightMapUpdater = new SimpleGPUHeightMapUpdater(simpleGPUHeightMapParameters);
                  simpleGPUHeightMapUpdater.create(ouster.getLowLevelSimulator().getImageWidth(),
                                                   ouster.getLowLevelSimulator().getImageHeight(),
                                                   ouster.getLowLevelSimulator().getMetersDepthFloatBuffer(),
                                                   ouster.getDepthCameraIntrinsics().getFx(),
                                                   ouster.getDepthCameraIntrinsics().getFy(),
                                                   ouster.getDepthCameraIntrinsics().getCx(),
                                                   ouster.getDepthCameraIntrinsics().getCy());

                  heightMapGraphic = new GDXHeightMapGraphic();
                  heightMapGraphic.getRenderGroundPlane().set(false);
                  baseUI.getPrimaryScene().addRenderableProvider(heightMapGraphic, GDXSceneLevel.VIRTUAL);
                  baseUI.getImGuiPanelManager().addPanel("Height Map", this::renderHeightMapImGuiWidgets);

                  FootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
                  SideDependentList<ConvexPolygon2D> footPolygons = FootstepPlanningModuleLauncher.createFootPolygons(robotModel);
                  bodyPathPlanner = new AStarBodyPathPlanner(footstepPlannerParameters, footPolygons, bodyPathPlannerStopwatch);

                  bodyPathPlanGraphic = new GDXBodyPathPlanGraphic();
                  baseUI.getPrimaryScene().addRenderableProvider(bodyPathPlanGraphic, GDXSceneLevel.VIRTUAL);

                  startPoseGizmo = new GDXSelectablePose3DGizmo();
                  startPoseGizmo.create(baseUI.getPrimary3DPanel());
                  startPoseGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(0.0, 0.0, 0.0);
                  baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(startPoseGizmo::calculate3DViewPick);
                  baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(input -> startPoseGizmo.process3DViewInput(input));
                  baseUI.getPrimaryScene().addRenderableProvider(startPoseGizmo::getVirtualRenderables, GDXSceneLevel.VIRTUAL);

                  goalPoseGizmo = new GDXSelectablePose3DGizmo();
                  goalPoseGizmo.create(baseUI.getPrimary3DPanel());
                  goalPoseGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(1.839, -1.142, 0.0);
                  goalPoseGizmo.getPoseGizmo().getTransformToParent().getRotation().appendYawRotation(-1.3767390862107274);
                  baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(goalPoseGizmo::calculate3DViewPick);
                  baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(input -> goalPoseGizmo.process3DViewInput(input));
                  baseUI.getPrimaryScene().addRenderableProvider(goalPoseGizmo::getVirtualRenderables, GDXSceneLevel.VIRTUAL);

                  //                  ros1Node.execute();
                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               ouster.render(baseUI.getPrimaryScene());

               if (updateHeightMap.get())
               {
                  RigidBodyTransform heightMapToWorld = heightMapPoseGizmo.getPoseGizmo().getGizmoFrame().getTransformToWorldFrame();
                  RigidBodyTransform sensorTransformToWorld = ousterPoseGizmo.getPoseGizmo().getGizmoFrame().getTransformToWorldFrame();
                  simpleGPUHeightMapUpdater.computeFromDepthMap((float) heightMapToWorld.getTranslationX(),
                                                                (float) heightMapToWorld.getTranslationY(),
                                                                sensorTransformToWorld);
                  SimpleGPUHeightMap heightMap = simpleGPUHeightMapUpdater.getHeightMap();

                  heightMapGraphic.getTransformToWorld().set(new RigidBodyTransform());
                  heightMapMessage = heightMap.buildMessage();
                  heightMapGraphic.generateMeshesAsync(heightMapMessage);
                  heightMapGraphic.update();
               }

               if (planBodyPath)
               {
                  planBodyPath = false;

                  HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
                  heightMapData.setEstimatedGroundHeight(-1.0);

                  bodyPathPlanner.clearLoggedData();
                  bodyPathPlanner.setHeightMapData(heightMapData);
                  FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
                  startFramePose.setToZero(startPoseGizmo.getPoseGizmo().getGizmoFrame());
                  startFramePose.getPosition().setY(0.2);
                  startFramePose.changeFrame(ReferenceFrame.getWorldFrame());
                  footstepPlannerRequest.getStartFootPoses().put(RobotSide.LEFT, new Pose3D(startFramePose));
                  startFramePose.setToZero(startPoseGizmo.getPoseGizmo().getGizmoFrame());
                  startFramePose.getPosition().setY(-0.2);
                  startFramePose.changeFrame(ReferenceFrame.getWorldFrame());
                  footstepPlannerRequest.getStartFootPoses().put(RobotSide.RIGHT, new Pose3D(startFramePose));
                  goalFramePose.setToZero(goalPoseGizmo.getPoseGizmo().getGizmoFrame());
                  goalFramePose.changeFrame(ReferenceFrame.getWorldFrame());
                  LogTools.info(StringTools.tupleString(goalFramePose.getPosition()));
                  LogTools.info("Yaw: {}", goalFramePose.getOrientation().getYaw());
                  goalFramePose.setToZero(goalPoseGizmo.getPoseGizmo().getGizmoFrame());
                  goalFramePose.getPosition().setY(0.2);
                  goalFramePose.changeFrame(ReferenceFrame.getWorldFrame());
                  footstepPlannerRequest.getGoalFootPoses().put(RobotSide.LEFT, new Pose3D(goalFramePose));
                  goalFramePose.setToZero(goalPoseGizmo.getPoseGizmo().getGizmoFrame());
                  goalFramePose.getPosition().setY(-0.2);
                  goalFramePose.changeFrame(ReferenceFrame.getWorldFrame());
                  footstepPlannerRequest.getGoalFootPoses().put(RobotSide.RIGHT, new Pose3D(goalFramePose));
                  footstepPlannerRequest.setTimeout(10.0);
//                  FootstepPlannerOutput footstepPlannerOutput = new FootstepPlannerOutput();
//                  bodyPathPlanner.handleRequest(footstepPlannerRequest, footstepPlannerOutput);
//                  List<Pose3D> bodyPathWaypoints = footstepPlannerOutput.getBodyPath();
//
//                  bodyPathPlanGraphic.generateMeshes(bodyPathWaypoints);
//                  bodyPathPlanGraphic.update();

                  FootstepPlanningModule footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(robotModel);
                  heightMapMessage.setEstimatedGroundHeight(-1.0);
                  footstepPlannerRequest.setHeightMapMessage(heightMapMessage);
                  footstepPlannerRequest.setPlanBodyPath(true);
                  footstepPlanningModule.handleRequest(footstepPlannerRequest);
                  FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
                  footstepPlannerLogger.logSession();

                  List<Pose3D> bodyPathWaypoints = footstepPlanningModule.getOutput().getBodyPath();

                  bodyPathPlanGraphic.generateMeshes(bodyPathWaypoints);
                  bodyPathPlanGraphic.update();
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderHeightMapImGuiWidgets()
         {
            ImGui.checkbox(labels.get("Update height map"), updateHeightMap);
            ImGui.checkbox(labels.get("Ouster Gizmo"), ousterPoseGizmo.getSelected());
            ImGui.checkbox(labels.get("Height Map Gizmo"), heightMapPoseGizmo.getSelected());
            ImGui.checkbox(labels.get("Start Pose Gizmo"), startPoseGizmo.getSelected());
            ImGui.checkbox(labels.get("Goal Pose Gizmo"), goalPoseGizmo.getSelected());
            ImGui.checkbox(labels.get("Render ground plane"), heightMapGraphic.getRenderGroundPlane());
            if (ImGui.button(labels.get("Plan body path")))
            {
               planBodyPath = true;
            }
         }

         @Override
         public void dispose()
         {
            bodyPathPlanner.halt();
            ros1Node.shutdown();
            environmentBuilder.destroy();
            bodyPathPlanGraphic.destroy();
            ouster.dispose();
            simpleGPUHeightMapUpdater.destroy();
            heightMapGraphic.destroy();
            baseUI.dispose();
         }
      });
   }
}
