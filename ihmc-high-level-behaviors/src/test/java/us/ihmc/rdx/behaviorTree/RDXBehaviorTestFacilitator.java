package us.ihmc.rdx.behaviorTree;


import com.badlogic.gdx.Gdx;
import controller_msgs.msg.dds.GoHomeMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import org.apache.commons.lang3.function.TriFunction;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTree;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeExecutor;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.detections.foundationPose.IsaacROSFoundationPoseCommunicatorMap;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionExecutor;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.foundationPose.RDXIsaacROSFoundationPoseVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.yolo.RDXROS2YOLOv8Visualizer;
import us.ihmc.rdx.ui.tools.RDXROS2StatsPanel;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2NodeBuilder.SpecialTransportMode;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.zed.global.zed;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

/** Includes RDX UI, RDX operated kinematics sim, SVO playback, no physics. */
public class RDXBehaviorTestFacilitator
{
   /** Disable perception if CUDA 12.9.1 is not installed or not working */
   private final boolean runPerception = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() && CUDATools.hasNVJPEG();
   private final String svoFile;
   /** Enables advanced mode where FoundationPose and this program to be running on a Jetson.
    *  Must run deployJetsonTestbed and run this process also on the Jetson. */
   private final boolean foundationPose = Boolean.parseBoolean(System.getProperty("foundationpose", "false"));
   /** If FOUNDATION_POSE mode is true, this boolean is true for the process on the Jetson and false for the local one. */
   private final boolean jetson = Boolean.parseBoolean(System.getProperty("jetson", "false"));
   private final Supplier<DRCRobotModel> robotModelBuilder;
   private final TriFunction<DRCRobotModel, ROS2NodeBuilder, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder;
   private final Supplier<RDXBaseUI> baseUIBuilder;
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final Function<DRCRobotModel, RobotCollisionModel> selectionCollisionModelBuilder;
   private final Supplier<ROS2NodeBuilder> ros2NodeBuilder;

   private HumanoidKinematicsSimulation kinematicsSimulation;
   private final Notification robotReady = new Notification();

   private ZEDSVOPlaybackSensor zedSensor;

   private ROS2BehaviorTreeExecutor behaviorTree;
   private Function<ROS2BehaviorTreeExecutor, Notification> behaviorTreeAccessorOneTime = null;
   private Consumer<ROS2BehaviorTreeExecutor> behaviorTreeAccessorEveryTick = null;
   private Runnable destroyBehaviorThread;

   private RDXROS2BehaviorTree behaviorTreeUI;
   private RDXRawImagePointCloudVisualizer pointCloudVisualizer;
   private final Notification uiIsReady = new Notification();
   private volatile boolean destroyRequested = false;

   private final ROS2ControllerHelper ros2ControllerHelper;

   public RDXBehaviorTestFacilitator(
         String svoFile,
         String robotName,
         Supplier<DRCRobotModel> robotModelBuilder,
         TriFunction<DRCRobotModel, ROS2NodeBuilder, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder,
         Supplier<RDXBaseUI> baseUIBuilder,
         WorkspaceResourceDirectory treeFilesDirectory,
         Function<DRCRobotModel, RobotCollisionModel> selectionCollisionModelBuilder
   )
   {
      this.svoFile = svoFile;
      this.robotModelBuilder = robotModelBuilder;
      this.kinematicsSimulationBuilder = kinematicsSimulationBuilder;
      this.baseUIBuilder = baseUIBuilder;
      this.treeFilesDirectory = treeFilesDirectory;
      this.selectionCollisionModelBuilder = selectionCollisionModelBuilder;

      ros2NodeBuilder = () ->
      {
         if (this.runPerception) // YOLO requires interprocess
            return new ROS2NodeBuilder();
         else
            return new ROS2NodeBuilder().specialTransportMode(SpecialTransportMode.INTRAPROCESS_ONLY);
      };

      if (!this.runPerception || !this.foundationPose || !this.jetson)
      {
         ThreadTools.startAThread(this::startSimulation, "StartSimulation");
         if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
            ThreadTools.startAThread(() -> ExceptionTools.handle(this::launchRDXUI, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE), "RDX");
      }
      if (!this.runPerception || !this.foundationPose || this.jetson)
         ThreadTools.startAThread(this::startBehaviorTree, "StartBehaviorTree");

      ros2ControllerHelper = new ROS2ControllerHelper(ros2NodeBuilder.get().build("facilitator"), robotName);
   }

   private void startSimulation()
   {
      Pose3D initialWalkingPose = new Pose3D();
      kinematicsSimulation = kinematicsSimulationBuilder.apply(robotModelBuilder.get(), ros2NodeBuilder.get(), initialWalkingPose);

      ThreadTools.startAsDaemon(() ->
      {
         YoRegistry registry = kinematicsSimulation.getYoRegistry();
         if (registry.findVariable("time") instanceof YoDouble time)
         {
            while (time.getValue() < 0.02)
            {
               ThreadTools.park(0.1);
            }
         }
         ThreadTools.park(1.0);
         robotReady.set();
         LogTools.info("Robot ready");
      }, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, "WaitForWalking");
   }

   private void startBehaviorTree()
   {
      ROS2Node ros2Node = ros2NodeBuilder.get().build("behavior_tree");
      DRCRobotModel robotModel = robotModelBuilder.get();
      ROS2ControllerHelper ros2 = new ROS2ControllerHelper(ros2Node, robotModel.getSimpleRobotName());
      ROS2SyncedRobotModel syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      ROS2PeerClockOffsetEstimator peerClockEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);

      IsaacROSFoundationPoseCommunicatorMap foundationPose;
      YOLOv8DetectionExecutor yolo;
      if (runPerception)
      {
         zedSensor = new ZEDSVOPlaybackSensor(0, ZEDModelData.ZED_2I, zed.SL_DEPTH_MODE_NEURAL_LIGHT, svoFile);
         zedSensor.setSensorFrame(syncedRobot.getReferenceFrames().getExperimentalCameraFrame());
         zedSensor.startSensor();

         foundationPose = new IsaacROSFoundationPoseCommunicatorMap(peerClockEstimator);

         yolo = new YOLOv8DetectionExecutor(ros2Node, peerClockEstimator, () -> true);
         yolo.enableModel("best_multi_02_17_2026");
         yolo.addDetectionConsumerCallback(foundationPose::updatePoseEstimations);
      }
      else
      {
         foundationPose = null;
         yolo = null;
      }

      behaviorTree = new ROS2BehaviorTreeExecutor(ros2, syncedRobot, zedSensor, yolo, foundationPose, null, peerClockEstimator);

      RepeatingTaskThread yoloThread = new RepeatingTaskThread("yolo", () ->
      {
         try
         {
            zedSensor.waitForGrab();
            RawImage colorImage = zedSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
            RawImage depthImage = zedSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
            if (colorImage.getPixelFormat() != PixelFormat.BGR8)
            {
               GpuMat bgrMat = new GpuMat();
               colorImage.getPixelFormat().convertToPixelFormat(colorImage.getGpuImageMat(), bgrMat, PixelFormat.BGR8);
               colorImage.release();
               colorImage = colorImage.replaceImage(bgrMat, PixelFormat.BGR8);
            }
            yolo.runModel(colorImage, depthImage);
            foundationPose.updateCommunicators();
            if (pointCloudVisualizer != null)
            {
               pointCloudVisualizer.setColorImage(colorImage);
               pointCloudVisualizer.setDepthImage(depthImage);
            }
            colorImage.release();
            depthImage.release();
         } catch (InterruptedException ignored) {}
      });
      if (yolo != null)
         yoloThread.startRepeating();

      RepeatingTaskThread thread = new RepeatingTaskThread("behavior_tree", () ->
      {
         syncedRobot.update();

         if (behaviorTreeAccessorOneTime != null)
         {
            behaviorTreeAccessorOneTime.apply(behaviorTree).set();
            behaviorTreeAccessorOneTime = null;
         }

         if (behaviorTreeAccessorEveryTick != null)
         {
            behaviorTreeAccessorEveryTick.accept(behaviorTree);
         }

         behaviorTree.update();
      });
      thread.setFrequencyLimit(ROS2BehaviorTree.SYNC_FREQUENCY);
      thread.setDaemon(true);
      thread.startRepeating();

      destroyBehaviorThread = () ->
      {
         try
         {
            if (foundationPose != null)
               foundationPose.closeCommunicators();
            if (yolo != null)
            {
               yoloThread.kill();
               yoloThread.interrupt();
               yolo.destroy();
            }
            if (zedSensor != null)
               zedSensor.close();
            kinematicsSimulation.destroy();
            thread.blockingKill();
            ros2Node.destroy();
            syncedRobot.destroy();
            behaviorTree.destroy();
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      };
   }

   private void launchRDXUI()
   {
      ROS2Node ros2Node = ros2NodeBuilder.get().build("behavior_ui");
      DRCRobotModel robotModel = robotModelBuilder.get();
      ROS2ControllerHelper ros2 = new ROS2ControllerHelper(ros2Node, robotModel.getSimpleRobotName());
      ROS2SyncedRobotModel syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      Throttler behaviorUpdateThrottler = new Throttler().setFrequency(ROS2BehaviorTree.SYNC_FREQUENCY);
      ROS2PeerClockOffsetEstimator peerClockEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
      RDXPerceptionVisualizersPanel visualizersPanel = new RDXPerceptionVisualizersPanel();

      RDXBaseUI baseUI = baseUIBuilder.get();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         private RDXROS2RobotVisualizer robotVisualizer;

         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().getSceneLevelsToRender().add(RDXSceneLevel.GROUND_TRUTH);
            baseUI.getPrimary3DPanel().getCamera3D().setCameraFocusPoint(new Point3D(0.7, 0.0, 0.4));
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(-3.0, -4.0, 4.0);

            robotVisualizer = new RDXROS2RobotVisualizer(ros2, syncedRobot);
            robotVisualizer.createAndSetupStandalone(baseUI);

            baseUI.getImGuiPanelManager().addPanel(new RDXROS2StatsPanel());

            var yoloVis = new RDXROS2YOLOv8Visualizer("YOLOv8", ros2Node, peerClockEstimator, PerceptionAPI.YOLO_ANNOTATED_IMAGE);
            yoloVis.setActive(true);
            visualizersPanel.addVisualizer(yoloVis);
            var fpVis = new RDXIsaacROSFoundationPoseVisualizer("FoundationPose", ros2Node, peerClockEstimator);
            fpVis.setActive(true);
            visualizersPanel.addVisualizer(fpVis);
            pointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud");
            pointCloudVisualizer.setActive(true);
            visualizersPanel.addVisualizer(pointCloudVisualizer);
            visualizersPanel.create(baseUI);
            baseUI.getImGuiPanelManager().addPanel(visualizersPanel);

            ImBoolean play = new ImBoolean(false);
            ImInt requestedPosition = new ImInt();
            ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
            baseUI.getImGuiPanelManager().addPanel("Facilitator", () ->
            {
               ImGui.beginDisabled(kinematicsSimulation == null);
               if (ImGui.button(labels.get("Restart Simulation")))
               {
                  kinematicsSimulation.destroy();
                  startSimulation();
               }
               ImGui.endDisabled();
               if (zedSensor != null)
               {
                  ImGui.sameLine();
                  if (ImGui.checkbox(labels.get("ZED Playback"), play) && zedSensor != null)
                  {
                     if (play.get())
                        zedSensor.play();
                     else
                        zedSensor.pause();
                  }
                  ImGui.beginDisabled(play.get());
                  int currentPosition = zedSensor.getCurrentPosition();
                  int zedLength = zedSensor.getLength();
                  if (play.get())
                     requestedPosition.set(currentPosition);
                  if (ImGuiTools.sliderInt(labels.get("Position"), requestedPosition, 0, Math.max(zedLength, 0)))
                  {
                     zedSensor.setCurrentPosition(requestedPosition.get() + zedSensor.getFps());
                     zedSensor.grabAndNotify();
                  }
                  ImGui.endDisabled();
               }
            });

            RobotCollisionModel selectionCollisionModel = selectionCollisionModelBuilder.apply(robotModel);
            selectionCollisionModel.setCollidableHelper(new CollidableHelper(), robotModel.getJointMap().getModelName(), "ground");
            behaviorTreeUI = new RDXROS2BehaviorTree(treeFilesDirectory,
                                                     syncedRobot,
                                                     peerClockEstimator,
                                                     selectionCollisionModel,
                                                     baseUI,
                                                     baseUI.getPrimary3DPanel(),
                                                     ros2);
            behaviorTreeUI.createAndSetupDefault(baseUI);
            RDXBehaviorTreeNode.logNotifications = runPerception && foundationPose; // Prevent duplicate entries
         }

         @Override
         public void render()
         {
            if (baseUI.getRenderIndex() > 10)
               uiIsReady.set();

            syncedRobot.update();
            robotVisualizer.update();
            visualizersPanel.update();

            if (behaviorUpdateThrottler.run())
               behaviorTreeUI.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();

            if (destroyRequested)
               Gdx.app.exit(); // FIXME: This is not working
         }

         @Override
         public void dispose()
         {
            visualizersPanel.destroy();
            robotVisualizer.destroy();
            behaviorTreeUI.destroy();
            ros2Node.destroy();
            baseUI.dispose();
         }
      });

   }

   public Notification accessBehaviorTreeOneTime(Consumer<ROS2BehaviorTreeExecutor> behaviorTreeAccessor)
   {
      Notification notification = new Notification();
      this.behaviorTreeAccessorOneTime = behaviorTree ->
      {
         behaviorTreeAccessor.accept(behaviorTree);
         notification.set();
         return notification;
      };
      return notification;
   }

   public void accessBehaviorTreeEveryTick(Consumer<ROS2BehaviorTreeExecutor> behaviorTreeAccessor)
   {
      this.behaviorTreeAccessorEveryTick = behaviorTreeAccessor;
   }

   public void waitForRobotReady()
   {
      robotReady.blockingPeek();
   }

   public void waitForUIReady()
   {
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         uiIsReady.blockingPeek();
   }

   public void homeRobot()
   {
      double trajectoryTime = 2.0;

      LogTools.info("Homing robot...");
      GoHomeMessage homeLeftArm = new GoHomeMessage();
      homeLeftArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeLeftArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_LEFT);
      homeLeftArm.setTrajectoryTime(trajectoryTime);
      ros2ControllerHelper.publishToController(homeLeftArm);

      GoHomeMessage homeRightArm = new GoHomeMessage();
      homeRightArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeRightArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_RIGHT);
      homeRightArm.setTrajectoryTime(trajectoryTime);
      ros2ControllerHelper.publishToController(homeRightArm);

      GoHomeMessage homePelvis = new GoHomeMessage();
      homePelvis.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_PELVIS);
      homePelvis.setTrajectoryTime(trajectoryTime);
      ros2ControllerHelper.publishToController(homePelvis);

      GoHomeMessage homeChest = new GoHomeMessage();
      homeChest.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_CHEST);
      homeChest.setTrajectoryTime(trajectoryTime);
      ros2ControllerHelper.publishToController(homeChest);

      ThreadTools.park(2.0);
   }

   public void destroy()
   {
      kinematicsSimulation.destroy();
      destroyBehaviorThread.run();

      destroyRequested = true;

      ros2ControllerHelper.getROS2Node().destroy();
   }
}
