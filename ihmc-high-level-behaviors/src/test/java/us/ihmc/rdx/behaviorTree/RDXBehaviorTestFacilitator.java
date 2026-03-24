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
import us.ihmc.behaviors.behaviorTree.BehaviorTreeExecutorNodeBuilder;
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
import us.ihmc.sensors.ImageSensor;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.zed.global.zed;
import us.ihmc.zed.library.ZEDJavaAPINativeLibrary;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;

/** Includes RDX UI, RDX operated kinematics sim, SVO playback, no physics. */
public class RDXBehaviorTestFacilitator
{
   /** Disable perception if CUDA 12.9.1 is not installed or not working */
   private final boolean runPerception = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer() && CUDATools.hasNVJPEG() && ZEDJavaAPINativeLibrary.load();
   private static final Path DOWNLOADS_DIRECTORY = Paths.get(System.getProperty("user.home"), "Downloads");
   private final Supplier<DRCRobotModel> robotModelBuilder;
   private final TriFunction<DRCRobotModel, ROS2NodeBuilder, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder;
   private final Supplier<RDXBaseUI> baseUIBuilder;
   private final WorkspaceResourceDirectory treeFilesDirectory;
   private final Function<DRCRobotModel, RobotCollisionModel> selectionCollisionModelBuilder;
   private final Supplier<ROS2NodeBuilder> ros2NodeBuilder;

   private HumanoidKinematicsSimulation kinematicsSimulation;
   private final Notification robotReady = new Notification();

   private final Object zedSensorLock = new Object();
   private final Object svoFileListLock = new Object();
   private final List<Path> availableSvoFiles = new ArrayList<>();
   private String[] availableSvoFileNames = new String[0];
   private volatile String requestedSvoFile;
   private volatile String activeSvoFile;
   private volatile boolean zedPlaybackEnabled;

   private ZEDSVOPlaybackSensor zedSensor;
   private RepeatingTaskThread yoloThread;
   private ROS2SyncedRobotModel behaviorTreeSyncedRobot;

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
         String robotName,
         Supplier<DRCRobotModel> robotModelBuilder,
         TriFunction<DRCRobotModel, ROS2NodeBuilder, RigidBodyTransformReadOnly, HumanoidKinematicsSimulation> kinematicsSimulationBuilder,
         Supplier<RDXBaseUI> baseUIBuilder,
         WorkspaceResourceDirectory treeFilesDirectory,
         Function<DRCRobotModel, RobotCollisionModel> selectionCollisionModelBuilder
   )
   {
      this.robotModelBuilder = robotModelBuilder;
      this.kinematicsSimulationBuilder = kinematicsSimulationBuilder;
      this.baseUIBuilder = baseUIBuilder;
      this.treeFilesDirectory = treeFilesDirectory;
      this.selectionCollisionModelBuilder = selectionCollisionModelBuilder;

      refreshSvoFiles();
      if (!availableSvoFiles.isEmpty())
         requestedSvoFile = availableSvoFiles.get(0).toString();

      ros2NodeBuilder = () ->
      {
         if (this.runPerception) // YOLO requires interprocess
            return new ROS2NodeBuilder();
         else
            return new ROS2NodeBuilder().specialTransportMode(SpecialTransportMode.INTRAPROCESS_ONLY);
      };

      ThreadTools.startAThread(this::startSimulation, "StartSimulation");
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
         ThreadTools.startAThread(() -> ExceptionTools.handle(this::launchRDXUI, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE), "RDX");
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
      behaviorTreeSyncedRobot = syncedRobot;

      IsaacROSFoundationPoseCommunicatorMap foundationPose;
      YOLOv8DetectionExecutor yolo;
      if (runPerception)
      {
         String initialSvoFile = requestedSvoFile;
         if (initialSvoFile != null)
         {
            try
            {
               zedSensor = createZedSensor(initialSvoFile);
               activeSvoFile = initialSvoFile;
            }
            catch (Exception e)
            {
               LogTools.error("Failed to open initial SVO file: {}", initialSvoFile, e);
            }
         }

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

      behaviorTree = new ROS2BehaviorTreeExecutor(ros2, syncedRobot, kinematicsSimulationBuilder, zedSensor, yolo, foundationPose, null, peerClockEstimator);

      if (yolo != null)
      {
         yoloThread = new RepeatingTaskThread("yolo", () ->
         {
            try
            {
               ZEDSVOPlaybackSensor localZedSensor = getZedSensorSnapshot();
               if (localZedSensor == null)
               {
                  ThreadTools.park(0.1);
                  return;
               }
               localZedSensor.waitForGrab();
               RawImage colorImage = localZedSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
               RawImage depthImage = localZedSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
               if (colorImage == null || depthImage == null)
               {
                  if (colorImage != null)
                     colorImage.release();
                  if (depthImage != null)
                     depthImage.release();
                  return;
               }
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
         yoloThread.startRepeating();
      }
      else
      {
         yoloThread = null;
      }

      RepeatingTaskThread thread = new RepeatingTaskThread("behavior_tree", () ->
      {
         syncedRobot.update();
         performPendingSvoSwitchIfRequested();

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
            closeZedSensor();
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

   private void performPendingSvoSwitchIfRequested()
   {
      if (!runPerception)
         return;

      String requested = requestedSvoFile;
      if (requested == null)
         return;

      if (requested.equals(activeSvoFile))
      {
         requestedSvoFile = null;
         return;
      }

      if (!Files.exists(Paths.get(requested)))
      {
         LogTools.error("Requested SVO file does not exist: {}", requested);
         requestedSvoFile = null;
         return;
      }

      switchZedSensor(requested);
      requestedSvoFile = null;
   }

   private boolean switchZedSensor(String newSvoFile)
   {
      LogTools.info("Switching ZED SVO to {}", newSvoFile);
      if (yoloThread != null)
         yoloThread.interrupt();

      closeZedSensor();

      ZEDSVOPlaybackSensor newSensor;
      try
      {
         newSensor = createZedSensor(newSvoFile);
      }
      catch (Exception e)
      {
         LogTools.error("Failed to open SVO file: {}", newSvoFile, e);
         return false;
      }

      synchronized (zedSensorLock)
      {
         zedSensor = newSensor;
      }
      activeSvoFile = newSvoFile;
      updateBehaviorTreeImageSensor(newSensor);
      return true;
   }

   private void updateBehaviorTreeImageSensor(ImageSensor newSensor)
   {
      if (behaviorTree == null)
         return;

      if (behaviorTree.getRootNode() != null)
         behaviorTree.getRootNode().getScene().setImageSensor(newSensor);

      if (behaviorTree.getNodeBuilder() instanceof BehaviorTreeExecutorNodeBuilder nodeBuilder)
         nodeBuilder.setImageSensor(newSensor);
   }

   private ZEDSVOPlaybackSensor createZedSensor(String svoFile)
   {
      ZEDSVOPlaybackSensor newSensor = new ZEDSVOPlaybackSensor(0, ZEDModelData.ZED_X_MINI, zed.SL_DEPTH_MODE_NEURAL_LIGHT, svoFile);
      if (behaviorTreeSyncedRobot != null)
         newSensor.setSensorFrame(behaviorTreeSyncedRobot.getReferenceFrames().getExperimentalCameraFrame());
      newSensor.startSensor();
      if (zedPlaybackEnabled)
         newSensor.play();
      return newSensor;
   }

   private void closeZedSensor()
   {
      ZEDSVOPlaybackSensor toClose;
      synchronized (zedSensorLock)
      {
         toClose = zedSensor;
         zedSensor = null;
      }
      activeSvoFile = null;
      updateBehaviorTreeImageSensor(null);
      if (toClose != null)
         toClose.close();
   }

   private ZEDSVOPlaybackSensor getZedSensorSnapshot()
   {
      synchronized (zedSensorLock)
      {
         return zedSensor;
      }
   }

   private void requestSvoSwitch(String newSvoFile)
   {
      if (newSvoFile == null || newSvoFile.isBlank())
         return;

      requestedSvoFile = newSvoFile;
   }

   private void refreshSvoFiles()
   {
      List<Path> files = findSvoFilesInDownloads();
      synchronized (svoFileListLock)
      {
         availableSvoFiles.clear();
         availableSvoFiles.addAll(files);
         availableSvoFileNames = files.stream()
                                      .map(path -> path.getFileName().toString())
                                      .toArray(String[]::new);
      }
   }

   private List<Path> findSvoFilesInDownloads()
   {
      if (!Files.isDirectory(DOWNLOADS_DIRECTORY))
         return Collections.emptyList();

      try (java.util.stream.Stream<Path> stream = Files.list(DOWNLOADS_DIRECTORY))
      {
         return stream.filter(Files::isRegularFile)
                      .filter(path -> path.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".svo2"))
                      .sorted(Comparator.comparing(path -> path.getFileName().toString().toLowerCase(Locale.ROOT)))
                      .collect(Collectors.toList());
      }
      catch (Exception e)
      {
         LogTools.error("Failed to list .svo2 files in {}", DOWNLOADS_DIRECTORY, e);
         return Collections.emptyList();
      }
   }

   private String[] getAvailableSvoFileNamesSnapshot()
   {
      synchronized (svoFileListLock)
      {
         return availableSvoFileNames.clone();
      }
   }

   private Path getAvailableSvoFileAt(int index)
   {
      synchronized (svoFileListLock)
      {
         if (index < 0 || index >= availableSvoFiles.size())
            return null;
         return availableSvoFiles.get(index);
      }
   }

   private void syncSelectedSvoIndex(ImInt selectedSvoIndex)
   {
      String target = activeSvoFile != null ? activeSvoFile : requestedSvoFile;
      if (target != null)
      {
         int index = getIndexForSvoFile(target);
         if (index >= 0)
         {
            selectedSvoIndex.set(index);
            return;
         }
      }

      if (availableSvoFileNames.length == 0)
      {
         selectedSvoIndex.set(-1);
         return;
      }

      if (selectedSvoIndex.get() < 0 || selectedSvoIndex.get() >= availableSvoFileNames.length)
         selectedSvoIndex.set(0);
   }

   private int getIndexForSvoFile(String filePath)
   {
      synchronized (svoFileListLock)
      {
         for (int i = 0; i < availableSvoFiles.size(); i++)
         {
            if (availableSvoFiles.get(i).toString().equals(filePath))
               return i;
         }
      }
      return -1;
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
            ImInt selectedSvoIndex = new ImInt(-1);
            Throttler svoFileRefreshThrottler = new Throttler().setFrequency(1.0);
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
               if (svoFileRefreshThrottler.run())
                  refreshSvoFiles();

               if (!runPerception)
               {
                  ImGui.text("Perception disabled (CUDA/ZED not available).");
                  return;
               }

               String[] availableSvoNames = getAvailableSvoFileNamesSnapshot();
               syncSelectedSvoIndex(selectedSvoIndex);
               if (availableSvoNames.length == 0)
               {
                  ImGui.text("No .svo2 files found in Downloads.");
               }
               else
               {
                  if (ImGui.combo(labels.get("SVO File"), selectedSvoIndex, availableSvoNames))
                  {
                     Path selectedPath = getAvailableSvoFileAt(selectedSvoIndex.get());
                     if (selectedPath != null)
                     {
                        requestSvoSwitch(selectedPath.toString());
                        requestedPosition.set(0);
                     }
                  }
                  Path selectedPath = getAvailableSvoFileAt(selectedSvoIndex.get());
                  if (selectedPath != null && ImGui.isItemHovered())
                     ImGui.setTooltip(selectedPath.toString());
               }

               String activeSvoSnapshot = activeSvoFile;
               if (activeSvoSnapshot != null)
                  ImGui.text("Active: " + Paths.get(activeSvoSnapshot).getFileName());

               String pendingSvoSnapshot = requestedSvoFile;
               if (pendingSvoSnapshot != null && !pendingSvoSnapshot.equals(activeSvoSnapshot))
                  ImGui.text("Pending: " + Paths.get(pendingSvoSnapshot).getFileName());

               ZEDSVOPlaybackSensor localZedSensor = getZedSensorSnapshot();
               ImGui.beginDisabled(localZedSensor == null);
               play.set(zedPlaybackEnabled);
               if (ImGui.checkbox(labels.get("ZED Playback"), play))
               {
                  zedPlaybackEnabled = play.get();
                  if (localZedSensor != null)
                  {
                     if (zedPlaybackEnabled)
                        localZedSensor.play();
                     else
                        localZedSensor.pause();
                  }
               }
               ImGui.beginDisabled(play.get());
               if (localZedSensor != null)
               {
                  int currentPosition = localZedSensor.getCurrentPosition();
                  int zedLength = localZedSensor.getLength();
                  if (play.get())
                     requestedPosition.set(currentPosition);
                  if (ImGuiTools.sliderInt(labels.get("Position"), requestedPosition, 0, Math.max(zedLength, 0)))
                  {
                     localZedSensor.setCurrentPosition(requestedPosition.get() + localZedSensor.getFps());
                     localZedSensor.grabAndNotify();
                  }
               }
               ImGui.endDisabled();
               ImGui.endDisabled();
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
