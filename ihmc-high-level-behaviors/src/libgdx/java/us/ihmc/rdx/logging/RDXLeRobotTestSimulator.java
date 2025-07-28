package us.ihmc.rdx.logging;

import gnu.trove.map.TObjectDoubleMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDataset;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDatasetDataWriter;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDatasetTools;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.perception.lerobot.LeRobotInferenceManager;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.simulation.scs2.RDXSCS2LogSession;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.yoVariables.euclid.YoPose3D;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.nio.file.Files;
import java.util.function.Function;
import java.util.function.Supplier;

import static us.ihmc.zed.global.zed.*;
import static us.ihmc.zed.global.zed.SL_MEM_CPU;

/**
 * This helps test the trained visuomotor policy models by running the log data
 * through the model and viewing the output.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 * <p>
 * TODO: Pass the output actions through the IK preview for a more complete visualization.
 */
public class RDXLeRobotTestSimulator
{
   private final Function<Pose3DReadOnly, SCS2AvatarSimulation> simulationStarter;
   private final Supplier<KinematicsStreamingToolboxModule> ikStreamingSupplier;
   private final RDXSCS2LogSession logSession;
   private final ZEDSVOScrubber zedScrubber;
   private final SideDependentList<YoPose3D> logHandPoses = new SideDependentList<>();
   private final SideDependentList<RDXReferenceFrameGraphic> actionHandPoses = new SideDependentList<>();
   private long lastZEDTimestamp = -1;
   private final ROS2Node ros2Node;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RDXROS2RobotVisualizer robotVisualizer;
   private LeRobotInferenceManager inferenceManager;
   private boolean simulationStarted = false;
   private SCS2AvatarSimulation avatarSimulation;
   private YoEnum<HighLevelControllerName> controllerState;
   private YoDouble time;
   private boolean ikStreamingStarted = false;
   private KinematicsStreamingToolboxModule ikStreaming;
   private YoDouble toolboxTime;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private transient final ImBoolean runInference = new ImBoolean();

   public RDXLeRobotTestSimulator(Function<Pose3DReadOnly, SCS2AvatarSimulation> simulationStarter,
                                  Supplier<KinematicsStreamingToolboxModule> ikStreamingSupplier,
                                  Supplier<DRCRobotModel> robotModelSupplier,
                                  RDXBaseUI baseUI,
                                  RDXSCS2LogSession logSession)
   {
      this.simulationStarter = simulationStarter;
      this.ikStreamingSupplier = ikStreamingSupplier;
      this.logSession = logSession;

      zedScrubber = logSession.getFirstZEDScrubber();

      ros2Node = new ROS2NodeBuilder().build("lerobot_test_ui");
      DRCRobotModel robotModel = robotModelSupplier.get();
      ROS2ControllerHelper ros2 = new ROS2ControllerHelper(ros2Node, robotModel.getSimpleRobotName());
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      robotVisualizer = new RDXROS2RobotVisualizer(ros2, syncedRobot);
      robotVisualizer.createAndSetupStandalone(baseUI);
      robotVisualizer.setActive(false);

      SideDependentList<String> robotHandNames = LeRobotDatasetTools.getRobotHandNames(logSession.getSession().getRobotDefinitions().get(0));
      for (RobotSide side : robotHandNames.sides())
      {
         logHandPoses.put(side, LeRobotDatasetDataWriter.findYoPose(robotHandNames.get(side), "Current", logSession.getSession().getRootRegistry()));

         RDXReferenceFrameGraphic graphic = new RDXReferenceFrameGraphic(0.3);
         actionHandPoses.put(side, graphic);
         baseUI.getPrimaryScene().addRenderableProvider(graphic);
      }
   }

   public void update()
   {
      syncedRobot.update();
      robotVisualizer.update();

      if (inferenceManager != null && inferenceManager.isRunning())
      {
         long zedSVOTimestamp = zedScrubber.getCurrentTimestamp();

         if (zedSVOTimestamp > 0 && zedSVOTimestamp != lastZEDTimestamp)
         {
            inferenceManager.publishHandPoses(logHandPoses.get(RobotSide.LEFT), logHandPoses.get(RobotSide.RIGHT));

            int imageHeight = zedScrubber.getImageHeight();
            int imageWidth = zedScrubber.getImageWidth();

            for (RobotSide side : RobotSide.values)
            {
               Pointer zedColorImageSLMatPointer = side == RobotSide.LEFT ? zedScrubber.getLeftColorImageSlMatPointer()
                                                                          : zedScrubber.getRightColorImageSlMatPointer();
               Mat bgra8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, // BGRA8
                                      sl_mat_get_ptr(zedColorImageSLMatPointer, SL_MEM_CPU),
                                      sl_mat_get_step_bytes(zedColorImageSLMatPointer, SL_MEM_CPU));

               inferenceManager.publishImage(side, bgra8Mat);
            }
         }

         for (RobotSide robotSide : RobotSide.values)
         {
            actionHandPoses.get(robotSide).setPoseInWorldFrame(inferenceManager.getActionHandPoses().get(robotSide));
         }
      }
   }

   public void renderImGuiWidgets(LeRobotDataset dataset)
   {
      ImGuiTools.separatorText("Inference test simulator");

      if (Files.exists(dataset.getDirectory().resolve("last/pretrained_model")))
      {
         if (avatarSimulation == null && !simulationStarted)
         {
            if (ImGui.button(labels.get("Start kinematic simulation")))
            {
               Pose3D initialPose = new Pose3D();
               for (Robot logRobot : logSession.getSession().getRobots())
               {
                  if (logRobot.getRobotDefinition().getName().equalsIgnoreCase(syncedRobot.getRobotModel().getSimpleRobotName()))
                  {
                     if (logRobot.getFloatingRootJoint() instanceof SimFloatingRootJoint logPelvisSixDoF)
                     {
                        initialPose.set(logPelvisSixDoF.getJointPose());
                     }
                  }
               }
               initialPose.setZ(0.0);

               ThreadTools.startAThread(() ->
               {
                  simulationStarted = true;
                  avatarSimulation = simulationStarter.apply(initialPose);
                  SimulationConstructionSet2 scs = avatarSimulation.getSimulationConstructionSet();
                  controllerState = (YoEnum<HighLevelControllerName>) scs.findVariable("highLevelControllerNameCurrentState");
                  time = (YoDouble) scs.findVariable("time[sec]");
               }, "StartSimulation");

               robotVisualizer.setActive(true);
               robotVisualizer.setOpacity(0.7f);
            }
         }
         else if (controllerState != null && time != null)
         {
            ImGui.text("Kinematic simulation state: %s time: %.2f s".formatted(controllerState.getEnumValue(), time.getValue()));

            ImGui.beginDisabled();
            if (ImGui.button(labels.get("Reset to log robot (not working)")))
            {
               for (Robot logRobot : logSession.getSession().getRobots())
               {
                  if (logRobot.getRobotDefinition().getName().equalsIgnoreCase(syncedRobot.getRobotModel().getSimpleRobotName()))
                  {
                     TObjectDoubleMap<String> jointPositions = new TObjectDoubleHashMap<>();
                     SubtreeStreams.fromChildren(OneDoFJointBasics.class,
                                                 logRobot.getRootBody()).forEach(joint -> jointPositions.put(joint.getName(), joint.getQ()));
                     if (logRobot.getFloatingRootJoint() instanceof SimFloatingRootJoint logPelvisSixDoF)
                     {
                        RigidBodyTransform logPelvisPose = new RigidBodyTransform(logPelvisSixDoF.getJointPose());

                        SixDoFJoint simControllerRootJoint = (SixDoFJoint) avatarSimulation.getControllerFullRobotModel().getRootJoint();
                        SimFloatingRootJoint simRobotRootJoint = (SimFloatingRootJoint) avatarSimulation.getRobot().getRootBody().getChildrenJoints().get(0);

//                        simControllerRootJoint.setJointConfiguration(logPelvisPose);
                        simRobotRootJoint.setJointConfiguration(logPelvisPose);

                        avatarSimulation.getEstimatorThread().initializeStateEstimators(logPelvisPose, jointPositions);
                     }
                  }
               }
            }
            ImGui.endDisabled();

            ImGui.pushStyleColor(ImGuiCol.Button, ImGuiTools.DARK_RED);
            if (ImGui.button(labels.get("Stop kinematic simulation")))
            {
               simulationStarted = false;
               avatarSimulation.destroy();
               avatarSimulation = null;
               controllerState = null;
               time = null;
            }
            ImGui.popStyleColor();
         }
         else
         {
            ImGui.text("Kinematic simulation starting...");
         }

         if (ikStreaming == null && !ikStreamingStarted)
         {
            if (ImGui.button(labels.get("Start IK streaming")))
            {
               ThreadTools.startAThread(() ->
               {
                  ikStreamingStarted = true;
                  ikStreaming = ikStreamingSupplier.get();
                  toolboxTime = (YoDouble) ikStreaming.getRegistry().findVariable("time");
               }, "StartIKStreaming");
            }
         }
         else if (toolboxTime != null)
         {
            ImGui.text("IK streaming time: %.2f s".formatted(toolboxTime.getValue()));

            if (inferenceManager == null || !inferenceManager.getModelName().equals(dataset.getName()))
            {
               if (inferenceManager != null)
                  inferenceManager.destroy();

               inferenceManager = new LeRobotInferenceManager(dataset.getName(),
                                                              syncedRobot.getRobotModel().getSimpleRobotName(),
                                                              syncedRobot.getFullRobotModel(),
                                                              ros2Node);
            }

            if (ImGui.checkbox(labels.get("Run model"), runInference))
            {
               inferenceManager.setRunning(runInference.get());
            }

            if (ImGui.button(labels.get("Connect to Python")))
            {
               inferenceManager.startPythonServer();
            }

            ImGui.text("Python side status: %.2f Hz".formatted(inferenceManager.getStatusFrequency()));

            ImGui.pushStyleColor(ImGuiCol.Button, ImGuiTools.DARK_RED);
            if (ImGui.button(labels.get("Stop IK streaming")))
            {
               ikStreamingStarted = false;
               ikStreaming.destroy();
               inferenceManager.destroy();
               ikStreaming = null;
               toolboxTime = null;
               inferenceManager = null;
            }
            ImGui.popStyleColor();
         }
         else
         {
            ImGui.text("IK streaming starting...");
         }

      }
      else
      {
         ImGui.text("No trained models present.");
      }
   }

   public void destroy()
   {
      inferenceManager.destroy();
      robotVisualizer.destroy();
      syncedRobot.destroy();
      ros2Node.destroy();
      if (avatarSimulation != null)
         avatarSimulation.destroy();
      if (ikStreaming != null)
         ikStreaming.destroy();
   }

   public RDXROS2RobotVisualizer getRobotVisualizer()
   {
      return robotVisualizer;
   }
}
