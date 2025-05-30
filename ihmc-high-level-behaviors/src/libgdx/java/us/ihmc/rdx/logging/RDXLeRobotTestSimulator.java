package us.ihmc.rdx.logging;

import gnu.trove.map.TObjectDoubleMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDataset;
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
import us.ihmc.perception.lerobot.LeRobotManager;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.simulation.scs2.RDXSCS2LogSession;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimFloatingRootJoint;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.nio.file.Files;
import java.util.function.Function;
import java.util.function.Supplier;

public class RDXLeRobotTestSimulator
{
   private final Function<Pose3DReadOnly, SCS2AvatarSimulation> simulationStarter;
   private final Supplier<KinematicsStreamingToolboxModule> ikStreamingSupplier;
   private final RDXSCS2LogSession logSession;
   private final ROS2Node ros2Node;
   private final ROS2SyncedRobotModel syncedRobot;
   private final RDXROS2RobotVisualizer robotVisualizer;
   private LeRobotManager inferenceManager;
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

      ros2Node = new ROS2NodeBuilder().build("lerobot_test_ui");
      DRCRobotModel robotModel = robotModelSupplier.get();
      ROS2ControllerHelper ros2 = new ROS2ControllerHelper(ros2Node, robotModel.getSimpleRobotName());
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      robotVisualizer = new RDXROS2RobotVisualizer(ros2, syncedRobot);
      robotVisualizer.createAndSetupStandalone(baseUI);
      robotVisualizer.setActive(false);
   }

   public void update()
   {
      syncedRobot.update();
      robotVisualizer.update();
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
         }
         else
         {
            ImGui.text("IK streaming starting...");
         }



         if (inferenceManager == null || inferenceManager.getModelName().equals(dataset.getName()))
         {
            inferenceManager = new LeRobotManager(dataset.getName());


            

         }

         if (ImGui.checkbox(labels.get("Run model"), runInference))
         {

         }
      }
      else
      {
         ImGui.text("No trained models present.");
      }
   }

   public void destroy()
   {
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
