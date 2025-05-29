package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDataset;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.perception.lerobot.LeRobotManager;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.nio.file.Files;
import java.util.function.Supplier;

public class RDXLeRobotTestSimulator
{
   private final Supplier<SCS2AvatarSimulation> simulationStarter;
   private final Supplier<KinematicsStreamingToolboxModule> ikStreamingSupplier;
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

   public RDXLeRobotTestSimulator(Supplier<SCS2AvatarSimulation> simulationStarter,
                                  Supplier<KinematicsStreamingToolboxModule> ikStreamingSupplier,
                                  Supplier<DRCRobotModel> robotModelSupplier,
                                  RDXBaseUI baseUI)
   {
      this.simulationStarter = simulationStarter;
      this.ikStreamingSupplier = ikStreamingSupplier;

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
               ThreadTools.startAThread(() ->
               {
                  simulationStarted = true;
                  avatarSimulation = simulationStarter.get();
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

            if (ImGui.button(labels.get("Stop kinematic simulation")))
            {
               simulationStarted = false;
               avatarSimulation.destroy();
               avatarSimulation = null;
               controllerState = null;
               time = null;
            }

            // TODO Reset kinematics simulation robot to the log robot's pose
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
