package us.ihmc.gdx.ui;

import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.behaviors.simulation.EnvironmentInitialSetup;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.ui.behaviors.registry.GDXBehaviorUIRegistry;
import us.ihmc.gdx.ui.missionControl.MissionControlProcess;
import us.ihmc.gdx.ui.missionControl.RestartableMissionControlProcess;
import us.ihmc.gdx.ui.missionControl.processes.*;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.impl.intraprocess.IntraProcessDomain;

import java.util.ArrayList;

import static us.ihmc.gdx.imgui.ImGuiTools.uniqueIDOnly;
import static us.ihmc.gdx.imgui.ImGuiTools.uniqueLabel;

public abstract class GDXProcessManagerPanel
{
   private final String windowName = ImGuiTools.uniqueLabel("Process Manager");
   private final String ROS_URI_STRING = String.valueOf(NetworkParameters.getROSURI());
   private final String RTPS_DOMAIN_ID_STRING = String.valueOf(NetworkParameters.getRTPSDomainID());
   private final String RTPS_SUBNET_STRING = String.valueOf(NetworkParameters.getHost(NetworkParameterKeys.RTPSSubnet));

   protected final ImInt robotTarget = new ImInt(1);
   protected final String[] robotTargets = new String[] {"Real robot", "Simulation"};
   protected final ImInt ros2Mode = new ImInt(0);
   protected final ImInt messagerMode = new ImInt(0);
   protected final ImBoolean logToFile = new ImBoolean(false);

   protected final ArrayList<MissionControlProcess> processes = new ArrayList<>();

   private final RestartableMissionControlProcess ros1MasterProcess;
   private final RestartableMissionControlProcess behaviorModuleProcess;
   private final RestartableMissionControlProcess behaviorManagerProcess;
   private final RestartableMissionControlProcess footstepPlanningModuleProcess;
   private final RestartableMissionControlProcess mapsenseHeadlessProcess;
   private final RestartableMissionControlProcess objectDetectionProcess;
   private final RestartableMissionControlProcess lidarREAProcess;
   protected final EnvironmentInitialSetup environmentInitialSetup;

   public GDXProcessManagerPanel()
   {
      // TODO: Make into GUI selection
      Pose3D startingPose = new Pose3D();
//      startingPose.set(0.17, 0.17, 0.0, Math.toRadians(180.0), 0.0, 0.0);
      environmentInitialSetup = new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::flatGround,
                                                                                    startingPose.getZ(),
                                                                                    Math.toRadians(0.0),
                                                                                    startingPose.getX(),
                                                                                    startingPose.getY());

      // TODO: GUI selection
      GDXBehaviorUIRegistry defaultBehaviors = GDXBehaviorUIRegistry.DEFAULT_BEHAVIORS;

      ros1MasterProcess = new ROS1MasterProcess();
      behaviorModuleProcess = new BehaviorModuleProcess(this::createRobotModel, ros2Mode, messagerMode);
      behaviorManagerProcess = new BehaviorManagerProcess(this::createRobotModel);
      footstepPlanningModuleProcess = new FootstepPlanningModuleProcess(this::createRobotModel, this::getROS2Mode);
      mapsenseHeadlessProcess = new MapSenseHeadlessProcess();
      objectDetectionProcess = new ObjectDetectionProcess(this::createRobotModel, this::getROS2Mode, this::getRobotTarget);
      lidarREAProcess = new LidarREAProcess();

      processes.add(ros1MasterProcess);
      processes.add(behaviorModuleProcess);
      processes.add(behaviorManagerProcess);
      processes.add(footstepPlanningModuleProcess);
      processes.add(mapsenseHeadlessProcess);
      processes.add(objectDetectionProcess);
      processes.add(lidarREAProcess);
   }

   private PubSubImplementation getROS2Mode()
   {
      return CommunicationMode.fromOrdinal(ros2Mode.get()).getPubSubImplementation();
   }

   private RobotTarget getRobotTarget()
   {
      return RobotTarget.values()[robotTarget.get()];
   }

   public void render()
   {
      ImGui.begin(windowName);

      ImGui.text("ROS Master URI: " + ROS_URI_STRING);
      ImGui.text("RTPS Domain ID: " + RTPS_DOMAIN_ID_STRING);
      ImGui.text("RTPS Subnet Restriction: " + RTPS_SUBNET_STRING);

      ImGui.pushItemWidth(150.0f);
      ImGui.text("Robot target:");
      ImGui.sameLine();
      ImGui.combo(uniqueIDOnly(this, "RobotTarget"), robotTarget, robotTargets, robotTargets.length);
      ImGui.text("Robot version:");
      ImGui.sameLine();
      ImGui.combo(uniqueIDOnly(this, "RobotVersion"), getRobotVersion(), getRobotVersions(), getRobotVersions().length);
      ImGui.text("ROS 2 Mode: ");
      ImGui.sameLine();
      ImGui.combo(uniqueIDOnly(this, "ROS2Mode"), ros2Mode, CommunicationMode.ROS2_NAMES, CommunicationMode.VALUES.length);
      ImGui.text("Messager Mode: ");
      ImGui.sameLine();
      ImGui.combo(uniqueIDOnly(this, "MessagerMode"), messagerMode, CommunicationMode.MESSAGER_NAMES, CommunicationMode.VALUES.length);
      ImGui.popItemWidth();

      ImGui.text("Simulation:");

      ImGui.checkbox(uniqueLabel(this, "Log to file"), logToFile);

      for (MissionControlProcess process : processes)
      {
         process.render();
      }

      ImGui.end();
   }

   protected abstract ImInt getRobotVersion();

   protected abstract String[] getRobotVersions();

   protected abstract DRCRobotModel createRobotModel();

   public void dispose()
   {
      // Destroy in a reasonable order
      mapsenseHeadlessProcess.destroy();
      behaviorModuleProcess.destroy();
      footstepPlanningModuleProcess.destroy();
      objectDetectionProcess.destroy();
      lidarREAProcess.destroy();

      // destroy em all just in case
      for (MissionControlProcess process : processes)
      {
         process.destroy();
      }

      IntraProcessDomain.getInstance().stopAll();
   }

   public void setRobotTarget(RobotTarget robotTarget)
   {
      this.robotTarget.set(robotTarget.ordinal());
   }

   public void setMessagerMode(CommunicationMode communicationMode)
   {
      messagerMode.set(communicationMode.ordinal());
   }

   public RestartableMissionControlProcess getBehaviorModuleProcess()
   {
      return behaviorModuleProcess;
   }

   public RestartableMissionControlProcess getRos1MasterProcess()
   {
      return ros1MasterProcess;
   }

   public RestartableMissionControlProcess getBehaviorManagerProcess()
   {
      return behaviorManagerProcess;
   }

   public RestartableMissionControlProcess getFootstepPlanningModuleProcess()
   {
      return footstepPlanningModuleProcess;
   }

   public RestartableMissionControlProcess getMapsenseHeadlessProcess()
   {
      return mapsenseHeadlessProcess;
   }

   public RestartableMissionControlProcess getObjectDetectionProcess()
   {
      return objectDetectionProcess;
   }

   public RestartableMissionControlProcess getLidarREAProcess()
   {
      return lidarREAProcess;
   }

   public String getWindowName()
   {
      return windowName;
   }
}
