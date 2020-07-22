package us.ihmc.atlas.behaviors;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.behaviors.scsSensorSimulation.SensorOnlySimulation;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedRobotModel;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.producers.VideoDataServerImageCallback;
import us.ihmc.humanoidBehaviors.ui.simulation.RobotAndMapViewer;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

public class AtlasKinematicSimWithCamera
{
   private static boolean SHOW_ROBOT_VIEWER = false;
   private static boolean LOG_TO_FILE = Boolean.parseBoolean(System.getProperty("log.to.file"));
   private static boolean CREATE_YOVARIABLE_SERVER = Boolean.parseBoolean(System.getProperty("create.yovariable.server"));

   private IHMCROS2Publisher<VideoPacket> scsCameraPublisher;
   private final Ros2Node ros2Node;

   public AtlasKinematicSimWithCamera(CommonAvatarEnvironmentInterface environment)
   {
      AtlasRobotModel robotModel = createRobotModel();

      HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
      kinematicsSimulationParameters.setPubSubImplementation(PubSubImplementation.FAST_RTPS);
      kinematicsSimulationParameters.setLogToFile(LOG_TO_FILE);
      kinematicsSimulationParameters.setCreateYoVariableServer(CREATE_YOVARIABLE_SERVER);
      AtlasKinematicSimulation.create(robotModel, kinematicsSimulationParameters);

      ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "kinematic_camera");

      if (SHOW_ROBOT_VIEWER) ThreadTools.startAThread(this::robotViewer, "RobotViewer");

      scsCameraPublisher = new IHMCROS2Publisher<>(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT);

      RemoteSyncedRobotModel syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);
      syncedRobot.update();
      syncedRobot.getReferenceFrames().getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);

      /// create scs
      SensorOnlySimulation sensorOnlySimulation = new SensorOnlySimulation();
      SimulationConstructionSet scs = sensorOnlySimulation.getSCS();

      // create camera in scs

      ROS2Input<RobotConfigurationData> robotConfigurationData = new ROS2Input<>(ros2Node,
                                                                                 RobotConfigurationData.class,
                                                                                 ROS2Tools.HUMANOID_CONTROLLER.withRobot(robotModel.getSimpleRobotName())
                                                                                                              .withOutput());

      String cameraName = "camera";

      CameraConfiguration cameraConfiguration = new CameraConfiguration(cameraName);
      cameraConfiguration.setCameraMount(cameraName);

      int width = 1024;
      int height = 544;
      int framesPerSecond = 25;
      scs.startStreamingVideoData(cameraConfiguration,
                                  width,
                                  height,
                                  new VideoDataServerImageCallback(new SCSVideoDataROS2Bridge(scsCameraPublisher::publish)),
                                  () -> robotConfigurationData.getLatest().getSyncTimestamp(),
                                  framesPerSecond);

      scs.simulate();
   }

   private void robotViewer()
   {
      LogTools.info("Creating robot and map viewer");
      new RobotAndMapViewer(createRobotModel(), ros2Node);
   }

   private AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
   }

   public static void main(String[] args)
   {
      new AtlasKinematicSimWithCamera(new FlatGroundEnvironment());
   }
}
