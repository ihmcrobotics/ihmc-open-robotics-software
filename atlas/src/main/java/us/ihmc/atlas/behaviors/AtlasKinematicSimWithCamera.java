package us.ihmc.atlas.behaviors;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.behaviors.scsSensorSimulation.SensorOnlySimulation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.producers.VideoDataServerImageCallback;
import us.ihmc.humanoidBehaviors.tools.RemoteSyncedHumanoidRobotState;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class AtlasKinematicSimWithCamera
{
   private IHMCROS2Publisher<VideoPacket> scsCameraPublisher;

   public AtlasKinematicSimWithCamera(DRCRobotModel robotModel, CommonAvatarEnvironmentInterface environment)
   {
      HumanoidKinematicsSimulation.create(robotModel, false, PubSubImplementation.FAST_RTPS);

      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "kinematic_camera");
      scsCameraPublisher = new IHMCROS2Publisher<>(ros2Node, VideoPacket.class);

      RemoteSyncedHumanoidRobotState remoteSyncedHumanoidFrames = new RemoteSyncedHumanoidRobotState(robotModel, ros2Node);
      remoteSyncedHumanoidFrames.pollHumanoidRobotState().getNeckFrame(NeckJointName.DISTAL_NECK_PITCH);

      /// create scs
      SensorOnlySimulation sensorOnlySimulation = new SensorOnlySimulation();
      SimulationConstructionSet scs = sensorOnlySimulation.getSCS();

      // create camera in scs

      ROS2Input<RobotConfigurationData> robotConfigurationData = new ROS2Input<>(ros2Node,
                                                                                 RobotConfigurationData.class,
                                                                                 robotModel.getSimpleRobotName(),
                                                                                 ROS2Tools.HUMANOID_CONTROLLER);

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
   }

   public static void main(String[] args)
   {
      new AtlasKinematicSimWithCamera(new AtlasRobotModel(AtlasBehaviorModule.ATLAS_VERSION, RobotTarget.SCS, false), new FlatGroundEnvironment());
   }
}
