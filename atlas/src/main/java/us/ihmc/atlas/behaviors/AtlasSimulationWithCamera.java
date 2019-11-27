package us.ihmc.atlas.behaviors;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.factory.AvatarSimulationFactory;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.generated.YUVPicture.YUVSubsamplingType;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.producers.VideoDataServer;
import us.ihmc.communication.producers.VideoDataServerImageCallback;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.ByteBuffer;

import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.DO_NOTHING_BEHAVIOR;
import static us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName.WALKING;

public class AtlasSimulationWithCamera
{
   private IHMCRealtimeROS2Publisher<VideoPacket> scsCameraPublisher;

   public SimulationConstructionSet createForManualTest(DRCRobotModel robotModel, CommonAvatarEnvironmentInterface environment)
   {
      return create(robotModel, environment, PubSubImplementation.FAST_RTPS);
   }

   public SimulationConstructionSet createForAutomatedTest(DRCRobotModel robotModel, CommonAvatarEnvironmentInterface environment)
   {
      return create(robotModel, environment, PubSubImplementation.INTRAPROCESS);
   }

   private SimulationConstructionSet create(DRCRobotModel robotModel, CommonAvatarEnvironmentInterface environment, PubSubImplementation pubSubImplementation)
   {
      SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false, simulationTestingParameters);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(environment, robotModel.getSimulateDT());
      scsInitialSetup.setInitializeEstimatorToActual(true);
      scsInitialSetup.setTimePerRecordTick(robotModel.getControllerDT());
      scsInitialSetup.setUsePerfectSensors(true);

      RobotContactPointParameters<RobotSide> contactPointParameters = robotModel.getContactPointParameters();
      ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
      contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
      contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                       contactPointParameters.getControllerToeContactLines());
      for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
      {
         contactableBodiesFactory.addAdditionalContactPoint(contactPointParameters.getAdditionalContactRigidBodyNames().get(i),
                                                            contactPointParameters.getAdditionalContactNames().get(i),
                                                            contactPointParameters.getAdditionalContactTransforms().get(i));
      }

      RealtimeRos2Node realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(pubSubImplementation, "humanoid_simulation_controller");

      scsCameraPublisher = ROS2Tools.createPublisher(realtimeRos2Node, VideoPacket.class, "/ihmc/video");

      HighLevelHumanoidControllerFactory controllerFactory = new HighLevelHumanoidControllerFactory(contactableBodiesFactory,
                                                                                                    robotModel.getSensorInformation().getFeetForceSensorNames(),
                                                                                                    robotModel.getSensorInformation()
                                                                                                              .getFeetContactSensorNames(),
                                                                                                    robotModel.getSensorInformation()
                                                                                                              .getWristForceSensorNames(),
                                                                                                    robotModel.getHighLevelControllerParameters(),
                                                                                                    robotModel.getWalkingControllerParameters(),
                                                                                                    robotModel.getCapturePointPlannerParameters());
      controllerFactory.useDefaultDoNothingControlState();
      controllerFactory.useDefaultWalkingControlState();
      controllerFactory.addRequestableTransition(DO_NOTHING_BEHAVIOR, WALKING);
      controllerFactory.addRequestableTransition(WALKING, DO_NOTHING_BEHAVIOR);
      controllerFactory.addControllerFailureTransition(DO_NOTHING_BEHAVIOR, DO_NOTHING_BEHAVIOR);
      controllerFactory.addControllerFailureTransition(WALKING, DO_NOTHING_BEHAVIOR);
      controllerFactory.setInitialState(HighLevelControllerName.WALKING);
      controllerFactory.createControllerNetworkSubscriber(robotModel.getSimpleRobotName(), realtimeRos2Node);

      AvatarSimulationFactory avatarSimulationFactory = new AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(robotModel);
      avatarSimulationFactory.setShapeCollisionSettings(robotModel.getShapeCollisionSettings());
      avatarSimulationFactory.setHighLevelHumanoidControllerFactory(controllerFactory);
      avatarSimulationFactory.setCommonAvatarEnvironment(environment);
      avatarSimulationFactory.setRobotInitialSetup(robotModel.getDefaultRobotInitialSetup(0.0, 0.0));
      avatarSimulationFactory.setSCSInitialSetup(scsInitialSetup);
      avatarSimulationFactory.setGuiInitialSetup(guiInitialSetup);
      avatarSimulationFactory.setRealtimeRos2Node(realtimeRos2Node);
      avatarSimulationFactory.setCreateYoVariableServer(false);

      AvatarSimulation avatarSimulation = avatarSimulationFactory.createAvatarSimulation();

      avatarSimulation.start();
      realtimeRos2Node.spin();  // TODO Should probably happen in start()

      // TODO set up some useful graphs

      SimulationConstructionSet scs = avatarSimulation.getSimulationConstructionSet();

      HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      TimestampProvider timeStampProvider = avatarSimulation.getSimulatedRobotTimeProvider();
      HumanoidFloatingRootJointRobot robot = avatarSimulation.getHumanoidFloatingRootJointRobot();

      AvatarRobotCameraParameters cameraParameters = sensorInformation.getCameraParameters(0);
      String cameraName = cameraParameters.getSensorNameInSdf();
      int width = robot.getCameraMount(cameraName).getImageWidth();
      int height = robot.getCameraMount(cameraName).getImageHeight();

      CameraConfiguration cameraConfiguration = new CameraConfiguration(cameraName);
      cameraConfiguration.setCameraMount(cameraName);

      LogTools.info("w: {}, h: {}", width, height);

      int framesPerSecond = 25;
      scs.startStreamingVideoData(cameraConfiguration,
                                  width,
                                  height,
                                  new VideoDataServerImageCallback(new VideoPacketCallback()),
                                  timeStampProvider,
                                  framesPerSecond);

      scs.setupGraph("root.atlas.t");
      scs.setupGraph(
            "root.atlas.DRCSimulation.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.highLevelControllerNameCurrentState");

      return scs;
   }

   private static final Object hackyLockBecauseJPEGEncoderIsNotThreadsafe = new Object();

   class VideoPacketCallback implements VideoDataServer
   {
      private final YUVPictureConverter converter = new YUVPictureConverter();
      private final JPEGEncoder encoder = new JPEGEncoder();

      @Override
      public void onFrame(VideoSource videoSource,
                          BufferedImage bufferedImage,
                          long timeStamp,
                          Point3DReadOnly cameraPosition,
                          QuaternionReadOnly cameraOrientation,
                          IntrinsicParameters intrinsicParameters)
      {

         YUVPicture picture = converter.fromBufferedImage(bufferedImage, YUVSubsamplingType.YUV420);
         try
         {
            ByteBuffer buffer;
            synchronized (hackyLockBecauseJPEGEncoderIsNotThreadsafe)
            {
               buffer = encoder.encode(picture, 75);
            }
            byte[] data = new byte[buffer.remaining()];
            buffer.get(data);
            VideoPacket videoPacket = HumanoidMessageTools.createVideoPacket(videoSource,
                                                                             timeStamp,
                                                                             data,
                                                                             cameraPosition,
                                                                             cameraOrientation,
                                                                             intrinsicParameters);
            scsCameraPublisher.publish(videoPacket);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         picture.delete();
      }

      @Override
      public boolean isConnected()
      {
         return true; // do nothing
      }
   }

   public static void main(String[] args)
   {
      SimulationConstructionSet scs = new AtlasSimulationWithCamera().createForManualTest(new AtlasRobotModel(AtlasBehaviorModule.ATLAS_VERSION,
                                                                                                              RobotTarget.SCS,
                                                                                                              false), new FlatGroundEnvironment());
      scs.simulate();
   }
}
