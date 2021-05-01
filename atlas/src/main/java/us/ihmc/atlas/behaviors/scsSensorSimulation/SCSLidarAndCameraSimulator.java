package us.ihmc.atlas.behaviors.scsSensorSimulation;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.VideoPacket;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.behaviors.SCSVideoDataROS2Bridge;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.producers.VideoDataServerImageCallback;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotDescription.LidarSensorDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.simulatedSensors.LidarMount;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.tools.gui.AWTTools;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.IOException;

/**
 * Potential improvements:
 * - Show camera view only
 * - Hide as much of SCS as possible
 * - Remove SCS entirely?
 * - Add textures
 */
public class SCSLidarAndCameraSimulator
{
   private final ROS2Node ros2Node;
   private final ROS2Input<RobotConfigurationData> robotConfigurationData;
   private final RemoteSyncedRobotModel syncedRobot;
   private final FramePose3D tempNeckFramePose = new FramePose3D();
   private final SimulationConstructionSet scs;
   private final FloatingJoint floatingHeadJoint;

   public SCSLidarAndCameraSimulator(PubSubImplementation pubSubImplementation, CommonAvatarEnvironmentInterface environment, DRCRobotModel robotModel)
   {
      this(pubSubImplementation, environment.getTerrainObject3D(), robotModel);
   }

   public SCSLidarAndCameraSimulator(PubSubImplementation pubSubImplementation, TerrainObject3D terrainObject3D, DRCRobotModel robotModel)
   {
      ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "lidar_and_camera");

      robotConfigurationData = new ROS2Input<>(ros2Node, ROS2Tools.getRobotConfigurationDataTopic(robotModel.getSimpleRobotName()));

      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);

      Robot robot = new Robot("Robot");

      floatingHeadJoint = new FloatingJoint("head", new Vector3D(), robot);
      Link link = new Link("lidar");
      double radius = 0.05;
      link.setMassAndRadiiOfGyration(1.0, radius, radius, radius);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      link.setLinkGraphics(linkGraphics);
      linkGraphics.addModelFile("models/hokuyo.dae", YoAppearance.Black());
      linkGraphics.translate(0, 0, -0.1);
      link.setLinkGraphics(linkGraphics);
      floatingHeadJoint.setLink(link);
//      floatingHeadJoint.setLink(new Link("hello"));

      String videoCameraMountName = "videoCameraMount";
      CameraMount videoCameraMount = new CameraMount(videoCameraMountName, new Vector3D(), robot);
      floatingHeadJoint.addCameraMount(videoCameraMount);

      GimbalJoint gimbalJoint = setupLidarGimbal(robot);

//      floatingHeadJoint.addJoint(gimbalJoint);

      robot.addRootJoint(floatingHeadJoint);
      robot.setGravity(0.0);

      scs = new SimulationConstructionSet(robot);
      scs.setDT(0.001, 100); // TODO: Check this, might greatly alter performance

      FunctionalRobotController controller = new FunctionalRobotController();
      controller.setDoControl(this::doControl);

      robot.setController(controller);

      // must create a joint and attach a CameraMount; make it another robot?

      // required for timestamp
      ROS2Input<RobotConfigurationData> robotConfigurationData = new ROS2Input<>(ros2Node,
                                                                                 ROS2Tools.getRobotConfigurationDataTopic(robotModel.getSimpleRobotName()));
      IHMCROS2Publisher<VideoPacket> scsCameraPublisher = new IHMCROS2Publisher<>(ros2Node, ROS2Tools.VIDEO);
      CameraConfiguration cameraConfiguration = new CameraConfiguration(videoCameraMountName);
      cameraConfiguration.setCameraMount(videoCameraMountName);
      scs.setupCamera(cameraConfiguration);
      int width = 1024;
      int height = 544;
      int framesPerSecond = 25;
      scs.startStreamingVideoData(cameraConfiguration,
                                  width,
                                  height,
                                  new VideoDataServerImageCallback(new SCSVideoDataROS2Bridge(scsCameraPublisher::publish)),
                                  () -> robotConfigurationData.getLatest().getSyncTimestamp(),
                                  framesPerSecond);

      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(terrainObject3D.getLinkGraphics());
      scs.skipLoadingDefaultConfiguration();
      scs.setupGraph("t");

      scs.getGUI().getFrame().setSize(AWTTools.getDimensionOfSmallestScreenScaled(0.25));
      scs.startOnAThread();
      scs.simulate();
   }

   private GimbalJoint setupLidarGimbal(Robot lidarRobot)
   {
      double height = 0.2;
      double radius = 0.05;
      GimbalJoint gimbalJoint = new GimbalJoint("gimbalZ", "gimbalX", "gimbalY", new Vector3D(0.0, 0.0, 1.0), lidarRobot, Axis3D.Z, Axis3D.X, Axis3D.Y);
      Link link = new Link("lidar");
      link.setMassAndRadiiOfGyration(1.0, radius, radius, radius);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      link.setLinkGraphics(linkGraphics);
      gimbalJoint.setLink(link);
      gimbalJoint.setDamping(1.0);

      CameraMount robotCam = new CameraMount("camera", new Vector3D(radius + 0.001, 0.0, height / 2.0), lidarRobot);
      gimbalJoint.addCameraMount(robotCam);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation().set(new Vector3D(radius + 0.001, 0.0, height / 2.0));
      LidarScanParameters lidarScanParameters = new LidarScanParameters(720, (float) (-Math.PI / 2), (float) (Math.PI / 2), 0f, 0.1f, 30.0f, 0f);
      LidarSensorDescription lidarSensorDescription = new LidarSensorDescription("lidar", transform);
      lidarSensorDescription.setPointsPerSweep(lidarScanParameters.getPointsPerSweep());
      lidarSensorDescription.setScanHeight(lidarScanParameters.getScanHeight());
      lidarSensorDescription.setSweepYawLimits(lidarScanParameters.getSweepYawMin(), lidarScanParameters.getSweepYawMax());
      lidarSensorDescription.setHeightPitchLimits(lidarScanParameters.heightPitchMin, lidarScanParameters.heightPitchMax);
      lidarSensorDescription.setRangeLimits(lidarScanParameters.getMinRange(), lidarScanParameters.getMaxRange());
      LidarMount lidarMount = new LidarMount(lidarSensorDescription);

      gimbalJoint.addLidarMount(lidarMount);

      linkGraphics.addModelFile("models/hokuyo.dae", YoAppearance.Black());
      linkGraphics.translate(0, 0, -0.1);
      link.setLinkGraphics(linkGraphics);
      return gimbalJoint;
   }

   private void doControl()
   {
      syncedRobot.update();
      tempNeckFramePose.setToZero(syncedRobot.getReferenceFrames().getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH));
      tempNeckFramePose.changeFrame(ReferenceFrame.getWorldFrame());

      floatingHeadJoint.setPosition(tempNeckFramePose.getPosition());
      floatingHeadJoint.setQuaternion(tempNeckFramePose.getOrientation());
   }

   private Graphics3DObject createGroundLinkGraphicsFromGroundProfile(GroundProfile3D groundProfile)
   {
      Graphics3DObject texturedGroundLinkGraphics = new Graphics3DObject();
      HeightMap heightMap = groundProfile.getHeightMapIfAvailable();
      texturedGroundLinkGraphics.addHeightMap(heightMap, 300, 300, YoAppearance.DarkGreen());
      return texturedGroundLinkGraphics;
   }

   private static AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
   }

   private static CommonAvatarEnvironmentInterface createCommonAvatarEnvironment()
   {
      String environmentName = PlanarRegionsListDefinedEnvironment.class.getSimpleName();
//      ImageIO.read(
//            UtilImageIO.loadImage()loadImage(new File(f, "leftEyeImage.png").getAbsolutePath());
      BufferedImage image = ExceptionTools.handle(() -> ImageIO.read(Class.class.getResourceAsStream("/sampleMeshes/cinderblock.png")),
                                                  DefaultExceptionHandler.PRINT_STACKTRACE);
      YoAppearanceTexture cinderBlockTexture = new YoAppearanceTexture(image);
      return new PlanarRegionsListDefinedEnvironment(environmentName,
                                                     BehaviorPlanarRegionEnvironments.createRoughUpAndDownStepsWithFlatTop(),
                                                     cinderBlockTexture,
                                                     0.02,
                                                     false);
   }

   public void destroy()
   {
      LogTools.info("Shutting down");
      ThreadTools.startAsDaemon(scs::stopSimulationThread, "WaitForSimulationThreadToStop");
      scs.closeAndDispose();
      ros2Node.destroy();
   }

   public static void main(String[] args) throws IOException
   {
//      ShowImages.showWindow(ImageIO.read(Objects.requireNonNull(Class.class.getResourceAsStream("/sampleMeshes/cinderblock.png"))), "video");
//      ImageIO.read(
//            UtilImageIO.loadImage()loadImage(new File(f, "leftEyeImage.png").getAbsolutePath());

//      new SCSLidarAndCameraSimulator(ros2Node, DefaultCommonAvatarEnvironment.setUpShortCinderBlockField("CinderBlockField", 0.0, 1.0), createRobotModel());
//      new SCSLidarAndCameraSimulator(ros2Node, createCommonAvatarEnvironment(), createRobotModel());
      new SCSLidarAndCameraSimulator(PubSubImplementation.INTRAPROCESS, new FiducialEnvironmentForDoorBehavior(), createRobotModel());
   }
}
