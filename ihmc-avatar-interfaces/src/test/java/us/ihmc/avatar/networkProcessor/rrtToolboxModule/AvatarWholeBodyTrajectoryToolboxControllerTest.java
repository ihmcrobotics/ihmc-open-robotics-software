package us.ihmc.avatar.networkProcessor.rrtToolboxModule;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.fail;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.AvatarHumanoidKinematicsToolboxControllerTest.createCapturabilityBasedStatus;
import static us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.AvatarHumanoidKinematicsToolboxControllerTest.extractRobotConfigurationData;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.PITCH;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.ROLL;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName.YAW;
import static us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.createTrajectoryMessage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.IntStream;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTest;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.SegmentedLine3DMeshDataGenerator;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxMessageFactory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.ConfigurationSpaceName;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.RigidBodyExplorationConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WaypointBasedTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessageTools.FunctionTrajectory;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxOutputStatus;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.simulatedSensors.DRCPerfectSensorReaderFactory;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class AvatarWholeBodyTrajectoryToolboxControllerTest implements MultiRobotTestInterface
{
   private static final AppearanceDefinition ghostApperance = YoAppearance.DarkGreen();
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getCreateGUI();
   static
   {
      simulationTestingParameters.setKeepSCSUp(true);
      simulationTestingParameters.setDataBufferSize(1 << 16);
   }

   private CommandInputManager commandInputManager;
   private StatusMessageOutputManager statusOutputManager;
   private YoVariableRegistry mainRegistry;
   private YoGraphicsListRegistry yoGraphicsListRegistry;
   private WholeBodyTrajectoryToolboxController toolboxController;

   private YoBoolean initializationSucceeded;
   private YoInteger numberOfIterations;

   private SimulationConstructionSet scs;

   private HumanoidFloatingRootJointRobot robot;
   private HumanoidFloatingRootJointRobot ghost;
   private RobotController toolboxUpdater;

   /**
    * Returns a separate instance of the robot model that will be modified in this test to create a
    * ghost robot.
    */
   public abstract DRCRobotModel getGhostRobotModel();

   @Before
   public void setup()
   {
      mainRegistry = new YoVariableRegistry("main");
      initializationSucceeded = new YoBoolean("initializationSucceeded", mainRegistry);
      numberOfIterations = new YoInteger("numberOfIterations", mainRegistry);
      yoGraphicsListRegistry = new YoGraphicsListRegistry();

      DRCRobotModel robotModel = getRobotModel();

      FullHumanoidRobotModel desiredFullRobotModel = robotModel.createFullRobotModel();
      commandInputManager = new CommandInputManager(WholeBodyTrajectoryToolboxModule.supportedCommands());
      commandInputManager.registerConversionHelper(new WholeBodyTrajectoryToolboxCommandConverter(desiredFullRobotModel));

      statusOutputManager = new StatusMessageOutputManager(WholeBodyTrajectoryToolboxModule.supportedStatus());

      toolboxController = new WholeBodyTrajectoryToolboxController(getRobotModel(), desiredFullRobotModel, commandInputManager, statusOutputManager,
                                                                   mainRegistry, yoGraphicsListRegistry, visualize);
      toolboxController.setPacketDestination(PacketDestination.BROADCAST); // The actual destination does not matter, just need to make sure the internal field is != null

      robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      toolboxUpdater = createToolboxUpdater();
      robot.setController(toolboxUpdater);
      robot.setDynamic(false);
      robot.setGravity(0);

      DRCRobotModel ghostRobotModel = getGhostRobotModel();
      RobotDescription robotDescription = ghostRobotModel.getRobotDescription();
      robotDescription.setName("Ghost");
      KinematicsToolboxControllerTest.recursivelyModifyGraphics(robotDescription.getChildrenJoints().get(0), ghostApperance);
      ghost = ghostRobotModel.createHumanoidFloatingRootJointRobot(false);
      ghost.setDynamic(false);
      ghost.setGravity(0);
      hideGhost();

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot[] {robot, ghost}, simulationTestingParameters);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
         scs.setCameraFix(0.0, 0.0, 1.0);
         scs.setCameraPosition(8.0, 0.0, 3.0);
         scs.startOnAThread();
      }
   }

   private void hideGhost()
   {
      ghost.setPositionInWorld(new Point3D(-100.0, -100.0, -100.0));
      ghost.update();
   }

   private void hideRobot()
   {
      robot.setPositionInWorld(new Point3D(-100.0, -100.0, -100.0));
      robot.update();
   }

   private void snapGhostToFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      new JointAnglesWriter(ghost, fullHumanoidRobotModel).updateRobotConfigurationBasedOnFullRobotModel();
   }

   @After
   public void tearDown()
   {
      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();

      if (mainRegistry != null)
      {
         mainRegistry.closeAndDispose();
         mainRegistry = null;
      }

      initializationSucceeded = null;

      yoGraphicsListRegistry = null;

      commandInputManager = null;
      statusOutputManager = null;

      toolboxController = null;

      robot = null;
      toolboxUpdater = null;

      if (scs != null)
      {
         scs.closeAndDispose();
         scs = null;
      }
   }

   private final double circleRadius = 0.25;
   private final SideDependentList<Point3D> circleCenters = new SideDependentList<>(new Point3D(0.4, 0.3, 1.1), new Point3D(0.4, -0.3, 1.1));
   private final Vector3D circleAxis = new Vector3D(1.0, 0.0, 0.0);

   @Test
   public void testHandCirclePositionOnlyAndNoChestNoPelvis() throws Exception, UnreasonableAccelerationException
   {
      double trajectoryTime = 5.0;
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.setInitialConfigration(fullRobotModel);
      configuration.setMaximumExpansionSize(1000);

      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      List<RigidBodyExplorationConfigurationMessage> rigidBodyConfigurations = new ArrayList<>();
      
      double timeResolution = trajectoryTime / 100.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = fullRobotModel.getHand(robotSide);
         FunctionTrajectory handFunction = time -> computeCircleTrajectory(time, trajectoryTime, circleRadius, circleCenters.get(robotSide), circleAxis);
         ConfigurationSpaceName[] unconstrainedDegreesOfFreedom = {YAW, PITCH, ROLL};
         WaypointBasedTrajectoryMessage trajectory = createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution, handFunction,
                                                                             unconstrainedDegreesOfFreedom);
         handTrajectories.add(trajectory);

         ConfigurationSpaceName[] handConfigurations = {YAW, PITCH, ROLL};
         RigidBodyExplorationConfigurationMessage rigidBodyConfiguration = new RigidBodyExplorationConfigurationMessage(hand, handConfigurations);
         
         rigidBodyConfigurations.add(rigidBodyConfiguration);

         if (visualize)
            scs.addStaticLinkGraphics(createFunctionTrajectoryVisualization(handFunction, 0.0, trajectoryTime, timeResolution, 0.01, YoAppearance.AliceBlue()));
      }

      ConfigurationSpaceName[] pelvisConfigurations = {ConfigurationSpaceName.Z};
      RigidBodyExplorationConfigurationMessage pelvisConfigurationMessage = new RigidBodyExplorationConfigurationMessage(fullRobotModel.getPelvis(), pelvisConfigurations);
      RigidBodyExplorationConfigurationMessage chestConfigurationMessage = new RigidBodyExplorationConfigurationMessage(fullRobotModel.getChest());
      rigidBodyConfigurations.add(pelvisConfigurationMessage);
      //rigidBodyConfigurations.add(chestConfigurationMessage);
            
      WholeBodyTrajectoryToolboxMessage message = new WholeBodyTrajectoryToolboxMessage(configuration, handTrajectories, rigidBodyConfigurations);
      commandInputManager.submitMessage(message);

      System.out.println("submit done");
      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxOutputStatus solution = runToolboxController(maxNumberOfIterations);

      if (numberOfIterations.getIntegerValue() < maxNumberOfIterations - 1)
         assertNotNull("The toolbox is done but did not report a solution.", solution);
      else
         fail("The toolbox has run for " + maxNumberOfIterations + " without converging nor aborting.");

      if (visualize)
         visualizeSolution(solution, 0.1 * timeResolution);
   }

   @Test
   public void testHandCirclePositionOnly() throws Exception, UnreasonableAccelerationException
   {
      double trajectoryTime = 5.0;
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.setInitialConfigration(fullRobotModel);

      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      double timeResolution = trajectoryTime / 100.0;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = fullRobotModel.getHand(robotSide);
         FunctionTrajectory handFunction = time -> computeCircleTrajectory(time, trajectoryTime, circleRadius, circleCenters.get(robotSide), circleAxis);
         ConfigurationSpaceName[] unconstrainedDegreesOfFreedom = {YAW, PITCH, ROLL};
         WaypointBasedTrajectoryMessage trajectory = createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution, handFunction,
                                                                             unconstrainedDegreesOfFreedom);
         handTrajectories.add(trajectory);

         if (visualize)
            scs.addStaticLinkGraphics(createFunctionTrajectoryVisualization(handFunction, 0.0, trajectoryTime, timeResolution, 0.01, YoAppearance.AliceBlue()));
      }

      WholeBodyTrajectoryToolboxMessage message = new WholeBodyTrajectoryToolboxMessage(configuration, handTrajectories);
      commandInputManager.submitMessage(message);

      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxOutputStatus solution = runToolboxController(maxNumberOfIterations);

      if (numberOfIterations.getIntegerValue() < maxNumberOfIterations - 1)
         assertNotNull("The toolbox is done but did not report a solution.", solution);
      else
         fail("The toolbox has run for " + maxNumberOfIterations + " without converging nor aborting.");

      if (visualize)
         visualizeSolution(solution, 0.1 * timeResolution);
   }

   @Test
   public void testHandCircleFullyConstrained() throws Exception, UnreasonableAccelerationException
   {
      double trajectoryTime = 5.0;
      FullHumanoidRobotModel fullRobotModel = createFullRobotModelAtInitialConfiguration();
      WholeBodyTrajectoryToolboxConfigurationMessage configuration = new WholeBodyTrajectoryToolboxConfigurationMessage();
      configuration.setInitialConfigration(fullRobotModel);

      List<WaypointBasedTrajectoryMessage> handTrajectories = new ArrayList<>();
      double timeResolution = trajectoryTime / 100.0;

      SideDependentList<Pose3D> poses = computePrivilegedHandPosesAtPositions(circleCenters);
      if (poses == null)
         throw new RuntimeException("Could not solve for positions: " + circleCenters);

      for (RobotSide robotSide : RobotSide.values)
      {
         Graphics3DObject graphics3dObject = new Graphics3DObject();
         graphics3dObject.transform(new RigidBodyTransform(poses.get(robotSide).getOrientation(), poses.get(robotSide).getPosition()));
         graphics3dObject.addCoordinateSystem(0.1);
         scs.addStaticLinkGraphics(graphics3dObject);

         RigidBody hand = fullRobotModel.getHand(robotSide);

         QuaternionReadOnly orientation = poses.get(robotSide).getOrientation();
         boolean ccw = robotSide == RobotSide.RIGHT;
         double phase = 0.0; //robotSide.negateIfLeftSide(Math.PI / 2.0);
         FunctionTrajectory handFunction = time -> computeCircleTrajectory(time, trajectoryTime, circleRadius, circleCenters.get(robotSide), circleAxis,
                                                                           orientation, ccw, phase);
         WaypointBasedTrajectoryMessage trajectory = createTrajectoryMessage(hand, 0.0, trajectoryTime, timeResolution, handFunction);
         handTrajectories.add(trajectory);

         if (visualize)
            scs.addStaticLinkGraphics(createFunctionTrajectoryVisualization(handFunction, 0.0, trajectoryTime, timeResolution, 0.01, YoAppearance.AliceBlue()));
      }

      WholeBodyTrajectoryToolboxMessage message = new WholeBodyTrajectoryToolboxMessage(configuration, handTrajectories);
      commandInputManager.submitMessage(message);

      int maxNumberOfIterations = 10000;
      WholeBodyTrajectoryToolboxOutputStatus solution = runToolboxController(maxNumberOfIterations);

      if (numberOfIterations.getIntegerValue() < maxNumberOfIterations - 1)
         assertNotNull("The toolbox is done but did not report a solution.", solution);
      else
         fail("The toolbox has run for " + maxNumberOfIterations + " without converging nor aborting.");

      if (visualize)
         visualizeSolution(solution, 0.1 * timeResolution);
   }

   private SideDependentList<Pose3D> computePrivilegedHandPosesAtPositions(SideDependentList<Point3D> desiredPositions)
   {
      CommandInputManager commandInputManager = new CommandInputManager(KinematicsToolboxModule.supportedCommands());
      StatusMessageOutputManager statusOutputManager = new StatusMessageOutputManager(KinematicsToolboxModule.supportedStatus());
      FullHumanoidRobotModel desiredFullRobotModel = getRobotModel().createFullRobotModel();

      commandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel));
      HumanoidKinematicsToolboxController whik = new HumanoidKinematicsToolboxController(commandInputManager, statusOutputManager, desiredFullRobotModel,
                                                                                         new YoGraphicsListRegistry(), new YoVariableRegistry("dummy"));

      FullHumanoidRobotModel fullRobotModelAtInitialConfiguration = createFullRobotModelWithArmsAtMidRange();
      whik.updateRobotConfigurationData(extractRobotConfigurationData(fullRobotModelAtInitialConfiguration));
      whik.updateCapturabilityBasedStatus(createCapturabilityBasedStatus(true, true));

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody hand = desiredFullRobotModel.getHand(robotSide);
         KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage(hand, desiredPositions.get(robotSide));
         message.setWeight(20.0);
         commandInputManager.submitMessage(message);
      }

      commandInputManager.submitMessage(KinematicsToolboxMessageFactory.holdRigidBodyCurrentOrientation(desiredFullRobotModel.getChest()));

      int counter = 0;
      int maxIterations = 500;

      while (counter++ <= maxIterations)
      {
         whik.update();
         System.out.println(whik.getSolution().getSolutionQuality());
         snapGhostToFullRobotModel(desiredFullRobotModel);
         scs.tickAndUpdate();
      }

      if (whik.getSolution().getSolutionQuality() > 0.005)
      {
         return null;
      }

      SideDependentList<Pose3D> handPoses = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         Pose3D handPose = new Pose3D();
         handPoses.put(robotSide, handPose);
         RigidBody hand = desiredFullRobotModel.getHand(robotSide);
         RigidBodyTransform transformToWorldFrame = hand.getBodyFixedFrame().getTransformToWorldFrame();
         handPose.set(transformToWorldFrame);
      }
      return handPoses;
   }

   private static Pose3D computeCircleTrajectory(double time, double trajectoryTime, double circleRadius, Point3DReadOnly circleCenter,
                                                 Vector3DReadOnly circleAxis)
   {
      return computeCircleTrajectory(time, trajectoryTime, circleRadius, circleCenter, circleAxis, new Quaternion());
   }

   private static Pose3D computeCircleTrajectory(double time, double trajectoryTime, double circleRadius, Point3DReadOnly circleCenter,
                                                 Vector3DReadOnly circleAxis, QuaternionReadOnly constantOrientation)
   {
      return computeCircleTrajectory(time, trajectoryTime, circleRadius, circleCenter, circleAxis, constantOrientation, false, 0.0);
   }

   private static Pose3D computeCircleTrajectory(double time, double trajectoryTime, double circleRadius, Point3DReadOnly circleCenter,
                                                 Vector3DReadOnly circleAxis, QuaternionReadOnly constantOrientation, boolean ccw, double phase)
   {
      AxisAngle circleRotation = EuclidGeometryTools.axisAngleFromZUpToVector3D(circleAxis);

      double theta = (ccw ? -time : time) / trajectoryTime * 2.0 * Math.PI + phase;
      double x = circleRadius * Math.cos(theta);
      double y = circleRadius * Math.sin(theta);
      Point3D point = new Point3D(x, y, 0.0);
      circleRotation.transform(point);
      point.add(circleCenter);

      return new Pose3D(point, constantOrientation);
   }

   private static Graphics3DObject createFunctionTrajectoryVisualization(FunctionTrajectory trajectoryToVisualize, double t0, double tf, double timeResolution,
                                                                         double radius, AppearanceDefinition appearance)
   {
      int numberOfWaypoints = (int) Math.round((tf - t0) / timeResolution) + 1;
      double dT = (tf - t0) / (numberOfWaypoints - 1);

      int radialResolution = 16;
      SegmentedLine3DMeshDataGenerator segmentedLine3DMeshGenerator = new SegmentedLine3DMeshDataGenerator(numberOfWaypoints, radialResolution, radius);
      Point3DReadOnly[] waypoints = IntStream.range(0, numberOfWaypoints).mapToDouble(i -> t0 + i * dT).mapToObj(trajectoryToVisualize::compute)
                                             .map(pose -> new Point3D(pose.getPosition())).toArray(size -> new Point3D[size]);
      segmentedLine3DMeshGenerator.compute(waypoints);
      Graphics3DObject graphics = new Graphics3DObject();
      for (MeshDataHolder mesh : segmentedLine3DMeshGenerator.getMeshDataHolders())
      {
         graphics.addMeshData(mesh, appearance);
      }
      return graphics;
   }

   private void visualizeSolution(WholeBodyTrajectoryToolboxOutputStatus solution, double timeResolution) throws UnreasonableAccelerationException
   {
      hideRobot();
      robot.getControllers().clear();

      FullHumanoidRobotModel robotForViz = getRobotModel().createFullRobotModel();
      FloatingInverseDynamicsJoint rootJoint = robotForViz.getRootJoint();
      OneDoFJoint[] joints = FullRobotModelUtils.getAllJointsExcludingHands(robotForViz);

      double trajectoryTime = solution.getTrajectoryTime();

      double t = 0.0;

      while (t <= trajectoryTime)
      {
         t += timeResolution;
         KinematicsToolboxOutputStatus frame = findFrameFromTime(solution, t);
         frame.getDesiredJointState(rootJoint, joints);
         robotForViz.updateFrames();
         snapGhostToFullRobotModel(robotForViz);
         scs.simulateOneTimeStep();
      }
   }

   private KinematicsToolboxOutputStatus findFrameFromTime(WholeBodyTrajectoryToolboxOutputStatus outputStatus, double time)
   {
      if (time <= 0.0)
         return outputStatus.getRobotConfigurations()[0];

      else if (time >= outputStatus.getTrajectoryTime())
         return outputStatus.getLastRobotConfiguration();

      else
      {
         double timeGap = 0.0;

         int indexOfFrame = 0;
         int numberOfTrajectoryTimes = outputStatus.getTrajectoryTimes().length;

         for (int i = 0; i < numberOfTrajectoryTimes; i++)
         {
            timeGap = time - outputStatus.getTrajectoryTimes()[i];
            if (timeGap < 0)
            {
               indexOfFrame = i;
               break;
            }
         }

         KinematicsToolboxOutputStatus frameOne = outputStatus.getRobotConfigurations()[indexOfFrame - 1];
         KinematicsToolboxOutputStatus frameTwo = outputStatus.getRobotConfigurations()[indexOfFrame];

         double timeOne = outputStatus.getTrajectoryTimes()[indexOfFrame - 1];
         double timeTwo = outputStatus.getTrajectoryTimes()[indexOfFrame];

         double alpha = (time - timeOne) / (timeTwo - timeOne);

         return KinematicsToolboxOutputStatus.interpolateOutputStatus(frameOne, frameTwo, alpha);
      }
   }

   private FullHumanoidRobotModel createFullRobotModelAtInitialConfiguration()
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel initialFullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      robotModel.getDefaultRobotInitialSetup(0.0, 0.0).initializeRobot(robot, robotModel.getJointMap());
      DRCPerfectSensorReaderFactory drcPerfectSensorReaderFactory = new DRCPerfectSensorReaderFactory(robot, null, 0);
      drcPerfectSensorReaderFactory.build(initialFullRobotModel.getRootJoint(), null, null, null, null, null, null);
      drcPerfectSensorReaderFactory.getSensorReader().read();
      return initialFullRobotModel;
   }

   private FullHumanoidRobotModel createFullRobotModelWithArmsAtMidRange()
   {
      FullHumanoidRobotModel robot = createFullRobotModelAtInitialConfiguration();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody chest = robot.getChest();
         RigidBody hand = robot.getHand(robotSide);
         Arrays.stream(ScrewTools.createOneDoFJointPath(chest, hand)).forEach(j -> setJointPositionToMidRange(j));
      }
      return robot;
   }

   private static void setJointPositionToMidRange(OneDoFJoint joint)
   {
      double jointLimitUpper = joint.getJointLimitUpper();
      double jointLimitLower = joint.getJointLimitLower();
      joint.setQ(0.5 * (jointLimitUpper + jointLimitLower));
   }

   private WholeBodyTrajectoryToolboxOutputStatus runToolboxController(int maxNumberOfIterations) throws UnreasonableAccelerationException
   {
      AtomicReference<WholeBodyTrajectoryToolboxOutputStatus> status = new AtomicReference<>(null);
      statusOutputManager.attachStatusMessageListener(WholeBodyTrajectoryToolboxOutputStatus.class, status::set);

      initializationSucceeded.set(false);
      this.numberOfIterations.set(0);

      if (visualize)
      {
         for (int i = 0; !toolboxController.isDone() && i < maxNumberOfIterations; i++)
            scs.simulateOneTimeStep();
      }
      else
      {
         for (int i = 0; !toolboxController.isDone() && i < maxNumberOfIterations; i++)
            toolboxUpdater.doControl();
      }
      return status.getAndSet(null);
   }

   private RobotController createToolboxUpdater()
   {
      return new RobotController()
      {
         private final JointAnglesWriter jointAnglesWriter = new JointAnglesWriter(robot, toolboxController.getSolverFullRobotModel());

         @Override
         public void doControl()
         {
            if (!initializationSucceeded.getBooleanValue())
               initializationSucceeded.set(toolboxController.initialize());

            if (initializationSucceeded.getBooleanValue())
            {
               try
               {
                  toolboxController.updateInternal();
               }
               catch (InterruptedException | ExecutionException e)
               {
                  e.printStackTrace();
               }
               jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
               numberOfIterations.increment();
            }
         }

         @Override
         public void initialize()
         {
         }

         @Override
         public YoVariableRegistry getYoVariableRegistry()
         {
            return mainRegistry;
         }

         @Override
         public String getName()
         {
            return mainRegistry.getName();
         }

         @Override
         public String getDescription()
         {
            return null;
         }
      };
   }
}
