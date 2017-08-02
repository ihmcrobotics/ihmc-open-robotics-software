package us.ihmc.avatar.behaviorTests.rrtPlannerTests;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.rrtToolboxModule.ConstrainedWholeBodyPlanningToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commonWalkingControlModules.trajectories.InitialClearancePoseTrajectoryGenerator;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.ToolboxStateMessage.ToolboxState;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicsBehavior;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ConstrainedWholeBodyPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ConfigurationBuildOrder;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ConfigurationBuildOrder.ConfigurationSpaceName;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ConfigurationSpace;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.ConstrainedWholeBodyPlanningToolboxHelper;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.DrawingTrajectory;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.GenericTaskNode;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.CTTaskNodeTreeVisualizer;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class ConstrainedWholeBodyPlanningToolboxTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   
   private static boolean visulaizerOn = false;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private ConstrainedWholeBodyPlanningToolboxModule cwbPlanningToolboxModule;

   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;

   private void setupCWBPlanningToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, false);
      cwbPlanningToolboxModule = new ConstrainedWholeBodyPlanningToolboxModule(robotModel, fullRobotModel, null, visulaizerOn);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT,
                                                                                   PacketDestination.KINEMATICS_TOOLBOX_MODULE);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE_PORT,
                                                                                   PacketDestination.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE);

   }

   private void showUpFullRobotModelWithConfiguration(FullHumanoidRobotModel createdFullRobotModel) throws SimulationExceededMaximumTimeException
   {
      for (int i = 0; i < createdFullRobotModel.getOneDoFJoints().length; i++)
      {
         double jointPosition = createdFullRobotModel.getOneDoFJoints()[i].getQ();
         Joint scsJoint = drcBehaviorTestHelper.getRobot().getJoint(createdFullRobotModel.getOneDoFJoints()[i].getName());

         if (scsJoint instanceof PinJoint)
         {
            PinJoint pinJoint = (PinJoint) scsJoint;
            pinJoint.setQ(jointPosition);
         }
         else
         {
            PrintTools.info(createdFullRobotModel.getOneDoFJoints()[i].getName() + " was not a PinJoint.");
         }
      }

      FloatingJoint scsRootJoint = drcBehaviorTestHelper.getRobot().getRootJoint();
      scsRootJoint.setQuaternion(new Quaternion(createdFullRobotModel.getRootJoint().getRotationForReading()));
      scsRootJoint.setPosition(new Point3D(createdFullRobotModel.getRootJoint().getTranslationForReading()));

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.005);
   }

   public ArrayList<Graphics3DObject> getXYZAxis(Pose3D pose)
   {
      double axisHeight = 0.1;
      double axisRadius = 0.01;
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      Graphics3DObject retX = new Graphics3DObject();
      Graphics3DObject retY = new Graphics3DObject();
      Graphics3DObject retZ = new Graphics3DObject();

      Point3D centerPoint = new Point3D(pose.getPosition());

      retX.translate(centerPoint);
      retY.translate(centerPoint);
      retZ.translate(centerPoint);

      RotationMatrix axisOrientation = new RotationMatrix(pose.getOrientation());

      RotationMatrix axisX = new RotationMatrix(axisOrientation);
      RotationMatrix axisY = new RotationMatrix(axisOrientation);
      RotationMatrix axisZ = new RotationMatrix(axisOrientation);

      retZ.rotate(axisZ);
      retZ.addCylinder(axisHeight, axisRadius, YoAppearance.Blue());

      axisX.appendPitchRotation(Math.PI * 0.5);
      retX.rotate(axisX);
      retX.addCylinder(axisHeight, axisRadius, YoAppearance.Red());

      axisY.appendRollRotation(-Math.PI * 0.5);
      retY.rotate(axisY);
      retY.addCylinder(axisHeight, axisRadius, YoAppearance.Green());

      ret.add(retX);
      ret.add(retY);
      ret.add(retZ);

      return ret;
   }

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (visualize)
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      if (cwbPlanningToolboxModule != null)
      {
         cwbPlanningToolboxModule.destroy();
         cwbPlanningToolboxModule = null;
      }

      if (kinematicsToolboxModule != null)
      {
         kinematicsToolboxModule.destroy();
         kinematicsToolboxModule = null;
      }

      if (toolboxCommunicator != null)
      {
         toolboxCommunicator.closeConnection();
         toolboxCommunicator = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Before
   public void setUp() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      CommonAvatarEnvironmentInterface environment = new FlatGroundEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(environment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());

      setupCWBPlanningToolboxModule();
   }
   
   @Test
   public void testForToolboxPacket() throws SimulationExceededMaximumTimeException, IOException
   {
      if(visulaizerOn)
         ThreadTools.sleep(10000);
      
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      System.out.println("Start");
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      System.out.println("Send wakeup " + drcBehaviorTestHelper.getYoTime());
      ToolboxStateMessage toolboxMessage;
      
      toolboxMessage = new ToolboxStateMessage(ToolboxState.WAKE_UP);
      toolboxMessage.setDestination(PacketDestination.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE);
      toolboxCommunicator.send(toolboxMessage);
      System.out.println("Send wakeup done " + drcBehaviorTestHelper.getYoTime());
      
//      /*
//       * reaching initial configuration
//       */
//      Quaternion initialOrientation = new Quaternion();
//      initialOrientation.appendRollRotation(Math.PI*0.5);
//      initialOrientation.appendYawRotation(Math.PI*0.5);
//      initialOrientation.appendPitchRotation(-Math.PI*0.0);
//      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 3.0, new Point3D(0.6, 0.35, 1.0), initialOrientation, referenceFrames.getMidFootZUpGroundFrame());
//      drcBehaviorTestHelper.send(handTrajectoryMessage);
//      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      
      /*
       * constrained end effector trajectory (WorldFrame).
       */
      System.out.println("Send packet " + drcBehaviorTestHelper.getYoTime());
      DrawingTrajectory endeffectorTrajectory = new DrawingTrajectory(10.0, RobotSide.LEFT);
      
      ConstrainedWholeBodyPlanningToolboxHelper.setConstrainedEndEffectorTrajectory(endeffectorTrajectory);
      ConstrainedWholeBodyPlanningToolboxHelper.setInitialFullRobotModel(drcBehaviorTestHelper.getControllerFullRobotModel());
      ConstrainedWholeBodyPlanningToolboxHelper.setFullRobotModelFactory(getRobotModel());
      
      ConstrainedWholeBodyPlanningRequestPacket packet = new ConstrainedWholeBodyPlanningRequestPacket();      
      packet.setTempValue(0.2);      
      packet.setNumberOfExpanding(1000);   
      packet.setDestination(PacketDestination.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE);
      
      toolboxCommunicator.send(packet);
      System.out.println("Send packet done" + drcBehaviorTestHelper.getYoTime());
      
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(10.0);
      System.out.println("End");
   }
   
//   @Test
   public void testForInverseKinematicsToolbox() throws SimulationExceededMaximumTimeException, IOException
   {
      ThreadTools.sleep(10000);
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      System.out.println("Start");

      
      RobotSide robotSide = RobotSide.RIGHT;

      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(getRobotModel(), drcBehaviorTestHelper.getYoTime(),
                                                                                     drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                                     drcBehaviorTestHelper.getSDFFullRobotModel());

      ReferenceFrame handControlFrame = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(robotSide);

      FullHumanoidRobotModel fullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      RigidBody chest = fullRobotModel.getChest();
      ReferenceFrame chestControlFrame = chest.getBodyFixedFrame();
      FrameOrientation initialChestOrientation = new FrameOrientation(chestControlFrame);
      initialChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      ReferenceFrame pelvisControlFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      FrameOrientation initialPelvisOrientation = new FrameOrientation(pelvisControlFrame);
      initialPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose desiredHandPose = new FramePose(handControlFrame);
      desiredHandPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHandPose.prependTranslation(0.50, 0.0, 0.3);
      ik.setTrajectoryTime(0.5);
      ik.setDesiredHandPose(robotSide, desiredHandPose);
      ik.holdCurrentChestOrientation();
      ik.holdCurrentPelvisOrientation();
      ik.holdCurrentPelvisHeight();

      drcBehaviorTestHelper.updateRobotModel();
      FramePose desiredHandPoseCopy = new FramePose(desiredHandPose);
      ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      desiredHandPoseCopy.changeFrame(chestFrame);

      drcBehaviorTestHelper.dispatchBehavior(ik);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(15.0);

      System.out.println("End");
   }
   
//   @Test
   public void testForRandomNodeRegion() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      System.out.println("Start");
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();
      
      /*
       * reaching initial configuration
       */
      Quaternion initialOrientation = new Quaternion();
      initialOrientation.appendRollRotation(Math.PI*0.5);
      initialOrientation.appendYawRotation(Math.PI*0.5);
      initialOrientation.appendPitchRotation(-Math.PI*0.3);
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 3.0, new Point3D(0.6, 0.35, 1.0), initialOrientation, referenceFrames.getMidFootZUpGroundFrame());
      drcBehaviorTestHelper.send(handTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      
      /*
       * constrained end effector trajectory (WorldFrame).
       */
      DrawingTrajectory endeffectorTrajectory = new DrawingTrajectory(10.0, RobotSide.LEFT);

      GenericTaskNode.constrainedEndEffectorTrajectory = endeffectorTrajectory;

      /*
       * tester
       */
      sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      sdfFullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      WheneverWholeBodyKinematicsSolver wbikTester = new WheneverWholeBodyKinematicsSolver(getRobotModel(), sdfFullRobotModel);

      GenericTaskNode.nodeTester = wbikTester;
      GenericTaskNode.midZUpFrame = referenceFrames.getMidFootZUpGroundFrame();

      /*
       * put on generic task node
       */
      double initialPelvisHeight = sdfFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
      GenericTaskNode rootNode = new GenericTaskNode(0.0, initialPelvisHeight, -10.0/180*Math.PI, 0.0, 0.0);
      rootNode.setNodeData(10, -70.0/180 * Math.PI);
      rootNode.setConfigurationJoints(sdfFullRobotModel);      
      rootNode.isValidNode();
      
      CTTaskNodeTree tree = new CTTaskNodeTree(rootNode);
      
      tree.getTaskNodeRegion().setRandomRegion(0, 0.0, endeffectorTrajectory.getTrajectoryTime());
      tree.getTaskNodeRegion().setRandomRegion(1, 0.75, 0.90);
      tree.getTaskNodeRegion().setRandomRegion(2, -20.0/180*Math.PI, 20.0/180*Math.PI);
      tree.getTaskNodeRegion().setRandomRegion(3, -20.0/180*Math.PI, 20.0/180*Math.PI);
      tree.getTaskNodeRegion().setRandomRegion(4, -5.0/180*Math.PI, 5.0/180*Math.PI);
      tree.getTaskNodeRegion().setRandomRegion(5, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(6, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(7, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(8, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(9, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(10, -90.0/180*Math.PI, 90.0/180*Math.PI);
            
      tree.testMonteCarlo(50);

      CTTaskNodeTreeVisualizer taskNodeTreeVisualizer = new CTTaskNodeTreeVisualizer(scs, tree);
      taskNodeTreeVisualizer.visualize();
      
      System.out.println("End");
   }

//   @Test
   public void testForNodeExpanding() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      System.out.println("Start");
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();
      
      /*
       * reaching initial configuration
       */
      Quaternion initialOrientation = new Quaternion();
      initialOrientation.appendRollRotation(Math.PI*0.5);
      initialOrientation.appendYawRotation(Math.PI*0.5);
      initialOrientation.appendPitchRotation(-Math.PI*0.3);
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 3.0, new Point3D(0.6, 0.35, 1.0), initialOrientation, referenceFrames.getMidFootZUpGroundFrame());
      drcBehaviorTestHelper.send(handTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      
      /*
       * constrained end effector trajectory (WorldFrame).
       */
      DrawingTrajectory endeffectorTrajectory = new DrawingTrajectory(10.0, RobotSide.LEFT);

      GenericTaskNode.constrainedEndEffectorTrajectory = endeffectorTrajectory;

      /*
       * tester
       */
      sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      sdfFullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      WheneverWholeBodyKinematicsSolver wbikTester = new WheneverWholeBodyKinematicsSolver(getRobotModel(), sdfFullRobotModel);

      GenericTaskNode.nodeTester = wbikTester;
      GenericTaskNode.midZUpFrame = referenceFrames.getMidFootZUpGroundFrame();

      /*
       * put on generic task node
       */
      double initialPelvisHeight = sdfFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
      GenericTaskNode rootNode = new GenericTaskNode(0.0, initialPelvisHeight, -10.0/180*Math.PI, 0.0, 0.0);
      rootNode.setNodeData(10, -70.0/180 * Math.PI);
      rootNode.setConfigurationJoints(sdfFullRobotModel);      
      rootNode.isValidNode();
      
      CTTaskNodeTree tree = new CTTaskNodeTree(rootNode);
      
      tree.getTaskNodeRegion().setRandomRegion(0, 0.0, endeffectorTrajectory.getTrajectoryTime());
      tree.getTaskNodeRegion().setRandomRegion(1, 0.75, 0.90);
      tree.getTaskNodeRegion().setRandomRegion(2, -20.0/180*Math.PI, 20.0/180*Math.PI);
      tree.getTaskNodeRegion().setRandomRegion(3, -20.0/180*Math.PI, 20.0/180*Math.PI);
      tree.getTaskNodeRegion().setRandomRegion(4, -5.0/180*Math.PI, 5.0/180*Math.PI);
      tree.getTaskNodeRegion().setRandomRegion(5, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(6, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(7, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(8, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(9, 0.0, 0.0);
      tree.getTaskNodeRegion().setRandomRegion(10, -90.0/180*Math.PI, 90.0/180*Math.PI);
            
      tree.expandTree(500);

      CTTaskNodeTreeVisualizer taskNodeTreeVisualizer = new CTTaskNodeTreeVisualizer(scs, tree);
      taskNodeTreeVisualizer.visualize();
      
      System.out.println("End");
   }

    //  @Test
   public void testForPoseOfGenericTaskNode() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      System.out.println("Start");

      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();
      
      /*
       * reaching initial configuration
       */
      Quaternion initialOrientation = new Quaternion();
      initialOrientation.appendRollRotation(Math.PI*0.5);
      initialOrientation.appendYawRotation(Math.PI*0.5);
      initialOrientation.appendPitchRotation(-Math.PI*0.3);
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.LEFT, 3.0, new Point3D(0.6, 0.35, 1.0), initialOrientation, referenceFrames.getMidFootZUpGroundFrame());
      drcBehaviorTestHelper.send(handTrajectoryMessage);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      
      /*
       * constrained end effector trajectory (WorldFrame).
       */
      DrawingTrajectory endeffectorTrajectory = new DrawingTrajectory(10.0, RobotSide.LEFT);

      GenericTaskNode.constrainedEndEffectorTrajectory = endeffectorTrajectory;

      /*
       * tester
       */
      sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();

      sdfFullRobotModel.updateFrames();
      referenceFrames.updateFrames();
      WheneverWholeBodyKinematicsSolver wbikTester = new WheneverWholeBodyKinematicsSolver(getRobotModel(), sdfFullRobotModel);

      GenericTaskNode.nodeTester = wbikTester;
      GenericTaskNode.midZUpFrame = referenceFrames.getMidFootZUpGroundFrame();

      /*
       * put on generic task node
       */
      double initialPelvisHeight = sdfFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
      GenericTaskNode rootNode = new GenericTaskNode(0.0, initialPelvisHeight, 0.0, 0.0, 0.0);
      rootNode.setConfigurationJoints(sdfFullRobotModel);
      rootNode.setNodeData(10, -0.4 * Math.PI); // is selected
      rootNode.setNodeData(9, -0.25 * Math.PI); // will be ignored.

      GenericTaskNode node1 = new GenericTaskNode(1.0, initialPelvisHeight, -20.0/180 * Math.PI, 0.0/180 * Math.PI, 0.0 * Math.PI);
      node1.setNodeData(10, -70.0/180 * Math.PI);
      node1.setParentNode(rootNode);
      
      
      GenericTaskNode node2 = new GenericTaskNode(2.0, initialPelvisHeight, -20.0/180 * Math.PI, 0.0/180 * Math.PI, 0.0 * Math.PI);
      node2.setNodeData(10, -70.0/180 * Math.PI);
      node2.setParentNode(node1);
      
      GenericTaskNode node3 = new GenericTaskNode(3.0, initialPelvisHeight, -20.0/180 * Math.PI, 0.0/180 * Math.PI, 0.0 * Math.PI);
      node3.setNodeData(10, -70.0/180 * Math.PI);
      node3.setParentNode(node2);
      
      GenericTaskNode node4 = new GenericTaskNode(4.0, initialPelvisHeight, -20.0/180 * Math.PI, 0.0/180 * Math.PI, 0.0 * Math.PI);
      node4.setNodeData(10, -70.0/180 * Math.PI);
      node4.setParentNode(node3);
      

      System.out.println(rootNode.isValidNode());
      System.out.println(node1.isValidNode());
      System.out.println(node2.isValidNode());
      System.out.println(node3.isValidNode());
      System.out.println(node4.isValidNode());

      /*
       * show the ik result
       */
      FullHumanoidRobotModel createdFullRobotModel;
      HumanoidReferenceFrames createdReferenceFrames;

      createdFullRobotModel = wbikTester.getDesiredFullRobotModel();
      createdReferenceFrames = new HumanoidReferenceFrames(createdFullRobotModel);

      wbikTester.printOutRobotModel(createdFullRobotModel, createdReferenceFrames.getMidFootZUpGroundFrame());
      showUpFullRobotModelWithConfiguration(createdFullRobotModel);

      scs.addStaticLinkGraphics(getXYZAxis(node4.getEndEffectorPose()));
      scs.addStaticLinkGraphics(getXYZAxis(node3.getEndEffectorPose()));
      scs.addStaticLinkGraphics(getXYZAxis(node2.getEndEffectorPose()));
      scs.addStaticLinkGraphics(getXYZAxis(node1.getEndEffectorPose()));
      scs.addStaticLinkGraphics(getXYZAxis(rootNode.getEndEffectorPose()));

      System.out.println("End");
   }

   //   @Test
   public void testForConstrainedTrajectory() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      System.out.println("Start");

      /*
       * constrained end effector trajectory
       */

      DrawingTrajectory endeffectorTrajectory = new DrawingTrajectory(10.0, RobotSide.LEFT);

      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(0.0, new ConfigurationSpace())));

      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(3.0, new ConfigurationSpace())));

      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(6.0, new ConfigurationSpace())));

      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(9.0, new ConfigurationSpace())));

      scs.addStaticLinkGraphics(getXYZAxis(endeffectorTrajectory.getEndEffectorPose(10.0, new ConfigurationSpace())));

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      System.out.println("End");
   }

   //   @Test
   public void testForConfigurationSpace() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      System.out.println("Start");

      Point3D translation;
      Quaternion orientation;
      Pose3D pose3D;

      ConfigurationBuildOrder configurationBuildOrder;
      ConfigurationSpace configurationSpace;

      configurationBuildOrder = new ConfigurationBuildOrder(ConfigurationSpaceName.X, ConfigurationSpaceName.Y, ConfigurationSpaceName.Z,
                                                            ConfigurationSpaceName.YAW, ConfigurationSpaceName.PITCH, ConfigurationSpaceName.ROLL);

      configurationSpace = new ConfigurationSpace();
      configurationSpace.setTranslation(0.5, 0.0, 1.0);
      configurationSpace.setRotation(Math.PI / 180 * 30, 0, 0);

      translation = new Point3D(configurationSpace.createRigidBodyTransform(configurationBuildOrder).getTranslationVector());
      orientation = new Quaternion(configurationSpace.createRigidBodyTransform(configurationBuildOrder).getRotationMatrix());
      pose3D = new Pose3D(translation, orientation);

      scs.addStaticLinkGraphics(getXYZAxis(pose3D));

      configurationSpace = new ConfigurationSpace();
      configurationSpace.setTranslation(1.0, 0.0, 1.0);
      configurationSpace.setRotation(Math.PI / 180 * 30, Math.PI / 180 * 30, 0);

      translation = new Point3D(configurationSpace.createRigidBodyTransform(configurationBuildOrder).getTranslationVector());
      orientation = new Quaternion(configurationSpace.createRigidBodyTransform(configurationBuildOrder).getRotationMatrix());
      pose3D = new Pose3D(translation, orientation);

      scs.addStaticLinkGraphics(getXYZAxis(pose3D));

      configurationSpace = new ConfigurationSpace();
      configurationSpace.setTranslation(1.5, 0.0, 1.0);
      configurationSpace.setRotation(Math.PI / 180 * 30, 0, Math.PI / 180 * 30);

      translation = new Point3D(configurationSpace.createRigidBodyTransform(configurationBuildOrder).getTranslationVector());
      orientation = new Quaternion(configurationSpace.createRigidBodyTransform(configurationBuildOrder).getRotationMatrix());
      pose3D = new Pose3D(translation, orientation);

      scs.addStaticLinkGraphics(getXYZAxis(pose3D));

      System.out.println("End");
   }

//   @Test
   public void testForToolboxMessage() throws SimulationExceededMaximumTimeException, IOException
   {
      ThreadTools.sleep(10000);
      
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      System.out.println("Start");

      System.out.println("Send wake up " + drcBehaviorTestHelper.getYoTime());

      ToolboxStateMessage toolboxMessage;
      toolboxMessage = new ToolboxStateMessage(ToolboxState.WAKE_UP);
      toolboxMessage.setDestination(PacketDestination.CONSTRAINED_WHOLE_BODY_PLANNING_TOOLBOX_MODULE);
      toolboxCommunicator.send(toolboxMessage);

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      System.out.println("Send input " + drcBehaviorTestHelper.getYoTime());

      ConstrainedWholeBodyPlanningRequestPacket requestPacket = new ConstrainedWholeBodyPlanningRequestPacket();
      requestPacket.setTempValue(3.1);
      toolboxCommunicator.send(requestPacket);

      System.out.println("Send input Done " + drcBehaviorTestHelper.getYoTime());

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      System.out.println("End");
   }

}
