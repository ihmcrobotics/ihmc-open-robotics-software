package us.ihmc.avatar.behaviorTests.SolarPanel;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.pushDoor.PushDoor;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.pushDoor.PushDoorPose;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.specifiedspace.TaskNode3D;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.specifiedspace.TaskNodeTree;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.TaskNodeTreeVisualizer;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.tools.WheneverWholeBodyKinematicsSolver;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.trajectory.EndEffectorLinearTrajectory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.robotSide.RobotSide;
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

public abstract class SpecifiedWholeBodyMotionPlanningTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private boolean isKinematicsToolboxVisualizerEnabled = false;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;
   
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   SolarPanel solarPanel;
      
   private void setUpSolarPanel()
   {
      Pose poseSolarPanel = new Pose();
      Quaternion quaternionSolarPanel = new Quaternion();
      poseSolarPanel.setPosition(0.7, -0.2, 1.03);
      quaternionSolarPanel.appendYawRotation(Math.PI*0.00);
      quaternionSolarPanel.appendRollRotation(0.0);
      quaternionSolarPanel.appendPitchRotation(-0.380);
      poseSolarPanel.setOrientation(quaternionSolarPanel);
      
      solarPanel = new SolarPanel(poseSolarPanel, 0.6, 0.6);      
   }
   
   private void setupKinematicsToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, isKinematicsToolboxVisualizerEnabled);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT, PacketDestination.KINEMATICS_TOOLBOX_MODULE);
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

      if (kinematicsToolboxModule != null)
      {
         kinematicsToolboxModule.destroy();
         kinematicsToolboxModule = null;
      }

      if (toolboxCommunicator != null)
      {
         toolboxCommunicator.close();
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

//      drcBehaviorTestHelper = new DRCBehaviorTestHelper(environment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(environment, getSimpleRobotName(), DRCObstacleCourseStartingLocation.OFFSET_ONE_METER_X_AND_Y_ROTATED_PI, simulationTestingParameters, getRobotModel());

      setupKinematicsToolboxModule();
   }
   
   public ArrayList<Graphics3DObject> getXYZAxis(Pose pose)
   {      
      double axisHeight = 0.2;
      double axisRadius = 0.005;
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

      axisX.appendPitchRotation(Math.PI*0.5);            
      retX.rotate(axisX);
      retX.addCylinder(axisHeight, axisRadius, YoAppearance.Red());

      
      axisY.appendRollRotation(-Math.PI*0.5);
      retY.rotate(axisY);
      retY.addCylinder(axisHeight, axisRadius, YoAppearance.Green());

      ret.add(retX);
      ret.add(retY);
      ret.add(retZ);

      return ret;
   }
   
   private Graphics3DObject getGraphicsSphere(Pose pose)
   {  
      Graphics3DObject graphicsSphere = new Graphics3DObject();
      Point3D translation1 = new Point3D(pose.getPoint());
      graphicsSphere.translate(translation1);
      graphicsSphere.addSphere(0.02, YoAppearance.DarkGray());
      return graphicsSphere;
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
   
//   @Test
   public void testForEndEffectorTrajectory() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
   
      drcBehaviorTestHelper.updateRobotModel();
       
      Pose pose1 = new Pose(new Point3D(1.0, 1.0, 1.0), new Quaternion());
      Pose pose2 = new Pose(new Point3D(1.0, 2.0, 1.0), new Quaternion());
      Pose pose3 = new Pose(new Point3D(1.0, 2.0, 2.0), new Quaternion());
      Pose pose4 = new Pose(new Point3D(2.0, 2.0, 2.0), new Quaternion());
       
      EndEffectorLinearTrajectory constrainedEndEffectorPose = new EndEffectorLinearTrajectory();
       
      constrainedEndEffectorPose.setInitialPose(pose1);
      constrainedEndEffectorPose.addLinearTrajectory(pose2, 1.0);
      constrainedEndEffectorPose.addLinearTrajectory(pose3, 1.0);
      constrainedEndEffectorPose.addLinearTrajectory(pose4, 1.0);      
       
      System.out.println(constrainedEndEffectorPose.getEndEffectorPose(-1.0));
      System.out.println(constrainedEndEffectorPose.getEndEffectorPose(2.0));
      System.out.println(constrainedEndEffectorPose.getEndEffectorPose(2.5));
       
      System.out.println(constrainedEndEffectorPose.getEndEffectorPose(5.5));
   }
   
//   @Test
   public void testForWheneverWholeBodyKinematicsSolver() throws SimulationExceededMaximumTimeException, IOException
   {      
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();            
      drcBehaviorTestHelper.getControllerFullRobotModel().updateFrames();
      
      FullHumanoidRobotModel createdFullRobotModel;
      HumanoidReferenceFrames createdReferenceFrames;
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
                             
      /*
       * test 1 
       */
      // construct
      WheneverWholeBodyKinematicsSolver wbikTester = new WheneverWholeBodyKinematicsSolver(getRobotModel());
      
      // create initial robot configuration
      wbikTester.updateRobotConfigurationData(FullRobotModelUtils.getAllJointsExcludingHands(sdfFullRobotModel), sdfFullRobotModel.getRootJoint());      
      
      wbikTester.initialize();      
      wbikTester.holdCurrentTrajectoryMessages();
      
      // Desired
      Quaternion desiredHandOrientation = new Quaternion();
      desiredHandOrientation.appendPitchRotation(Math.PI*30/180);
      wbikTester.setDesiredHandPose(RobotSide.RIGHT, new Pose(new Point3D(0.6, -0.4, 1.0), desiredHandOrientation));
      wbikTester.setHandSelectionMatrixFree(RobotSide.LEFT);
      
      Quaternion desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendPitchRotation(Math.PI*10/180);
      wbikTester.setDesiredChestOrientation(desiredChestOrientation);
            
      wbikTester.setDesiredPelvisHeight(0.75);
      
      wbikTester.putTrajectoryMessages();

      PrintTools.info(""+wbikTester.isSolved());      
      
      /*
       * reversible test
       */
      
      wbikTester.initialize();      
      wbikTester.holdCurrentTrajectoryMessages();
      
      // Desired
      desiredHandOrientation = new Quaternion();
      desiredHandOrientation.appendPitchRotation(Math.PI*30/180);
      wbikTester.setDesiredHandPose(RobotSide.RIGHT, new Pose(new Point3D(0.5, -0.6, 1.1), desiredHandOrientation));
      wbikTester.setHandSelectionMatrixFree(RobotSide.LEFT);
      
      desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendPitchRotation(Math.PI*10/180);
      wbikTester.setDesiredChestOrientation(desiredChestOrientation);
            
      wbikTester.setDesiredPelvisHeight(0.75);
      
      wbikTester.putTrajectoryMessages();

      PrintTools.info(""+wbikTester.isSolved());      
      
      /*
       * reversible test
       */
      
      wbikTester.initialize();      
      wbikTester.holdCurrentTrajectoryMessages();
      
      // Desired
      desiredHandOrientation = new Quaternion();
      desiredHandOrientation.appendPitchRotation(Math.PI*30/180);
      wbikTester.setDesiredHandPose(RobotSide.RIGHT, new Pose(new Point3D(0.6, -0.4, 1.0), desiredHandOrientation));
      wbikTester.setHandSelectionMatrixFree(RobotSide.LEFT);
      
      desiredChestOrientation = new Quaternion();
      desiredChestOrientation.appendPitchRotation(Math.PI*10/180);
      wbikTester.setDesiredChestOrientation(desiredChestOrientation);
            
      wbikTester.setDesiredPelvisHeight(0.75);
      
      wbikTester.putTrajectoryMessages();

      PrintTools.info(""+wbikTester.isSolved());   
      
      /*
       * Show up
       */
      createdFullRobotModel = wbikTester.getDesiredFullRobotModel();
      createdReferenceFrames = new HumanoidReferenceFrames(createdFullRobotModel);
      
      wbikTester.printOutRobotModel(createdFullRobotModel, createdReferenceFrames.getMidFootZUpGroundFrame());
      showUpFullRobotModelWithConfiguration(createdFullRobotModel);
            
      PrintTools.info("END");     
   } 
      
//   @Test
   public void testForTaskNodeTree() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
   
      drcBehaviorTestHelper.updateRobotModel();
            
      TaskNode3D rootNode = new TaskNode3D();
      
      TaskNodeTree taskNodeTree = new TaskNodeTree(rootNode, "pelvisHeight", "chestYaw", "chestPitch");
      
      taskNodeTree.getTaskNodeRegion().setRandomRegion(0, 0.0, 10.0);
      taskNodeTree.getTaskNodeRegion().setRandomRegion(1, Math.PI*(-0.2), Math.PI*(0.2));
      taskNodeTree.getTaskNodeRegion().setRandomRegion(2, Math.PI*(-0.2), Math.PI*(0.2));
      taskNodeTree.getTaskNodeRegion().setRandomRegion(3, Math.PI*(-0.2), Math.PI*(0.2));
      
      System.out.println(taskNodeTree.getTrajectoryTime());
      
      taskNodeTree.expandTree(300);
      
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      
      TaskNodeTreeVisualizer taskNodeTreeVisualizer = new TaskNodeTreeVisualizer(scs, taskNodeTree);
      taskNodeTreeVisualizer.visualize();
   
      taskNodeTree.saveNodes();
      
      PrintTools.info("END");     
   } 
   
//   @Test
   public void testForposeForNode() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
            
      /*
       * Initialize tester.
       */
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      
      WheneverWholeBodyKinematicsSolver wbikTester = new WheneverWholeBodyKinematicsSolver(getRobotModel());
      
      wbikTester.updateRobotConfigurationData(FullRobotModelUtils.getAllJointsExcludingHands(sdfFullRobotModel), sdfFullRobotModel.getRootJoint());
                  
      TaskNode3D.nodeTester = wbikTester;

      /*
       * Define end effector trajectory.  
       */
      
      Pose pose1 = new Pose(new Point3D(0.6, -0.4, 0.9), new Quaternion());
      Pose pose2 = new Pose(new Point3D(0.6, -0.5, 0.9), new Quaternion());
      Pose pose3 = new Pose(new Point3D(0.6, -0.5, 1.1), new Quaternion());
      Pose pose4 = new Pose(new Point3D(0.6, -0.4, 1.1), new Quaternion());
      
      EndEffectorLinearTrajectory constrainedEndEffectorPose = new EndEffectorLinearTrajectory();
      
      constrainedEndEffectorPose.setInitialPose(pose1);
      constrainedEndEffectorPose.addLinearTrajectory(pose2, 3.0);
      constrainedEndEffectorPose.addLinearTrajectory(pose3, 3.0);
      constrainedEndEffectorPose.addLinearTrajectory(pose4, 3.0); 
      constrainedEndEffectorPose.setRobotSideOfEndEffector(RobotSide.RIGHT);
            
      TaskNode3D.endEffectorTrajectory = constrainedEndEffectorPose;      
      
      /*
       * Tree expanding.
       */
      
      double initialPelvisHeight = sdfFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
      
      TaskNode3D rootNode = new TaskNode3D(0.0, initialPelvisHeight, 0.0, 0.0);      
            
      TaskNode3D node1 = new TaskNode3D(2.0, initialPelvisHeight+0.02, Math.PI/180*10, Math.PI/180*10);
      TaskNode3D node2 = new TaskNode3D(5.0, initialPelvisHeight+0.03, Math.PI/180*10, Math.PI/180*10);
      TaskNode3D node3 = new TaskNode3D(3.0, initialPelvisHeight+0.03, Math.PI/180*10, Math.PI/180*10);
      
      TaskNode3D node4 = new TaskNode3D(3.5, initialPelvisHeight+0.03, Math.PI/180*10, Math.PI/180*10);
      TaskNode3D node5 = new TaskNode3D(4.0, initialPelvisHeight+0.03, Math.PI/180*10, Math.PI/180*10);
      TaskNode3D node6 = new TaskNode3D(5.0, initialPelvisHeight+0.03, Math.PI/180*10, Math.PI/180*10);
      
      node1.setParentNode(rootNode);
      node2.setParentNode(node1);
      node3.setParentNode(rootNode);
      node4.setParentNode(rootNode);
      node5.setParentNode(rootNode);
      node6.setParentNode(rootNode);
      
      System.out.println(rootNode.isValidNode());
      System.out.println(node1.isValidNode());
      System.out.println(node2.isValidNode());
      System.out.println(node3.isValidNode());
      System.out.println(node4.isValidNode());
      System.out.println(node5.isValidNode());
      System.out.println(node6.isValidNode());
            
      PrintTools.info("END");     
   } 
   
//   @Test
//   public void testForCleaningPlanning() throws SimulationExceededMaximumTimeException, IOException
//   {
//      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
//      assertTrue(success);
//
//      drcBehaviorTestHelper.updateRobotModel();
//      
//      /*
//       * Motion planning node.
//       * 
//       * Input @param.
//       * solar panel position and orientation.
//       * cleaning path. - robot side, via points, type(linear, circular_not yet).
//       * trajectory time for initial position to root node pose.
//       * finding initial node.
//       * 
//       * 
//       * Output @param.
//       * wholebody trajectory message.
//       * -> hand trajectory message(end effector) is obtained from end effector path.
//       * -> chest and pelvis trajectory message is obtained from planner.
//       */
//      
//      /*
//       * Initialize tester.
//       */
//      
//      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
//      sdfFullRobotModel.updateFrames();
//      
//      WheneverWholeBodyKinematicsSolver wbikTester = new WheneverWholeBodyKinematicsSolver(getRobotModel());
//      
//      wbikTester.updateRobotConfigurationData(FullRobotModelUtils.getAllJointsExcludingHands(sdfFullRobotModel), sdfFullRobotModel.getRootJoint());
//                  
//      TaskNode3D.nodeTester = wbikTester;
//
//      /*
//       * Wall to be cut define
//       */
//      
//      
//      
//      
//      /*
//       * Define end effector trajectory.  
//       */
//      
//      Quaternion desiredHandOrientation = new Quaternion();
//      desiredHandOrientation.appendPitchRotation(Math.PI*30/180);      
//      
//      Pose pose1 = .getPose();
//      Pose pose2 = .getPose();
//      Pose pose3 = .getPose();
//      Pose pose4 = .getPose();
//      
//      EndEffectorLinearTrajectory constrainedEndEffectorPose = new EndEffectorLinearTrajectory();
//      
//      constrainedEndEffectorPose.setInitialPose(pose1);
//      constrainedEndEffectorPose.addLinearTrajectory(pose2, 3.0);
//      constrainedEndEffectorPose.addLinearTrajectory(pose3, 3.0);
//      constrainedEndEffectorPose.addLinearTrajectory(pose4, 3.0); 
//      constrainedEndEffectorPose.setRobotSideOfEndEffector(RobotSide.RIGHT);
//            
//      TaskNode3D.endEffectorTrajectory = constrainedEndEffectorPose;      
//      
//      /*
//       * Tree expanding.
//       */
//      
//      double initialPelvisHeight = sdfFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23();
//      
//      TaskNode3D rootNode = new TaskNode3D(0.0, initialPelvisHeight, 0.0, 0.0);     
//      rootNode.setConfigurationJoints(sdfFullRobotModel);
//      
//      TaskNodeTree taskNodeTree = new TaskNodeTree(rootNode, "pelvisHeight", "chestYaw", "chestPitch");
//      
//      taskNodeTree.getTaskNodeRegion().setRandomRegion(0, 0.0, constrainedEndEffectorPose.getTrajectoryTime());
//      taskNodeTree.getTaskNodeRegion().setRandomRegion(1, 0.75, 0.9);
//      taskNodeTree.getTaskNodeRegion().setRandomRegion(2, Math.PI*(-0.2), Math.PI*(0.2));
//      taskNodeTree.getTaskNodeRegion().setRandomRegion(3, Math.PI*(-0.2), Math.PI*(0.2));
//      
//      System.out.println(taskNodeTree.getTrajectoryTime());
//      PrintTools.info(""+rootNode.isValidNode());
//      
//      FullHumanoidRobotModel createdFullRobotModel = wbikTester.getDesiredFullRobotModel();
//      HumanoidReferenceFrames createdReferenceFrames = new HumanoidReferenceFrames(createdFullRobotModel);
//      
//      wbikTester.printOutRobotModel(createdFullRobotModel, createdReferenceFrames.getMidFootZUpGroundFrame());
//      showUpFullRobotModelWithConfiguration(createdFullRobotModel);
//      
////      taskNodeTree.expandTree(30);
////      
//      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
//      
//      TaskNodeTreeVisualizer taskNodeTreeVisualizer = new TaskNodeTreeVisualizer(scs, taskNodeTree);
//      taskNodeTreeVisualizer.visualize();
//   
//      taskNodeTree.saveNodes();
//      
//      
//      /*
//       * Debug.
//       */
//      
//      scs.addStaticLinkGraphics(getGraphicsSphere(pose1));
//      scs.addStaticLinkGraphics(getGraphicsSphere(pose2));
//      scs.addStaticLinkGraphics(getGraphicsSphere(pose3));
//      scs.addStaticLinkGraphics(getGraphicsSphere(pose4));
//   }
      
   
   @Test
   public void testForPushDoorKinematics() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
   
      drcBehaviorTestHelper.updateRobotModel();
            
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(sdfFullRobotModel);
      referenceFrames.updateFrames();
      
      Point3D pushDoorLocation = new Point3D(1.4, -0.5, 0.0);
      Quaternion pushDoorOrientation = new Quaternion();
      pushDoorOrientation.appendYawRotation(Math.PI/180*10);
      FramePose pushDoorFramePose = new FramePose(referenceFrames.getMidFootZUpGroundFrame(), new Pose(pushDoorLocation, pushDoorOrientation));
      PushDoor pushDoor = new PushDoor(pushDoorFramePose, 1.0, 0.9);
      
      
      PushDoorPose pushDoorPose1 = new PushDoorPose(pushDoor, 0.0*Math.PI/180, 0.0*Math.PI/180);
      PushDoorPose pushDoorPose2 = new PushDoorPose(pushDoor, -10.0*Math.PI/180, 0.0*Math.PI/180);
      PushDoorPose pushDoorPose3 = new PushDoorPose(pushDoor, -20.0*Math.PI/180, 0.0*Math.PI/180);
      PushDoorPose pushDoorPose4 = new PushDoorPose(pushDoor, -20.0*Math.PI/180, 10.0*Math.PI/180);
      PushDoorPose pushDoorPose5 = new PushDoorPose(pushDoor, -20.0*Math.PI/180, -20.0*Math.PI/180);
      
      
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      
      scs.addStaticLinkGraphics(pushDoor.getGraphics());
      scs.addStaticLinkGraphics(getXYZAxis(pushDoorPose1.getEndEffectorPose()));
      scs.addStaticLinkGraphics(getXYZAxis(pushDoorPose2.getEndEffectorPose()));
      scs.addStaticLinkGraphics(getXYZAxis(pushDoorPose3.getEndEffectorPose()));
      scs.addStaticLinkGraphics(getXYZAxis(pushDoorPose4.getEndEffectorPose()));
      scs.addStaticLinkGraphics(getXYZAxis(pushDoorPose5.getEndEffectorPose()));
      
      PrintTools.info("END");     
   } 
   

   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
}
