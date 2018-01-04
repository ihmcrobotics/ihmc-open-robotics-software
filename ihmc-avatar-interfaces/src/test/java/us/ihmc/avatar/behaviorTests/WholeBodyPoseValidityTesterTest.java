package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.CleaningMotionStateMachineBehavior;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.ControlPointOptimizationStateMachineBehavior;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.SolarPanelCleaningInfo;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.TimeDomain1DNode;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.TimeDomain3DNode;
import us.ihmc.humanoidBehaviors.behaviors.rrtPlanner.ValidNodesStateMachineBehavior;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.RRTNode1DTimeDomain;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.RRTPlannerSolarPanelCleaning;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.RRTTreeTimeDomain;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.SolarPanelMotionPlanner;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.SolarPanelMotionPlanner.CleaningMotion;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.SolarPanelWholeBodyTrajectoryMessageFacotry;
import us.ihmc.humanoidBehaviors.behaviors.wholebodyValidityTester.SolarPanelPoseValidityTester;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.manipulation.planning.robotcollisionmodel.RobotCollisionModel;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelLinearPath;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;

public abstract class WholeBodyPoseValidityTesterTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private boolean isKinematicsToolboxVisualizerEnabled = false;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;
   
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   SolarPanel solarPanel;
   
   public class SolarPanelCleaningEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D EnvSet;

      public SolarPanelCleaningEnvironment()
      {
         setUpSolarPanel();
         
         EnvSet = DefaultCommonAvatarEnvironment.setUpGround("Ground");
                  
         EnvSet.addRotatableBox(solarPanel.getRigidBodyTransform(), solarPanel.getSizeX(), solarPanel.getSizeY(), solarPanel.getSizeZ(), YoAppearance.Aqua());
      }
      
      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return EnvSet;
      }

      @Override
      public List<? extends Robot> getEnvironmentRobots()
      {
         return null;
      }

      @Override
      public void createAndSetContactControllerToARobot()
      {

      }

      @Override
      public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
      {

      }

      @Override
      public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
      {
      }
   }
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
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
         toolboxCommunicator.disconnect();
         toolboxCommunicator.closeConnection();
         toolboxCommunicator = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Before
   public void setUp() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      CommonAvatarEnvironmentInterface envrionment = new SolarPanelCleaningEnvironment();

      //drcBehaviorTestHelper = new DRCBehaviorTestHelper(envrionment, getSimpleRobotName(), DRCObstacleCourseStartingLocation.STAIRS, simulationTestingParameters, getRobotModel());
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(envrionment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());

      setupKinematicsToolboxModule();
   }
    
   public void testACleaningMotion() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      drcBehaviorTestHelper.updateRobotModel();
      
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      for(int i=0;i<sdfFullRobotModel.getOneDoFJoints().length;i++)
      {
         OneDoFJoint aJoint = sdfFullRobotModel.getOneDoFJoints()[i];
         PrintTools.info(""+aJoint.getName()+" "+aJoint.getJointLimitLower()+" "+aJoint.getJointLimitUpper());
      }
            
            
      setUpSolarPanel();
            
      RRTNode1DTimeDomain.nodeValidityTester = new SolarPanelPoseValidityTester(getRobotModel(), 
                                                                                drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                                sdfFullRobotModel, drcBehaviorTestHelper.getReferenceFrames());
      
      RRTNode1DTimeDomain.nodeValidityTester.setSolarPanel(solarPanel);
      
      drcBehaviorTestHelper.dispatchBehavior(RRTNode1DTimeDomain.nodeValidityTester);
      
      // ********** Planning *** //      
      double motionTime = 0;
      
      WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
      
      SolarPanelMotionPlanner solarPanelPlanner = new SolarPanelMotionPlanner(solarPanel);
          
      if(solarPanelPlanner.setWholeBodyTrajectoryMessage(CleaningMotion.ReadyPose) == true)
      {
         wholeBodyTrajectoryMessage = solarPanelPlanner.getWholeBodyTrajectoryMessage();
         motionTime = solarPanelPlanner.getMotionTime();
         drcBehaviorTestHelper.send(wholeBodyTrajectoryMessage);         
      }      

      Assert.assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(motionTime + 1.0));

      if(solarPanelPlanner.setWholeBodyTrajectoryMessage(CleaningMotion.LinearCleaningMotion) == true)
      {
         wholeBodyTrajectoryMessage = solarPanelPlanner.getWholeBodyTrajectoryMessage();
         motionTime = solarPanelPlanner.getMotionTime();
         drcBehaviorTestHelper.send(wholeBodyTrajectoryMessage);
      }      
      Assert.assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(motionTime));
      
      scs.addStaticLinkGraphics(getPrintCleaningPath(RRTNode1DTimeDomain.cleaningPath));

      if (visualize)
      {
         // ************************************* //
         // show
         // ************************************* //
         JFrame frame;
         DrawPanel drawPanel;
         Dimension dim;


         frame = new JFrame("RRTTest");
         drawPanel = new DrawPanel(solarPanelPlanner.rrtPlanner);
         dim = new Dimension(1600, 800);
         frame.setPreferredSize(dim);
         frame.setLocation(200, 100);

         frame.add(drawPanel);
         frame.pack();
         frame.setVisible(true);
      }
      
      // ************************************* //
      // show
      // ************************************* //
      PrintTools.info("END");     
   }   
   
   //@Test(timeout = 30000)
   public void showUpRobotCollisionModel() throws SimulationExceededMaximumTimeException, IOException
   {  
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      drcBehaviorTestHelper.updateRobotModel();
      
      
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
            
      RobotCollisionModel robotCollisionModel;
      robotCollisionModel = new RobotCollisionModel(sdfFullRobotModel);
      
      scs.addStaticLinkGraphics(robotCollisionModel.getCollisionGraphics());
      
      PrintTools.info("cur Height is "+ sdfFullRobotModel.getPelvis().getBodyFixedFrame().getTransformToWorldFrame().getM23());
      
      PrintTools.info("cur Height is "+ sdfFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getM23());
      
      
      PrintTools.info("Out " );      
   }
   
   //@Test(timeout = 30000)
   public void validNodesStateMachineBehaviorTest() throws SimulationExceededMaximumTimeException, IOException
   {
      if(isKinematicsToolboxVisualizerEnabled)
         ThreadTools.sleep(13000);
      
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
            
      SolarPanelCleaningPose readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.15, -Math.PI*0.2);
      SolarPanelPath cleaningPath = new SolarPanelPath(readyPose);
      
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.15, -Math.PI*0.3), 4.0);         
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.2, -0.15, -Math.PI*0.3), 1.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.2, -0.15, -Math.PI*0.2), 4.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.3, -0.15, -Math.PI*0.2), 1.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.3, -0.15, -Math.PI*0.3), 4.0);
      
      SolarPanelCleaningInfo.setCleaningPath(cleaningPath);
      
      ArrayList<RRTNode> nodes1D = new ArrayList<RRTNode>();
      
      nodes1D.add(new TimeDomain1DNode(0.0, 0.0));            // true
      nodes1D.add(new TimeDomain1DNode(0.3, Math.PI*0.1));    // true
      nodes1D.add(new TimeDomain1DNode(0.4, -Math.PI*0.2));   // false
      nodes1D.add(new TimeDomain1DNode(1.7, Math.PI*0.2));    // ture
      nodes1D.add(new TimeDomain1DNode(0.3, Math.PI*0.2));    // ture
      nodes1D.add(new TimeDomain1DNode(0.0, 0.0));
      
      ArrayList<RRTNode> nodes3D = new ArrayList<RRTNode>();
      
      nodes3D.add(new TimeDomain3DNode(0.0, 0.7, Math.PI*0.1, Math.PI*0.1));            // true
      nodes3D.add(new TimeDomain3DNode(3.0, 0.75, -Math.PI*0.1, -Math.PI*0.1));            // true
      nodes3D.add(new TimeDomain3DNode(5.0, 0.73, Math.PI*0.15, -Math.PI*0.1));            // true
      nodes3D.add(new TimeDomain3DNode(2.0, 0.70, Math.PI*0.0, Math.PI*0.1));            // true
      nodes3D.add(new TimeDomain3DNode(5.0, 0.78, -Math.PI*0.1, -Math.PI*0.1));            // true
      nodes3D.add(new TimeDomain3DNode(1.0, 0.60, Math.PI*0.13, Math.PI*0.1));            // true      
      
      ValidNodesStateMachineBehavior testNodesBehavior = new ValidNodesStateMachineBehavior(nodes3D, drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                                                    drcBehaviorTestHelper.getYoTime(), getRobotModel(), sdfFullRobotModel, drcBehaviorTestHelper.getReferenceFrames());
      testNodesBehavior.setSolarPanel(solarPanel);
      
      drcBehaviorTestHelper.dispatchBehavior(testNodesBehavior);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(5.0);
      PrintTools.info("behavior Out " + testNodesBehavior.getNodesValdity());
   }
         
   //@Test(timeout = 30000)
   public void wholebodyTrajectoryBehaviorTest() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
                  
      //HandTrajectoryBehavior handBehavior = new HandTrajectoryBehavior("rigthHand", drcBehaviorTestHelper.getBehaviorCommunicationBridge(), drcBehaviorTestHelper.getYoTime());
      WholeBodyTrajectoryBehavior wholebodyBehavior = new WholeBodyTrajectoryBehavior("wholebody", drcBehaviorTestHelper.getBehaviorCommunicationBridge(), drcBehaviorTestHelper.getYoTime());
      WholeBodyTrajectoryMessage wholebodyMessage = new WholeBodyTrajectoryMessage();
      HandTrajectoryMessage handMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 3.0, new Point3D(0.7, -0.35, 1.2), new Quaternion(), ReferenceFrame.getWorldFrame());
      Quaternion chestOrientation = new Quaternion();
      chestOrientation.appendYawRotation(Math.PI*0.3);
      ChestTrajectoryMessage chestMessage = new ChestTrajectoryMessage(3.0, chestOrientation, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      //PelvisHeightTrajectoryMessage pelvisHeightMessage = new PelvisHeightTrajectoryMessage(3.0, 0.6);
      
      wholebodyMessage.setHandTrajectoryMessage(handMessage);
      wholebodyMessage.setChestTrajectoryMessage(chestMessage);
      wholebodyBehavior.setInput(wholebodyMessage);
            
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(wholebodyBehavior, 3.0);

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(15);
      
      PrintTools.info("behavior Out " );      
   }
   
   //@Test(timeout = 30000)
   public void threeDimensionalNodeBehaviorTest() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      
      WholeBodyTrajectoryBehavior wholebodyBehavior = new WholeBodyTrajectoryBehavior("wholebody", drcBehaviorTestHelper.getBehaviorCommunicationBridge(), drcBehaviorTestHelper.getYoTime());      
      WholeBodyTrajectoryMessage wholebodyMessage = new WholeBodyTrajectoryMessage();
      
      SolarPanelWholeBodyTrajectoryMessageFacotry motionFactory = new SolarPanelWholeBodyTrajectoryMessageFacotry(sdfFullRobotModel);
      motionFactory.setMessage(new SolarPanelCleaningPose(solarPanel, 0.4, 0.2, 0.0), 0.65, Math.PI*0.0, -Math.PI*0.1, 3.0);
      
      wholebodyMessage = motionFactory.getWholeBodyTrajectoryMessage();
      wholebodyBehavior.setInput(wholebodyMessage);
            
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(wholebodyBehavior, 3.0);

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(15);
      
      PrintTools.info("behavior Out " );      
   }
   
   //@Test(timeout = 30000)
   public void controlPointOptimizationStateMachineBehaviorTest() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
            
      SolarPanelCleaningPose readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.15, -Math.PI*0.2);
      SolarPanelPath cleaningPath = new SolarPanelPath(readyPose);
      
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.15, -Math.PI*0.3), 4.0);         
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.2, -0.15, -Math.PI*0.3), 1.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.2, -0.15, -Math.PI*0.2), 4.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.3, -0.15, -Math.PI*0.2), 1.0);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.3, -0.15, -Math.PI*0.3), 4.0);
      
      SolarPanelCleaningInfo.setCleaningPath(cleaningPath);
      TimeDomain1DNode rootNode = new TimeDomain1DNode(cleaningPath.getArrivalTime().get(0), 0);
            
      ControlPointOptimizationStateMachineBehavior controlPointOptimizationBehavior
      = new ControlPointOptimizationStateMachineBehavior(rootNode, drcBehaviorTestHelper.getBehaviorCommunicationBridge(), drcBehaviorTestHelper.getYoTime(), getRobotModel(), sdfFullRobotModel, drcBehaviorTestHelper.getReferenceFrames());
      
      drcBehaviorTestHelper.dispatchBehavior(controlPointOptimizationBehavior);

      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(15);
      
      PrintTools.info("behavior Out " );      
   }
   
   @Test(timeout = 300000)
   public void cleaningMotionStateMachineBehaviorTest() throws SimulationExceededMaximumTimeException, IOException
   {
      if(isKinematicsToolboxVisualizerEnabled)
         ThreadTools.sleep(13000);
      
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      drcBehaviorTestHelper.updateRobotModel();
      
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
            
      CleaningMotionStateMachineBehavior cleaningStateMachineBehavior
      = new CleaningMotionStateMachineBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(), drcBehaviorTestHelper.getYoTime(), getRobotModel(), sdfFullRobotModel, drcBehaviorTestHelper.getReferenceFrames());
      
      drcBehaviorTestHelper.dispatchBehavior(cleaningStateMachineBehavior);
      
      Assert.assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(40));
      
      PrintTools.info("behavior Out " );      
   }
  

   private void setupKinematicsToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, isKinematicsToolboxVisualizerEnabled);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT, PacketDestination.KINEMATICS_TOOLBOX_MODULE);      
   }
   
   
   private void setUpSolarPanel()
   {
      Pose3D poseSolarPanel = new Pose3D();
      Quaternion quaternionSolarPanel = new Quaternion();
      poseSolarPanel.setPosition(0.7, -0.05, 1.0);
      quaternionSolarPanel.appendRollRotation(0.0);
      quaternionSolarPanel.appendPitchRotation(-Math.PI*0.25);
      poseSolarPanel.setOrientation(quaternionSolarPanel);
      
      solarPanel = new SolarPanel(poseSolarPanel, 0.6, 0.6);      
   }
   
   
// ************************************* //
   class DrawPanel extends JPanel
   {
      int timeScale = 70;
      int pelvisYawScale = 300;
      RRTPlannerSolarPanelCleaning planner;
      
      DrawPanel(RRTPlannerSolarPanelCleaning plannerTimeDomain)
      {
         this.planner = plannerTimeDomain;
      }

      @Override
      public void paint(Graphics g)
      {
         super.paint(g);       
         
         for(int j =0;j<planner.getNumberOfPlanners();j++)
         {
            RRTTreeTimeDomain tree = planner.getPlanner(j).getTree();
            ArrayList<RRTNode> wholeNode = tree.getWholeNode();
            g.setColor(Color.BLACK);
            for(int i =1;i<wholeNode.size();i++)
            {
               RRTNode rrtNode1 = wholeNode.get(i);
               RRTNode rrtNode2 = rrtNode1.getParentNode();
               branch(g, rrtNode1.getNodeData(0), rrtNode1.getNodeData(1), rrtNode2.getNodeData(0), rrtNode2.getNodeData(1), 4);
            }
            
            
            g.setColor(Color.BLUE);
            ArrayList<RRTNode> nodePath = tree.getPathNode();
            for(int i =1;i<nodePath.size();i++)
            {
               RRTNode rrtNode1 = nodePath.get(i);
               RRTNode rrtNode2 = rrtNode1.getParentNode();
               branch(g, rrtNode1.getNodeData(0), rrtNode1.getNodeData(1), rrtNode2.getNodeData(0), rrtNode2.getNodeData(1), 4);
            }
            
             g.setColor(Color.CYAN);
             ArrayList<RRTNode> nodeShort = planner.getPlanner(j).getOptimalPath();
             for(int i =1;i<nodeShort.size();i++)
             {
                RRTNode rrtNode1 = nodeShort.get(i);
                RRTNode rrtNode2 = nodeShort.get(i-1);
                branch(g, rrtNode1.getNodeData(0), rrtNode1.getNodeData(1), rrtNode2.getNodeData(0), rrtNode2.getNodeData(1), 4);
             }
             
             g.setColor(Color.RED);
             ArrayList<RRTNode> nodeFail = tree.failNodes;
             PrintTools.info("whole "+ j +" "+wholeNode.size() + " path " + nodePath.size() + " nodeShort " + nodeShort.size() + " fail " + nodeFail.size());
             for(int i =0;i<nodeFail.size();i++)
             {
                RRTNode rrtNode1 = nodeFail.get(i);
                point(g, rrtNode1.getNodeData(0), rrtNode1.getNodeData(1), 4);
             }
         }

         

         
         g.setColor(Color.yellow);
         branch(g, RRTNode1DTimeDomain.cleaningPath.getArrivalTime().get(1), -Math.PI*0.4, RRTNode1DTimeDomain.cleaningPath.getArrivalTime().get(1), Math.PI*0.4, 4);
         branch(g, RRTNode1DTimeDomain.cleaningPath.getArrivalTime().get(2), -Math.PI*0.4, RRTNode1DTimeDomain.cleaningPath.getArrivalTime().get(2), Math.PI*0.4, 4);
         branch(g, RRTNode1DTimeDomain.cleaningPath.getArrivalTime().get(3), -Math.PI*0.4, RRTNode1DTimeDomain.cleaningPath.getArrivalTime().get(3), Math.PI*0.4, 4);
      }
      
      public int t2u(double time)
      {
         return (int) Math.round((time * timeScale) + 50);
      }

      public int y2v(double yaw)
      {
         return (int) Math.round(((-yaw)) * pelvisYawScale + 400);
      }
      
      public void point(Graphics g, double time, double yaw, int size)
      {
         int diameter = size;
         g.drawOval(t2u(time) - diameter / 2, y2v(yaw) - diameter / 2, diameter, diameter);
      }
      
      public void branch(Graphics g, double time1, double yaw1, double time2, double yaw2, int size)
      {
         point(g, time1, yaw1, size);
         point(g, time2, yaw2, size);
         
         g.drawLine(t2u(time1), y2v(yaw1), t2u(time2), y2v(yaw2));
      }
   }
   
   private ArrayList<Graphics3DObject> getPrintCleaningPath(SolarPanelPath cleaningPath)
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();
      
      for(int i=0;i<cleaningPath.getLinearPath().size();i++)
      {
         ret.addAll(getPrintLinearPath(cleaningPath.getLinearPath().get(i)));
      }
      
      return ret;
   }
   
   private ArrayList<Graphics3DObject> getPrintLinearPath(SolarPanelLinearPath linearPath)
   {
      ArrayList<Graphics3DObject> ret = new ArrayList<Graphics3DObject>();

      
      Graphics3DObject nodeOneSphere = new Graphics3DObject();
      Graphics3DObject nodeTwoSphere = new Graphics3DObject();
      
      Graphics3DObject lineCapsule = new Graphics3DObject();
      
      Point3D translationNodeOne = linearPath.getStartPose().getDesiredHandPosition();
      nodeOneSphere.translate(translationNodeOne);
      nodeOneSphere.addSphere(0.02, YoAppearance.DarkGray());
      
      Point3D translationNodeTwo = linearPath.getEndPose().getDesiredHandPosition();
      nodeTwoSphere.translate(translationNodeTwo);
      nodeTwoSphere.addSphere(0.02, YoAppearance.DarkGray());
      
      Point3D translationLine = new Point3D((translationNodeOne.getX()+translationNodeTwo.getX())/2, (translationNodeOne.getY()+translationNodeTwo.getY())/2, (translationNodeOne.getZ()+translationNodeTwo.getZ())/2);
      AxisAngle rotationLine = new AxisAngle(-(translationNodeOne.getY()-translationNodeTwo.getY()), (translationNodeOne.getX()-translationNodeTwo.getX()), 0, Math.PI/2);
      lineCapsule.translate(translationLine);      
      lineCapsule.rotate(rotationLine);
      lineCapsule.addCapsule(0.02, translationNodeOne.distance(translationNodeTwo), YoAppearance.Gray());
            
      ret.add(nodeOneSphere);
      ret.add(nodeTwoSphere);

      return ret;
   }
   
   // ************************************* //
}
