package us.ihmc.avatar.behaviorTests.SolarPanel;

import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.awt.Graphics;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.JPanel;

import org.junit.After;
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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.RRTNode3DTimeDomain;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.RRTPlannerSolarPanelCleaning;
import us.ihmc.humanoidBehaviors.behaviors.solarPanel.RRTTreeTimeDomain;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.rrt.WheneverWholeBodyValidityTester;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelLinearPath;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.SolarPanelEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class SpecifiedWholeBodyMotionPlanningTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private boolean isKinematicsToolboxVisualizerEnabled = true;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;
   
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   SolarPanel solarPanel;
      
   private void setUpSolarPanel()
   {
      Pose poseSolarPanel = new Pose();
      Quaternion quaternionSolarPanel = new Quaternion();
      poseSolarPanel.setPosition(0.65, -0.2, 1.03);
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

      CommonAvatarEnvironmentInterface envrionment = new SolarPanelEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(envrionment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());

      setupKinematicsToolboxModule();
   }
   
   @Test
   public void isValidTest() throws SimulationExceededMaximumTimeException, IOException
   {
      if(false)
         ThreadTools.sleep(13000);
      
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      drcBehaviorTestHelper.updateRobotModel();
            
      drcBehaviorTestHelper.getControllerFullRobotModel().updateFrames();
      
      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
      sdfFullRobotModel.updateFrames();
            
      setUpSolarPanel();
            
      RRTNode3DTimeDomain.nodeValidityTester = new WheneverWholeBodyValidityTester(sdfFullRobotModel);
      
      kinematicsToolboxModule.getToolboxController().update();
      
      
      
      
//      PrintTools.info(""+sdfFullRobotModel.getHand(RobotSide.RIGHT).getBodyFixedFrame().getTransformToWorldFrame());
      
      
      
      ForceSensorDefinition[] forceSensorDefinitions;
      IMUDefinition[] imuDefinitions;
      OneDoFJoint[] joints = sdfFullRobotModel.getOneDoFJoints();
      imuDefinitions = sdfFullRobotModel.getIMUDefinitions();
      forceSensorDefinitions = sdfFullRobotModel.getForceSensorDefinitions();      
      
      RobotConfigurationData currentRobotConfigurationData = new RobotConfigurationData(joints, forceSensorDefinitions, null, imuDefinitions);
      
      currentRobotConfigurationData.setJointState(joints);
      
      RRTNode3DTimeDomain.nodeValidityTester.updateRobotConfigurationData(currentRobotConfigurationData);
      
      
      // ********** Planning *** //      
      SolarPanelCleaningPose readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.05, -Math.PI*0.2);    
      SolarPanelPath cleaningPath = new SolarPanelPath(readyPose);
      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.05, -Math.PI*0.3), 4.0);  
            
      RRTNode3DTimeDomain.cleaningPath = cleaningPath;
      
      RRTNode3DTimeDomain node1 = new RRTNode3DTimeDomain(1.0, 0.8, 0/180*Math.PI, 0/180*Math.PI);
      RRTNode3DTimeDomain node2 = new RRTNode3DTimeDomain(1.0, 0.9, 15/180*Math.PI, 0/180*Math.PI);
      RRTNode3DTimeDomain node3 = new RRTNode3DTimeDomain(1.0, 0.7, -15/180*Math.PI, 10/180*Math.PI);
      
      PrintTools.info(""+node1.isValidNode());
//      ThreadTools.sleep(1000);
//      PrintTools.info(""+node2.isValidNode());
//      ThreadTools.sleep(1000);
//      PrintTools.info(""+node3.isValidNode());
//      ThreadTools.sleep(1000);
      
      PrintTools.info("END");     
   } 
    
   //@Test
   public void plannerTest() throws SimulationExceededMaximumTimeException, IOException
   {
//      if(isKinematicsToolboxVisualizerEnabled)
//         ThreadTools.sleep(13000);
//      
//      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
//      assertTrue(success);
//
//      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
//
//      drcBehaviorTestHelper.updateRobotModel();
//      
//      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
//      
//      drcBehaviorTestHelper.getControllerFullRobotModel().updateFrames();
//      
//      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
//      sdfFullRobotModel.updateFrames();
//            
//      setUpSolarPanel();
//            
//      RRTNode3DTimeDomain.nodeValidityTester = new WheneverWholeBodyPoseTester(getRobotModel(), 
//                                                                                drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
//                                                                                sdfFullRobotModel, drcBehaviorTestHelper.getReferenceFrames());
//            
//      drcBehaviorTestHelper.dispatchBehavior(RRTNode3DTimeDomain.nodeValidityTester);
//      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
//      
//      // ********** Planning *** //      
//      SolarPanelCleaningPose readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.05, -Math.PI*0.2);    
//      SolarPanelPath cleaningPath = new SolarPanelPath(readyPose);
//      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.05, -Math.PI*0.3), 4.0);  
//            
//      RRTNode3DTimeDomain.cleaningPath = cleaningPath;
//      RRTNode3DTimeDomain.nodeValidityTester.setSolarPanel(solarPanel);
//      
//      RRTNode3DTimeDomain node0 = new RRTNode3DTimeDomain();
//      node0.isValidNode();
//      
//      RRTNode3DTimeDomain node1 = new RRTNode3DTimeDomain(1.0, 0.8, 0/180*Math.PI, 0/180*Math.PI);
//      RRTNode3DTimeDomain node2 = new RRTNode3DTimeDomain(1.0, 0.9, 15/180*Math.PI, 0/180*Math.PI);
//      RRTNode3DTimeDomain node3 = new RRTNode3DTimeDomain(1.0, 0.7, -15/180*Math.PI, 10/180*Math.PI);
//      
//      PrintTools.info(""+node1.isValidNode());
//      ThreadTools.sleep(1000);
//      PrintTools.info(""+node2.isValidNode());
//      ThreadTools.sleep(1000);
//      PrintTools.info(""+node3.isValidNode());
//      ThreadTools.sleep(1000);
//      
//      
//      scs.addStaticLinkGraphics(getPrintCleaningPath(RRTNode3DTimeDomain.cleaningPath));
//
//      if (visualize)
//      {
//         // ************************************* //
//         // show
//         // ************************************* //
//         JFrame frame;
//         DrawPanel drawPanel;
//         Dimension dim;
//
//
//         frame = new JFrame("RRTTest");
//         drawPanel = new DrawPanel(solarPanelPlanner.rrtPlanner);
//         dim = new Dimension(1600, 800);
//         frame.setPreferredSize(dim);
//         frame.setLocation(200, 100);
//
//         frame.add(drawPanel);
//         frame.pack();
//         frame.setVisible(true);
//      }
      
      // ************************************* //
      // show
      // ************************************* //
      PrintTools.info("END");     
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
//         branch(g, RRTNode3DTimeDomain.cleaningPath.getArrivalTime().get(1), -Math.PI*0.4, RRTNode3DTimeDomain.cleaningPath.getArrivalTime().get(1), Math.PI*0.4, 4);
//         branch(g, RRTNode3DTimeDomain.cleaningPath.getArrivalTime().get(2), -Math.PI*0.4, RRTNode3DTimeDomain.cleaningPath.getArrivalTime().get(2), Math.PI*0.4, 4);
//         branch(g, RRTNode3DTimeDomain.cleaningPath.getArrivalTime().get(3), -Math.PI*0.4, RRTNode1DTimeDomain.cleaningPath.getArrivalTime().get(3), Math.PI*0.4, 4);
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
}
