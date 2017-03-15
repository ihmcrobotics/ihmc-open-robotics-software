package us.ihmc.avatar.rrtManipulation;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicsBehavior;
import us.ihmc.manipulation.planning.manipulation.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.manipulation.solarpanelmotion.SolarPanelCleaningBehavior;
import us.ihmc.manipulation.planning.manipulation.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class AvatarSolarPanelCleaningMotionTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private boolean isKinematicsToolboxVisualizerEnabled = false;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private KinematicsToolboxModule kinematicsToolboxModule;
   private PacketCommunicator toolboxCommunicator;
      
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
         // TODO Auto-generated method stub
         return EnvSet;
      }

      @Override
      public List<? extends Robot> getEnvironmentRobots()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public void createAndSetContactControllerToARobot()
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
      {
         // TODO Auto-generated method stub

      }

      @Override
      public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
      {
         // TODO Auto-generated method stub
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
      //if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer()) // set
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

      CommonAvatarEnvironmentInterface envrionment = new SolarPanelCleaningEnvironment();
      //CommonAvatarEnvironmentInterface envrionment = new FlatGroundEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(envrionment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());

      setupKinematicsToolboxModule();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 30.0) 
   //@Test(timeout = 160000)
   public void testSolvingForBothHandPoses() throws SimulationExceededMaximumTimeException, IOException
   {
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      PrintTools.info("a"+drcBehaviorTestHelper.getYoTime());
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      setupCamera(scs);
      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(getRobotModel(), drcBehaviorTestHelper.getYoTime(),
                                                                                     drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                                     getRobotModel().createFullRobotModel());

      ReferenceFrame handControlFrameR = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.RIGHT);
      ReferenceFrame handControlFrameL = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.LEFT);

      FramePose desiredHandPoseR = new FramePose(handControlFrameR);
      desiredHandPoseR.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHandPoseR.translate(0.20, 0.0, 0.0);      
      ik.setTrajectoryTime(3.0);
      ik.setDesiredHandPose(RobotSide.RIGHT, desiredHandPoseR);
      FramePose desiredHandPoseL = new FramePose(handControlFrameL);
      desiredHandPoseL.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHandPoseL.translate(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(3.0);
      ik.setDesiredHandPose(RobotSide.LEFT, desiredHandPoseL);
      drcBehaviorTestHelper.dispatchBehavior(ik);
      
      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(5.0);
      assertTrue(success);

      Quaternion controllerDesiredHandOrientationR = EndToEndHandTrajectoryMessageTest.findControllerDesiredOrientation(RobotSide.RIGHT, scs);
      Quaternion desiredHandOrientationR = new Quaternion();
      desiredHandPoseR.getOrientation(desiredHandOrientationR);
      Quaternion controllerDesiredHandOrientationL = EndToEndHandTrajectoryMessageTest.findControllerDesiredOrientation(RobotSide.LEFT, scs);
      Quaternion desiredHandOrientationL = new Quaternion();
      desiredHandPoseL.getOrientation(desiredHandOrientationL);

      Point3D controllerDesiredHandPositionR = EndToEndHandTrajectoryMessageTest.findControllerDesiredPosition(RobotSide.RIGHT, scs);
      Point3D controllerDesiredHandPositionL = EndToEndHandTrajectoryMessageTest.findControllerDesiredPosition(RobotSide.LEFT, scs);
      Point3D rightPosition = new Point3D();
      desiredHandPoseR.getPosition(rightPosition);
      Point3D leftPosition = new Point3D();
      desiredHandPoseL.getPosition(leftPosition);


   }
   
   @Test(timeout = 160000)
   public void testCombinedMotion() throws SimulationExceededMaximumTimeException, IOException
   {
      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();      
      setupCamera(scs);
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      
      // ********** Planning *** //
      
      

      
      // ********** Behavior *** //
      
      
      
      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(getRobotModel(), drcBehaviorTestHelper.getYoTime(),
                                                                                     drcBehaviorTestHelper.getBehaviorCommunicationBridge(),
                                                                                     getRobotModel().createFullRobotModel());
      
      SolarPanelCleaningBehavior solarPanelCleaningBehavior = new SolarPanelCleaningBehavior(solarPanel, ik);
      
      double motionTime = 2.0;
      SolarPanelCleaningPose startPose = new SolarPanelCleaningPose(new Point3D(0.5,  -0.25,  1.2), 0.0, -Math.PI/0.25, 0.0);
      solarPanelCleaningBehavior.setIKwithGoalPose(startPose, motionTime);
      PrintTools.info("goto Ready");
      drcBehaviorTestHelper.dispatchBehavior(ik);
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(motionTime+0.1);
      
      
      solarPanelCleaningBehavior.generateMotion();      
      ArrayList<SolarPanelCleaningPose> poses = solarPanelCleaningBehavior.getTrajectory().getPoses();
      
      for (int i =0;i<poses.size();i++)
      {
         ik.clear();
         
         PrintTools.info("ad");
         solarPanelCleaningBehavior.setIKwithGoalPose(poses.get(i), 0.5);
         PrintTools.info("ad");
         drcBehaviorTestHelper.dispatchBehavior(ik);
         PrintTools.info("ad");
         drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions( 0.5+0.1);
         PrintTools.info("ad");
      }

      
      
      
      
      drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(5.0);
   }
   
   private void setUpSolarPanel()
   {
      Pose poseSolarPanel = new Pose();
      Quaternion quaternionSolarPanel = new Quaternion();
      poseSolarPanel.setPosition(0.7, 0.05, 1.0);
      quaternionSolarPanel.appendRollRotation(0.0);
      quaternionSolarPanel.appendPitchRotation(-Math.PI*0.25);
      poseSolarPanel.setOrientation(quaternionSolarPanel);
      
      solarPanel = new SolarPanel(poseSolarPanel, 0.6, 0.6);
   }

   public void setupCamera(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(1.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(-2.5, 5.0, 3.0);

      drcBehaviorTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
   
  
   private void setupKinematicsToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, isKinematicsToolboxVisualizerEnabled);
      toolboxCommunicator = drcBehaviorTestHelper.createAndStartPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT, PacketDestination.KINEMATICS_TOOLBOX_MODULE);
   }
      

}