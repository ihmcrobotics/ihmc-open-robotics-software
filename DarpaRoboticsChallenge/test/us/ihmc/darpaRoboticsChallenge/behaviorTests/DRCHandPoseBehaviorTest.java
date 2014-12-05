package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.util.Arrays;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public abstract class DRCHandPoseBehaviorTest implements MultiRobotTestInterface
{
   private static final boolean KEEP_SCS_UP = true;

   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = KEEP_SCS_UP || createMovie;

   private DRCDemo01NavigationEnvironment testEnvironment = new DRCDemo01NavigationEnvironment();
   private ObjectCommunicator networkObjectCommunicator = new LocalObjectCommunicator();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private RobotSide robotSideToTest = RobotSide.LEFT;

   private DoubleYoVariable yoTime;

   private RobotDataReceiver robotDataReceiver;
   private ForceSensorDataHolder forceSensorDataHolder;

   private BehaviorCommunicationBridge communicationBridge;

   private FullRobotModel fullRobotModel;
   
   
   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, networkObjectCommunicator, getSimpleRobotName(), null,
              DRCObstacleCourseStartingLocation.DEFAULT, checkNothingChanged, showGUI, createMovie, true, getRobotModel());

      
      Robot robotToTest = drcSimulationTestHelper.getRobot();
      yoTime = robotToTest.getYoTime();

      fullRobotModel = getRobotModel().createFullRobotModel();

      forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));

      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder, true);
      networkObjectCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      ObjectCommunicator junkyObjectCommunicator = new LocalObjectCommunicator();

      communicationBridge = new BehaviorCommunicationBridge(junkyObjectCommunicator, networkObjectCommunicator, robotToTest.getRobotsYoVariableRegistry());


      // setupCameraForHandstepsOnWalls();
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (KEEP_SCS_UP)
      {
         ThreadTools.sleepForever();
      }

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @Test
   public void testSimpleHandPoseMove() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      HandPoseBehavior handPoseBehavior = new HandPoseBehavior(communicationBridge, yoTime);


      fullRobotModel.updateFrames();
      //ReferenceFrame handFrame = fullRobotModel.getHandControlFrame(robotSideToTest);
      
      FramePose handPose = new FramePose();  
      handPose.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));
      
      handPose.changeFrame(ReferenceFrame.getWorldFrame());
      handPose.setZ(handPose.getZ() + 0.2);

//      Point3d position = new Point3d(0.5, 0.35, 1.0);
//      Quat4d orientation = new Quat4d(-1.0, 0.0, 0.0, 0.5 * Math.PI);
//      
//
//      Vector3d translation = new Vector3d(position.getX(), position.getY(), position.getZ());

      RigidBodyTransform pose = new RigidBodyTransform(); //handFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      handPose.getPose(pose);
      
      double swingTrajectoryTime = 2.0;
      
      handPoseBehavior.setInput(Frame.WORLD, pose, robotSideToTest, swingTrajectoryTime);

      double simulationRunTime = swingTrajectoryTime + 1.0;
      
   
      Thread behaviorThread = new Thread()
      {
         public void run()
         {
            {
               double startTime = Double.NaN;
               boolean simStillRunning = true;
               boolean initalized = false;
               
               while(simStillRunning)
               {
                  if (!initalized)
                  {
                     startTime = yoTime.getDoubleValue();
                     initalized = true;
                  }
                  
                  double timeSpentSimulating = yoTime.getDoubleValue() - startTime;
                  simStillRunning = timeSpentSimulating < simulationRunTime;
                  
                  handPoseBehavior.doControl();        
               }             
            }
         }
      };
      
      behaviorThread.start();
      
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);
   
      
      
      System.out.println("testSimpleHandPoseMove(): behaviorDone=" + handPoseBehavior.isDone());
      
      assertTrue(success);
      assertTrue(handPoseBehavior.isDone());



      BambooTools.reportTestFinishedMessage();
   }


}
