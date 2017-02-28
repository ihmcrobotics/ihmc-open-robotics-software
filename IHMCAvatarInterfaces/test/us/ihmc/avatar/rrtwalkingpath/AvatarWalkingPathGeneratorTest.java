package us.ihmc.avatar.rrtwalkingpath;

import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.EnvironmentInfo;
import us.ihmc.simulationconstructionset.util.environments.WalkingPathPlanningEnvironments;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class AvatarWalkingPathGeneratorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   
   private CommunicationBridge communicationBridge;
   private DoubleYoVariable yoTime;

   private FullHumanoidRobotModel fullRobotModel;
   private HumanoidRobotDataReceiver robotDataReceiver;
   private HumanoidFloatingRootJointRobot robot;
   private static final boolean DEBUG = false;
   
   private EnvironmentInfo testEnvironmentInfo;
   
   SimpleCollisionDetector collisionDetector = new SimpleCollisionDetector();
   CollisionDetectionResult collisionDetectionResult = new CollisionDetectionResult();

   SimpleCollisionShapeFactory shapeFactory;   
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      fullRobotModel = getRobotModel().createFullRobotModel();
   }
   
   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer()) // set
      //if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
         communicationBridge = null;
         yoTime = null;
         fullRobotModel = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   private void setUpSimulationTestHelper(CommonAvatarEnvironmentInterface environment, DRCObstacleCourseStartingLocation startingLocation)
   {
      
      if(drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
      }

      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, getSimpleRobotName(), startingLocation, simulationTestingParameters, getRobotModel());
      
   }
   
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testOne() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      
      shapeFactory = (SimpleCollisionShapeFactory) collisionDetector.getShapeFactory();
            

      
      testEnvironmentInfo = new EnvironmentInfo();

      CommonAvatarEnvironmentInterface commonAvatarEnvironmentInterface = new WalkingPathPlanningEnvironments(testEnvironmentInfo);
      setUpSimulationTestHelper(commonAvatarEnvironmentInterface, selectedLocation);
      


      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      
      setupCamera(simulationConstructionSet);
      
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      
      
      
      
   // ******************************** //
      
      double swingTime = 2.0;
      double transferTime = 0.8;
      double stepLength = 0.3;
      double stepWidth = 0.14;
      int steps = 3;

      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime, transferTime);
      FootstepDataMessage footstepData;
      RobotSide robotSide;
      double footstepY;
      double footstepX;
      Point3d location;      
      Quat4d orientation;
      
      robotSide = RobotSide.LEFT;
      footstepX = 0.2;
      footstepY = 0.14;
      location = new Point3d(footstepX, footstepY, 0.0);
      orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      footstepData = new FootstepDataMessage(robotSide, location, orientation);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);
  
      
      robotSide = RobotSide.RIGHT;
      footstepX = 0.4;
      footstepY = -0.14;
      location = new Point3d(footstepX, footstepY, 0.0);
      orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      footstepData = new FootstepDataMessage(robotSide, location, orientation);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);
      
      
      robotSide = RobotSide.LEFT;
      footstepX = 0.4;
      footstepY = 0.14;
      location = new Point3d(footstepX, footstepY, 0.0);
      orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      footstepData = new FootstepDataMessage(robotSide, location, orientation);
      footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
      footsteps.add(footstepData);

      drcSimulationTestHelper.send(footsteps);
      double simulationTime = (swingTime + transferTime) * steps + 1.0;
      
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
   // ******************************** //
      

      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      
   }
   
   private void setupCamera(SimulationConstructionSet scs)
   {
      Point3d cameraFix = new Point3d(2.0, 0.0, 1.1);
      Point3d cameraPosition = new Point3d(-5.0, 10.0, 15.0);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
   // ******************************** //

   private void setUpEnvironment()
   {
      CollisionShape obstacle1 = shapeFactory.addShape(shapeFactory.createBox(0.5, 0.5, 0.5));
      CollisionShape obstacle2 = shapeFactory.addShape(shapeFactory.createBox(0.5, 0.5, 0.5));
      CollisionShape obstacle3 = shapeFactory.addShape(shapeFactory.createBox(0.5, 0.5, 0.5));
      CollisionShape obstacle4 = shapeFactory.addShape(shapeFactory.createBox(0.5, 0.5, 0.5));
      CollisionShape obstacle5 = shapeFactory.addShape(shapeFactory.createBox(0.5, 0.5, 0.5));
      
      obstacle1.setCollisionMask(0b000000);
      obstacle1.setCollisionGroup(0b000000);
      obstacle2.setCollisionMask(0b000000);
      obstacle2.setCollisionGroup(0b000000);
      obstacle3.setCollisionMask(0b000000);
      obstacle3.setCollisionGroup(0b000000);
      obstacle4.setCollisionMask(0b000000);
      obstacle4.setCollisionGroup(0b000000);
      obstacle5.setCollisionMask(0b000000);
      obstacle5.setCollisionGroup(0b000000);
      
      
   }
   
}
