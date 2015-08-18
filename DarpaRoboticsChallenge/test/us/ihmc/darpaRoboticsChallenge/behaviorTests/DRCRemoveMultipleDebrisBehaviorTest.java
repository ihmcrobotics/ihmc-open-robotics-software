package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.communication.packets.behaviors.DebrisData;
import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDebrisEnvironment;
import us.ihmc.darpaRoboticsChallenge.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.RemoveMultipleDebrisBehavior;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.ContactableSelectableBoxRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCRemoveMultipleDebrisBehaviorTest implements MultiRobotTestInterface
{
   private final boolean DEBUG = false;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final double ROBOT_POSITION_TOLERANCE = 0.05;
   private final double POSITION_ERROR_MARGIN = 0.025;
   private final double ANGLE_ERROR_MARGIN = 0.05;

   private DRCBehaviorTestHelper drcBehaviorTestHelper;

   private final DRCDebrisEnvironment testEnvironment = new DRCDebrisEnvironment();

   private DoubleYoVariable yoTime;
   private SDFRobot robot;
   private SDFFullHumanoidRobotModel fullRobotModel;
   private DRCRobotModel drcRobotModel;

   private ArrayList<ContactableSelectableBoxRobot> debrisRobots;
   private ArrayList<DebrisData> debrisDataList = new ArrayList<>();

   private void showMemoryUsageBeforeTest()
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
         drcBehaviorTestHelper.destroySimulation();
         drcBehaviorTestHelper = null;
      }

      GlobalTimer.clearTimers();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Before
   public void setUp()
   {
      showMemoryUsageBeforeTest();

      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            Vector3d additionalOffset = new Vector3d(0.0, 0.0, 0.0);
            double yaw = 0.0;
            OffsetAndYawRobotInitialSetup offsetAndYawRobotInitialSetup = new OffsetAndYawRobotInitialSetup(additionalOffset, yaw);
            return offsetAndYawRobotInitialSetup;
         }
      };

      testEnvironment.addVerticalDebrisLeaningAgainstAWall(1.9, -0.35, Math.toRadians(-20.0), Math.toRadians(18.0));
      testEnvironment.addStandingDebris(1.0, 0.2, Math.toRadians(20.0));
      testEnvironment.addHorizontalDebrisLeaningOnTwoBoxes(new Point3d(1.3, 0.0, 0.6), Math.toRadians(0.0), Math.toRadians(90.0));
      
      testEnvironment.createDebrisContactController();
      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null, startingLocation,
            simulationTestingParameters, getRobotModel());

      yoTime = drcBehaviorTestHelper.getRobot().getYoTime();

      robot = drcBehaviorTestHelper.getRobot();
      fullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      drcRobotModel = getRobotModel();
      debrisRobots = new ArrayList<>();
   }

   @EstimatedDuration(duration = 90.0)
   @Test(timeout = 3000000)
   public void testRemovingthreeDebris() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();
      
      drcBehaviorTestHelper.updateRobotModel();

      assertTrue(drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      drcBehaviorTestHelper.updateRobotModel();

      //TODO: pass in the wrist sensor when it will be used in the behaviors
      final RemoveMultipleDebrisBehavior removeMultipleDebrisBehavior = new RemoveMultipleDebrisBehavior(
            drcBehaviorTestHelper.getBehaviorCommunicationBridge(), fullRobotModel, drcBehaviorTestHelper.getReferenceFrames(), null, yoTime, drcRobotModel);

      generateDebrisData();
      removeMultipleDebrisBehavior.initialize();
      removeMultipleDebrisBehavior.setInputs(debrisDataList);
      assertTrue(removeMultipleDebrisBehavior.hasInputBeenSet());
      drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(removeMultipleDebrisBehavior, 120.0);
      assertTrue(removeMultipleDebrisBehavior.isDone());
      
      assertDebrisHaveBeenRemoved();
      
      
      BambooTools.reportTestFinishedMessage();
   }

   private void assertDebrisHaveBeenRemoved()
   {
      // TODO Auto-generated method stub
      
   }

   private void generateDebrisData()
   {
      debrisRobots.addAll(testEnvironment.getEnvironmentRobots());
      for (int i = 0; i < debrisRobots.size(); i++)
      {
         //this offset is very important because debrisTransform sent from the UI have the origin at the bottom of the debris, whereas here the robots have their origin at the center of the debris
         double zOffsetToHaveOriginAtDebrisBottom = testEnvironment.getDebrisLength() / 2.0;
         
         RigidBodyTransform debrisTransform = new RigidBodyTransform();
         debrisRobots.get(i).getBodyTransformToWorld(debrisTransform);

         PoseReferenceFrame debrisReferenceFrame = new PoseReferenceFrame("debrisReferenceFrame", worldFrame);
         debrisReferenceFrame.setPoseAndUpdate(debrisTransform);

         FramePose debrisPose = new FramePose(debrisReferenceFrame);
         debrisPose.setZ(-zOffsetToHaveOriginAtDebrisBottom);
         debrisPose.changeFrame(worldFrame);

         FrameVector graspVector = new FrameVector(debrisReferenceFrame);
         graspVector.set(-1.0, 0.0, 0.0);
         graspVector.changeFrame(worldFrame);

         FramePoint graspVectorPosition = new FramePoint(debrisReferenceFrame);
         graspVectorPosition.setZ(0.6 - zOffsetToHaveOriginAtDebrisBottom);
         graspVectorPosition.changeFrame(worldFrame);

         debrisPose.getRigidBodyTransform(debrisTransform);
         
         debrisDataList.add(new DebrisData(debrisTransform, graspVector.getVector(), graspVectorPosition.getPointCopy()));
      }
   }
}
