package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.avatar.testTools.ScriptedHandstepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandstepPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationConstructionSetTools.util.environments.BigStepUpWithHandPlatformEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCBigStepUpWithHandPlatformTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   
   private DRCSimulationTestHelper drcSimulationTestHelper;

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
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@ContinuousIntegrationTest(estimatedDuration = 26.8)
	@Test(timeout = 130000)
   public void testBigStepUpWithHandPlatform() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double stepHeight = 0.2;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      BigStepUpWithHandPlatformEnvironment environment = new BigStepUpWithHandPlatformEnvironment(stepHeight);
      
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, "testBigStepUpWithHandPlatform", selectedLocation, simulationTestingParameters, getRobotModel());

      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForBigStepUpWithHandPlatform();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      ScriptedHandstepGenerator scriptedHandstepGenerator = drcSimulationTestHelper.createScriptedHandstepGenerator();

      double leftHandstepY = 0.4;
      double rightHandstepY = -0.4;
      ArrayList<Handstep> handsteps = createHandstepForTopsOfHandholds(leftHandstepY, rightHandstepY, scriptedHandstepGenerator);

      for (Handstep handstep : handsteps)
      {
    	  Point3D location = new Point3D();
    	  Quaternion orientation = new Quaternion();
    	  Vector3D surfaceNormal = new Vector3D();
    	  handstep.getPose(location, orientation);
          handstep.getSurfaceNormal(surfaceNormal);
    	  
         HandstepPacket handstepPacket = new HandstepPacket(handstep.getRobotSide(),
        		 											location, orientation, surfaceNormal,
        		 											handstep.getSwingTrajectoryTime());
         drcSimulationTestHelper.send(handstepPacket);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);
      }

      FootstepDataListMessage footstepDataList = createFootstepsForStepOntoPlatform(RobotSide.LEFT, 0.5, 0.1, stepHeight, scriptedFootstepGenerator);
      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      //TODO: This only works if you step a little too high. Otherwise it catches its foot on the step
      double extraHeightToAvoidStubbing = 0.05;
      footstepDataList = createFootstepsForStepOntoPlatform(RobotSide.RIGHT, 0.5, -0.1, stepHeight + extraHeightToAvoidStubbing, scriptedFootstepGenerator);
      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      
      // Make sure that the hands are on the target...
      FloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      DoubleYoVariable leftArmZForce = (DoubleYoVariable) robot.getVariable("gc_l_arm_wrx_0_fZ");
      DoubleYoVariable rightArmZForce = (DoubleYoVariable) robot.getVariable("gc_r_arm_wrx_0_fZ");
      assertTrue(leftArmZForce.getDoubleValue() > 50.0);
      assertTrue(rightArmZForce.getDoubleValue() > 50.0);
      
      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForBigStepUpWithHandPlatform()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private ArrayList<Handstep> createHandstepForTopsOfHandholds(double leftHandstepY, double rightHandstepY, ScriptedHandstepGenerator scriptedHandstepGenerator)
   {
      ArrayList<Handstep> ret = new ArrayList<Handstep>();

      double height = 0.9;
      
      RobotSide robotSide = RobotSide.LEFT;
      Tuple3DBasics position = new Point3D(0.5, leftHandstepY, height);
      Vector3D surfaceNormal = new Vector3D(0.0, 0.0, 1.0);
      double rotationAngleAboutNormal = -0.3;
      double swingTrajectoryTime = 1.0;

      Handstep handstep = scriptedHandstepGenerator.createHandstep(robotSide, position, surfaceNormal, rotationAngleAboutNormal, swingTrajectoryTime);
      ret.add(handstep);
      
      robotSide = RobotSide.RIGHT;
      position = new Point3D(0.5, rightHandstepY, height);
      surfaceNormal = new Vector3D(0.0, 0.0, 1.0);
      rotationAngleAboutNormal = -rotationAngleAboutNormal;

      handstep = scriptedHandstepGenerator.createHandstep(robotSide, position, surfaceNormal, rotationAngleAboutNormal, swingTrajectoryTime);
      ret.add(handstep);

      return ret;
   }
   
   private FootstepDataListMessage createFootstepsForStepOntoPlatform(RobotSide robotSide, double stepX, double stepY, double stepZ, ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {stepX, stepY, stepZ},
            {0.0, 0.0, 0.0, 1.0}
         }
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(robotSide, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

}

