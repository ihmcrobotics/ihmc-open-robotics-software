package us.ihmc.avatar;

import controller_msgs.PelvisHeightTrajectoryMessage;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeightMapBasedFootstepAdjustment;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.ground.BumpyGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public abstract class DRCBumpyAndShallowRampsWalkingTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation testingSimulation;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (testingSimulation != null)
      {
         testingSimulation.destroy();
         testingSimulation = null;
      }
      robotModel = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private DRCRobotModel robotModel;

   @BeforeEach
   public void getRobotModelBeforeTests()
   {
      robotModel = getRobotModel();
   }

   @Test
   public void testDRCOverShallowRamp() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double standingTimeDuration = 1.0;
      double maximumWalkTime = 30.0;
      double desiredVelocityValue = 0.75;

      boolean useVelocityAndHeadingScript = false;
      boolean cheatWithGroundHeightAtForFootstep = true;

      ImmutablePair<CommonAvatarEnvironmentInterface, Double> combinedTerrainObjectAndRampEndX = createRamp();
      CommonAvatarEnvironmentInterface environment = combinedTerrainObjectAndRampEndX.getLeft();

      double rampEndX = combinedTerrainObjectAndRampEndX.getRight();

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, null);

      testingSimulation = simulationTestHelperFactory.createAvatarTestingSimulation();
      if (cheatWithGroundHeightAtForFootstep)
      {
         testingSimulation.getAvatarSimulation()
                          .getStepGeneratorThread()
                          .getSteppingManager()
                          .setFootstepAdjustment(new HeightMapBasedFootstepAdjustment(environment.getTerrainObject3D().getHeightMapIfAvailable()));
      }
      testingSimulation.start();

      SimulationConstructionSet2 scs = testingSimulation.getSimulationConstructionSet();

      testingSimulation.simulateNow(0.5);
      FullHumanoidRobotModel controllerFullRobotModel = testingSimulation.getAvatarSimulation().getControllerFullRobotModel();
      controllerFullRobotModel.updateFrames();
      FramePoint3D pelvisPosition = new FramePoint3D(controllerFullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      PelvisHeightTrajectoryMessage pelvisHeightMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, pelvisPosition.getZ() - 0.05);

      testingSimulation.getAvatarSimulation().getHighLevelHumanoidControllerFactory().getCommandInputManager().submitMessage(pelvisHeightMessage);

      String suffix = "StepGeneratorCommandInputManager";
      YoBoolean walk = (YoBoolean) scs.findVariable("walk_" + suffix);
      YoDouble q_x = (YoDouble) scs.findVariable("q_" + controllerFullRobotModel.getRootBody().getName() + "_x");
      YoDouble desiredSpeed = (YoDouble) scs.findVariable("desiredVelocity_" + suffix + "X");

      YoDouble comError = (YoDouble) scs.findVariable("positionError_comHeight");

      walk.set(false);
      testingSimulation.simulateNow(standingTimeDuration);
      walk.set(true);

      desiredSpeed.set(desiredVelocityValue);

      double timeIncrement = 1.0;
      boolean done = false;
      while (!done)
      {
         assertTrue(testingSimulation.simulateNow(timeIncrement));

         if (Math.abs(comError.getDoubleValue()) > 0.11)
         {
            fail("comError = " + Math.abs(comError.getDoubleValue()));
         }

         if (scs.getTime().getDoubleValue() > standingTimeDuration + maximumWalkTime)
            done = true;
         if (q_x.getDoubleValue() > rampEndX)
            done = true;
      }

      createVideo(scs);
   }

   // This has never worked. Would be nice if we can get it to work.")
   @Disabled
   @Test
   public void testDRCOverRandomBlocks() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double standingTimeDuration = 1.0;
      double maximumWalkTime = 10.0;
      double desiredVelocityValue = 0.5;

      boolean useVelocityAndHeadingScript = false;
      boolean cheatWithGroundHeightAtForFootstep = true;

      WalkingControllerParameters drcControlParameters = robotModel.getWalkingControllerParameters();

      //      drcControlParameters.setNominalHeightAboveAnkle(drcControlParameters.nominalHeightAboveAnkle() - 0.03);    // Need to do this or the leg goes straight and the robot falls.

      ImmutablePair<CommonAvatarEnvironmentInterface, Double> combinedTerrainObjectAndRampEndX = createRandomBlocks();
      CommonAvatarEnvironmentInterface environment = combinedTerrainObjectAndRampEndX.getLeft();

      double rampEndX = combinedTerrainObjectAndRampEndX.getRight();
      RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.01, 0);

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, null);

      testingSimulation = simulationTestHelperFactory.createAvatarTestingSimulation();
      if (cheatWithGroundHeightAtForFootstep)
      {
         testingSimulation.getAvatarSimulation()
                          .getStepGeneratorThread()
                          .getSteppingManager()
                          .setFootstepAdjustment(new HeightMapBasedFootstepAdjustment(environment.getTerrainObject3D().getHeightMapIfAvailable()));
      }
      testingSimulation.start();

      SimulationConstructionSet2 scs = testingSimulation.getSimulationConstructionSet();

      String suffix = "StepGeneratorCommandInputManager";
      YoBoolean walk = (YoBoolean) scs.findVariable("walk_" + suffix);
      YoDouble q_x = (YoDouble) scs.findVariable("q_" + testingSimulation.getControllerFullRobotModel().getRootBody().getName() + "_x");
      YoDouble desiredSpeed = (YoDouble) scs.findVariable("desiredVelocity_" + suffix + "X");

      YoDouble comError = (YoDouble) scs.findVariable("positionError_comHeight");

      walk.set(false);
      testingSimulation.simulateNow(standingTimeDuration);
      walk.set(true);

      desiredSpeed.set(desiredVelocityValue);

      //    ThreadTools.sleepForever();

      double timeIncrement = 1.0;
      boolean done = false;
      boolean success = true;
      while (!done)
      {
         assertTrue(testingSimulation.simulateNow(timeIncrement));

         if (Math.abs(comError.getDoubleValue()) > 0.09)
         {
            success = false;
            fail("comError = " + Math.abs(comError.getDoubleValue()));
         }

         if (scs.getTime().getDoubleValue() > standingTimeDuration + maximumWalkTime)
            done = true;
         if (q_x.getDoubleValue() > rampEndX)
            done = true;
      }

      createVideo(scs);
      assertTrue(success);
   }

   private ImmutablePair<CommonAvatarEnvironmentInterface, Double> createRamp()
   {
      double rampSlopeUp = 0.1;
      double rampSlopeDown = 0.08;

      double rampXStart0 = 0.5;
      double rampXLength0 = 6.0; //2.0;
      double landingHeight = rampSlopeUp * rampXLength0;
      double landingLength = 1.0;
      double rampXLength1 = landingHeight / rampSlopeDown;

      double rampYStart = -2.0;
      double rampYEnd = 6.0;

      double landingStartX = rampXStart0 + rampXLength0;
      double landingEndX = landingStartX + landingLength;
      double rampEndX = landingEndX + rampXLength1;

      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("JustARamp");

      AppearanceDefinition appearance = YoAppearance.Green();
      combinedTerrainObject.addRamp(rampXStart0, rampYStart, landingStartX, rampYEnd, landingHeight, appearance);
      combinedTerrainObject.addBox(landingStartX, rampYStart, landingEndX, rampYEnd, 0.0, landingHeight, YoAppearance.Gray());
      combinedTerrainObject.addRamp(rampEndX, rampYStart, landingEndX, rampYEnd, landingHeight, appearance);

      combinedTerrainObject.addBox(rampXStart0 - 2.0, rampYStart, rampEndX + 6.0, rampYEnd, -0.05, 0.0);

      CommonAvatarEnvironmentInterface environment = () -> combinedTerrainObject;
      return new ImmutablePair<>(environment, rampEndX);
   }

   private ImmutablePair<CommonAvatarEnvironmentInterface, Double> createRandomBlocks()
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("RandomBlocks");

      Random random = new Random(1776L);
      int numberOfBoxes = 200;

      double xMin = -0.2, xMax = 5.0;
      double yMin = -1.0, yMax = 1.0;
      double maxLength = 0.4;
      double maxHeight = 0.06;

      combinedTerrainObject.addBox(xMin - 2.0, yMin - maxLength, xMax + 2.0, yMax + maxLength, -0.01, 0.0, YoAppearance.Gold());

      for (int i = 0; i < numberOfBoxes; i++)
      {
         double xStart = RandomNumbers.nextDouble(random, xMin, xMax);
         double yStart = RandomNumbers.nextDouble(random, yMin, yMax);
         double xEnd = xStart + RandomNumbers.nextDouble(random, maxLength * 0.1, maxLength);
         double yEnd = yStart + RandomNumbers.nextDouble(random, maxLength * 0.1, maxLength);
         double zStart = 0.0;
         double zEnd = zStart + RandomNumbers.nextDouble(random, maxHeight * 0.1, maxHeight);
         combinedTerrainObject.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd, YoAppearance.Green());
      }

      CommonAvatarEnvironmentInterface environment = () -> combinedTerrainObject;
      return new ImmutablePair<>(environment, xMax);
   }

   @Disabled
   @Test
   public void testDRCBumpyGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double standingTimeDuration = 1.0;
      double walkingTimeDuration = 40.0;

      boolean useVelocityAndHeadingScript = true;

      //TODO: This should work with cheatWithGroundHeightAtForFootstep = false also, but for some reason height gets messed up and robot gets stuck...
      boolean cheatWithGroundHeightAtForFootstep = true;

      TerrainObject3D groundProfile = createBumpyGroundProfile();
      CommonAvatarEnvironmentInterface environment = () -> groundProfile;
      boolean drawGroundProfile = true;

      WalkingControllerParameters drcControlParameters = robotModel.getWalkingControllerParameters();
      RobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0, 0);

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, null);

      testingSimulation = simulationTestHelperFactory.createAvatarTestingSimulation();
      if (cheatWithGroundHeightAtForFootstep)
      {
         testingSimulation.getAvatarSimulation()
                          .getStepGeneratorThread()
                          .getSteppingManager()
                          .setFootstepAdjustment(new HeightMapBasedFootstepAdjustment(groundProfile.getHeightMapIfAvailable()));
      }
      testingSimulation.start();

      SimulationConstructionSet2 scs = testingSimulation.getSimulationConstructionSet();

      String suffix = "StepGeneratorCommandInputManager";
      YoBoolean walk = (YoBoolean) scs.findVariable("walk_" + suffix);
      YoDouble q_x = (YoDouble) scs.findVariable("q_" + testingSimulation.getControllerFullRobotModel().getRootBody().getName() + "_x");
      YoDouble desiredSpeed = (YoDouble) scs.findVariable("desiredVelocity_" + suffix + "X");

      YoDouble comError = (YoDouble) scs.findVariable("positionError_comHeight");

      YoDouble stepLength = (YoDouble) scs.findVariable("maxStepLengthCSG");
      YoDouble offsetHeightAboveGround = (YoDouble) scs.findVariable("offsetHeightAboveGround");
      stepLength.set(0.4);
      offsetHeightAboveGround.set(-0.1);

      walk.set(false);
      testingSimulation.simulateNow(standingTimeDuration);
      walk.set(true);

      double timeIncrement = 1.0;

      while (scs.getTime().getDoubleValue() - standingTimeDuration < walkingTimeDuration)
      {
         assertTrue(testingSimulation.simulateNow(timeIncrement));

         if (Math.abs(comError.getDoubleValue()) > 0.09)
         {
            fail("Math.abs(comError.getDoubleValue()) > 0.09: " + comError.getDoubleValue() + " at t = " + scs.getTime());
         }
      }

      createVideo(scs);
      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void createVideo(SimulationConstructionSet2 scs)
   {
      if (simulationTestingParameters.getCreateSCSVideos())
      {
         // TODO GITHUB WORKFLOWS
      }
   }

   private static BumpyTerrainProfile createBumpyGroundProfile()
   {
      double xAmp1 = 0.05, xFreq1 = 0.5, xAmp2 = 0.01, xFreq2 = 0.5;
      double yAmp1 = 0.01, yFreq1 = 0.07, yAmp2 = 0.05, yFreq2 = 0.37;
      double flatgroundBoxWidthAtZero = 0.6;
      BumpyTerrainProfile groundProfile = new BumpyTerrainProfile(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2, flatgroundBoxWidthAtZero);

      return groundProfile;
   }

   private static class BumpyTerrainProfile extends BumpyGroundProfile implements TerrainObject3D
   {
      public BumpyTerrainProfile(double xAmp1,
                                 double xFreq1,
                                 double xAmp2,
                                 double xFreq2,
                                 double yAmp1,
                                 double yFreq1,
                                 double yAmp2,
                                 double yFreq2,
                                 double flatgroundBoxWidthAtZero)
      {
         super(xAmp1,
               xFreq1,
               xAmp2,
               xFreq2,
               yAmp1,
               yFreq1,
               yAmp2,
               yFreq2,
               (double) -10.0F,
               (double) 10.0F,
               (double) -10.0F,
               (double) 10.0F,
               flatgroundBoxWidthAtZero);
      }

      @Override
      public List<? extends Shape3DReadOnly> getTerrainCollisionShapes()
      {
         return new ArrayList<>();
      }

      @Override
      public Graphics3DObject getLinkGraphics()
      {
         Graphics3DObject texturedGroundLinkGraphics = new Graphics3DObject();

         HeightMap heightMap = getHeightMapIfAvailable();

         texturedGroundLinkGraphics.addHeightMap(heightMap, 300, 300, YoAppearance.DarkGreen());

         return texturedGroundLinkGraphics;
      }
   }
}
