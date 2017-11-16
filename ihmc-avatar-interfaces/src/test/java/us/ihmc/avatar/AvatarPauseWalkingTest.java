package us.ihmc.avatar;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public abstract class AvatarPauseWalkingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCRobotModel robotModel;
   private YoBoolean walkPaused;

   public abstract double getSwingTime();

   public abstract double getTransferTime();

   public abstract double getStepLength();

   public abstract double getStepWidth();

   public abstract double getTimeForPausing();

   public abstract double getTimeForResuming();

   public abstract int getNumberOfFoosteps();

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
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
      robotModel = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testPauseWalking() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      walkPaused.set(false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      sendFootstepCommand(getNumberOfFoosteps());
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getTimeForPausing()));
      PauseWalkingMessage pauseWalkingMessage = new PauseWalkingMessage(true);
      drcSimulationTestHelper.send(pauseWalkingMessage);
      walkPaused.set(true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getTimeForResuming()));
      pauseWalkingMessage = new PauseWalkingMessage(false);
      drcSimulationTestHelper.send(pauseWalkingMessage);
      walkPaused.set(false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(getNumberOfFoosteps() * (getSwingTime() + getTransferTime())));
   }

   private void sendFootstepCommand(int numberOfFootsteps)
   {
      FootstepDataListMessage footstepMessage = new FootstepDataListMessage(getSwingTime(), getTransferTime());
      RobotSide side = RobotSide.LEFT;
      Quaternion orientation = new Quaternion();
      for (int i = 1; i < numberOfFootsteps; i++)
      {
         addFootstep(new Point3D(i * getStepLength(), side.negateIfRightSide(getStepWidth() / 2.0), 0.0), orientation, side, footstepMessage);
         side = side.getOppositeSide();
      }
      addFootstep(new Point3D((numberOfFootsteps - 1) * getStepLength(), side.negateIfRightSide(getStepWidth() / 2.0), 0.0), orientation, side,
                  footstepMessage);
      drcSimulationTestHelper.send(footstepMessage);
   }

   private void addFootstep(Point3D stepLocation, Quaternion orient, RobotSide robotSide, FootstepDataListMessage message)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.setLocation(stepLocation);
      footstepData.setOrientation(orient);
      footstepData.setRobotSide(robotSide);
      message.add(footstepData);
   }

   private void setupTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();

      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            return new OffsetAndYawRobotInitialSetup(new Vector3D(0.0, 0.0, 0.0), 0.0);
         }
      };
      robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setStartingLocation(startingLocation);
      drcSimulationTestHelper.setTestEnvironment(emptyEnvironment);
      drcSimulationTestHelper.createSimulation(className);
      YoVariableRegistry registry = drcSimulationTestHelper.getYoVariableRegistry();
      walkPaused = new YoBoolean("isWalkPaused", registry);
      ThreadTools.sleep(1000);
      setupCameraSideView();
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

//   private class ControllerSpy extends SimpleRobotController
//   {
//      private final DRCRobotModel robotModel;
//      private final FullHumanoidRobotModel fullHumanoidRobotModel;
//      private final BipedSupportPolygons bipedSupportPolygons;
//      private final HighLevelHumanoidControllerToolbox humanoidControllerToolbox;
//      private final YoDouble copPositionX, copPositionY;
//      private final DRCSimulationTestHelper drcSimulationTestHelper;
//      private final FramePoint2D copPosition = new FramePoint2D();
//      private boolean condition = true;
//
//      public ControllerSpy(DRCSimulationTestHelper drcSimulationTestHelper, DRCRobotModel robotModel)
//      {
//         this.robotModel = robotModel;
//         this.fullHumanoidRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
//         this.bipedSupportPolygons = drcSimulationTestHelper.getAvatarSimulation().getMomentumBasedControllerFactory().getHighLevelHumanoidControllerToolbox()
//                                                            .getBipedSupportPolygons();
//         this.humanoidControllerToolbox = drcSimulationTestHelper.getAvatarSimulation().getMomentumBasedControllerFactory()
//                                                                 .getHighLevelHumanoidControllerToolbox();
//         this.copPositionX = (YoDouble) drcSimulationTestHelper.getYoVariable("icpPlannerDesiredCoPPositionX");
//         this.copPositionY = (YoDouble) drcSimulationTestHelper.getYoVariable("icpPlannerDesiredCoPPositionY");
//         this.drcSimulationTestHelper = drcSimulationTestHelper;
//         drcSimulationTestHelper.addRobotControllerOnControllerThread(this);
//      }
//
//      @Override
//      public void doControl()
//      {
//         copPosition.set(copPositionX.getDoubleValue(), copPositionY.getDoubleValue());
//         if(condition && drcSimulationTestHelper.getSimulationConstructionSet().getTime() > getTimeForPausing() && drcSimulationTestHelper.getSimulationConstructionSet().getTime() < getTimeForResuming())
//            condition &= (bipedSupportPolygons.getSupportPolygonInWorld().signedDistance(copPosition) < 0);
//      }
//
//      public boolean didCoPRemainInsideTheSupportPolygon()
//      {
//         return condition;
//      }
//   }
}
