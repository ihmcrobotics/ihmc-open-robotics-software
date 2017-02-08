package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.Assert.assertTrue;

import java.util.List;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerWithTimeFreezer;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.thread.ThreadTools;

public abstract class AvatarAbsoluteStepTimingsTest implements MultiRobotTestInterface
{
   private static SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   static
   {
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static final int TICK_EPSILON = 2;

   public void testTakingStepsWithAbsoluteTimings() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();
      CommonAvatarEnvironmentInterface environment = new TestingEnvironment();
      DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, className, startingLocation, simulationTestingParameters, robotModel);
      ThreadTools.sleep(1000);
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.setCameraPosition(8.0, -8.0, 5.0);
      scs.setCameraFix(1.5, 0.0, 0.8);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      double swingStartInterval = 1.125;
      int steps = 20;

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepWidth = (walkingControllerParameters.getMinStepWidth() + walkingControllerParameters.getMaxStepWidth()) / 2.0;
      double stepLength = walkingControllerParameters.getDefaultStepLength() / 2.0;
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();
      double finalTransferTime = walkingControllerParameters.getDefaultFinalTransferTime();
      FootstepDataListMessage footsteps = new FootstepDataListMessage(swingTime, transferTime, finalTransferTime);
      for (int stepIndex = 0; stepIndex < steps; stepIndex++)
      {
         RobotSide side = stepIndex % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double y = side == RobotSide.LEFT ? stepWidth / 2.0 : -stepWidth / 2.0;
         Point3d location = new Point3d((double) stepIndex * stepLength, y, 0.0);
         Quat4d orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = new FootstepDataMessage(side, location, orientation);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         footstepData.setAbsoluteTime(swingStartInterval * (double) (stepIndex + 1));
         footsteps.add(footstepData);
      }

      double controllerDt = getRobotModel().getControllerDT();
      double timeEpsilon = (double) TICK_EPSILON * controllerDt;

      drcSimulationTestHelper.send(footsteps);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(swingStartInterval - timeEpsilon));
      String failMessage = "Swing did not start at the expected time.";

      for (int stepIndex = 0; stepIndex < steps; stepIndex++)
      {
         assertTrue(failMessage, getWalkingState(scs).isDoubleSupport());
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0 * timeEpsilon));
         assertTrue(failMessage, getWalkingState(scs).isSingleSupport());
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(swingStartInterval - 2.0 * timeEpsilon));
      }

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));
   }


   public void testMinimumTransferTimeIsRespected() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, className, startingLocation, simulationTestingParameters, robotModel);
      ThreadTools.sleep(1000);
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.setCameraPosition(8.0, -8.0, 5.0);
      scs.setCameraFix(1.5, 0.0, 0.8);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      FootstepDataListMessage footsteps = new FootstepDataListMessage(0.6, 0.3, 0.1);
      double minimumTransferTime = getRobotModel().getWalkingControllerParameters().getMinimumTransferTime();

      // add very fast footstep:
      {
         RobotSide side = RobotSide.LEFT;
         double y = side == RobotSide.LEFT ? 0.15 : -0.15;
         Point3d location = new Point3d(0.0, y, 0.0);
         Quat4d orientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = new FootstepDataMessage(side, location, orientation);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         footstepData.setAbsoluteTime(minimumTransferTime / 2.0);
         footsteps.add(footstepData);
      }

      drcSimulationTestHelper.send(footsteps);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(minimumTransferTime / 2.0));
      checkTransferTimes(scs, minimumTransferTime);
   }

   private void checkTransferTimes(SimulationConstructionSet scs, double minimumTransferTime)
   {
      DoubleYoVariable firstTransferTime = getDoubleYoVariable(scs, "icpPlannerTransferTime0", ICPPlannerWithTimeFreezer.class.getSimpleName());
      assertTrue("Executing transfer that is faster then allowed.", firstTransferTime.getDoubleValue() >= minimumTransferTime);
   }

   private static DoubleYoVariable getDoubleYoVariable(SimulationConstructionSet scs, String name, String namespace)
   {
      return getYoVariable(scs, name, namespace, DoubleYoVariable.class);
   }

   private WalkingStateEnum getWalkingState(SimulationConstructionSet scs)
   {
      return (WalkingStateEnum) getYoVariable(scs, "WalkingState", WalkingHighLevelHumanoidController.class.getSimpleName(), EnumYoVariable.class).getEnumValue();
   }

   private static <T extends YoVariable<T>> T getYoVariable(SimulationConstructionSet scs, String name, String namespace, Class<T> clazz)
   {
      YoVariable<?> uncheckedVariable = scs.getVariable(namespace, name);
      if (uncheckedVariable == null)
         throw new RuntimeException("Could not find yo variable: " + namespace + "/" + name + ".");
      if (!clazz.isInstance(uncheckedVariable))
         throw new RuntimeException("YoVariable " + name + " is not of type " + clazz.getSimpleName());
      return clazz.cast(uncheckedVariable);
   }

   public class TestingEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D terrain;
      private final Random random = new Random(19389481L);

      public TestingEnvironment()
      {
         WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
         double flatArea = walkingControllerParameters.getDefaultStepLength() * 1.0;
         double maxElevation = walkingControllerParameters.getMinSwingHeightFromStanceFoot() * 0.5;

         terrain = new CombinedTerrainObject3D(getClass().getSimpleName());
         terrain.addBox(-0.5, -1.0, flatArea, 1.0, -0.01, 0.0);

         for (int i = 0; i < 50; i++)
         {
            double xStart = flatArea + (double) i * flatArea;
            double height = maxElevation * 2.0 * (random.nextDouble() - 0.5);
            double length = flatArea;
            terrain.addBox(xStart, -1.0, xStart + length, 1.0, height - 0.01, height);
         }
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return terrain;
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

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters = null;
   }
}
