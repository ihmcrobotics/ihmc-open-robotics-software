package us.ihmc.atlas.circleWalkTest;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoVariables.variable.YoEnum;

public class AtlasJumpTest
{
   private DRCSimulationTestHelper simulationTestHelper;

   private final double defaultInitialHeightAboveGround = 1.1;
   private final double defaultGravity = -9.81;

   public void setupTest()
   {
      setupTest(defaultInitialHeightAboveGround);
   }

   public void setupTest(double initialHeightAboveGround)
   {
      setupTest(createDefaultRobotModel(), initialHeightAboveGround, createDefaultSimulationTestingParameters(), null, false, defaultGravity);
   }

   public void setupTest(DRCRobotModel robot, DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup)
   {
      setupTest(robot, initialSetup, false);
   }

   public void setupTest(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup, boolean initializeEstimatorToActual)
   {
      setupTest(createDefaultRobotModel(), initialSetup, initializeEstimatorToActual);
   }

   public void setupTest(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup, boolean initializeEstimatorToActual, double gravity)
   {
      setupTest(createDefaultRobotModel(), initialSetup, initializeEstimatorToActual, gravity);
   }

   public void setupTest(DRCRobotModel robot, DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup, boolean initializeEstimatorToActual)
   {
      setupTest(robot, defaultInitialHeightAboveGround, createDefaultSimulationTestingParameters(), initialSetup, initializeEstimatorToActual, defaultGravity);
   }

   public void setupTest(DRCRobotModel robot, DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup, boolean initializeEstimatorToActual,
                         double gravity)
   {
      setupTest(robot, defaultInitialHeightAboveGround, createDefaultSimulationTestingParameters(), initialSetup, initializeEstimatorToActual, gravity);
   }

   public void setupTest(DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup)
   {
      setupTest(createDefaultRobotModel(), defaultInitialHeightAboveGround, createDefaultSimulationTestingParameters(), initialSetup, false, defaultGravity);
   }

   public void setupTest(SimulationTestingParameters simulationTestingParameters, DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup)
   {
      setupTest(createDefaultRobotModel(), defaultInitialHeightAboveGround, simulationTestingParameters, initialSetup, false, defaultGravity);
   }

   public void setupTest(DRCRobotModel robotModel, double initialHeightAboveGround, SimulationTestingParameters simulationTestingParameters,
                         DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup, boolean initializeEstimatorToActual, double gravityZ)
   {
      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         OffsetAndYawRobotInitialSetup initialSetup = new OffsetAndYawRobotInitialSetup(0.0, 0.0, initialHeightAboveGround);

         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            return initialSetup;
         }
      };
      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment()
      {
         private final Vector3D gravity = new Vector3D(0.0, 0.0, gravityZ);

         @Override
         public Vector3D getGravityVectorWorldFrame()
         {
            return gravity;
         }
      };
      simulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, flatGroundEnvironment);
      simulationTestHelper.setInitialSetup(initialSetup);
      simulationTestHelper.getSCSInitialSetup().setInitializeEstimatorToActual(initializeEstimatorToActual);
      simulationTestHelper.setStartingLocation(startingLocation);
      simulationTestHelper.createSimulation(getClass().getSimpleName());
   }

   @Test
   public void testOrientationControlUnderZeroGravity()
   {
      setupTest(new InitialRobotSetup(new Vector3D(0.0, 0.0, 0.0), new Vector3D(0.0, 2.0, 0.0)), true, 0.0);
      testStateActivation(HighLevelControllerName.JUMPING);
      try
      {
         simulationTestHelper.simulateAndBlock(5.0);
      }
      catch (Exception e)
      {
         assertFalse(e.getMessage(), true);
      }
   }

   @Test
   public void testJumpControllerStateRequest()
   {
      setupTest(1.0);
      testStateActivation(HighLevelControllerName.JUMPING);
   }
   
   @Test
   public void testStandingInPlaceWithJumpController()
   {
      setupTest(0.0);
      keepSCSUp = true;
      testStateActivation(HighLevelControllerName.JUMPING);
   }

   @Test
   public void testWalkControllerStateRequest()
   {
      setupTest(0.0);
      testStateActivation(HighLevelControllerName.WALKING);
   }

   @SuppressWarnings("unchecked")
   private void testStateActivation(HighLevelControllerName stateName)
   {
      HighLevelStateMessage jumpControllerMessage = new HighLevelStateMessage();
      jumpControllerMessage.highLevelControllerName = HighLevelStateMessage.JUMPING;
      simulationTestHelper.send(jumpControllerMessage);
      try
      {
         simulationTestHelper.simulateAndBlock(2.0);
         YoEnum<HighLevelControllerName> controllerName = (YoEnum<HighLevelControllerName>) simulationTestHelper.getYoVariable("highLevelControllerName");
         assertFalse("Controller name variable not found", controllerName == null);
         assertTrue("Jump controller not activated", controllerName.getEnumValue() == stateName);
      }
      catch (Exception e)
      {
         assertFalse("Caught exception: " + e.getMessage(), true);
      }
   }

   public DRCRobotModel createDefaultRobotModel()
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      return robotModel;
   }

   public SimulationTestingParameters createDefaultSimulationTestingParameters()
   {
      SimulationTestingParameters simulationTestingParameters = new SimulationTestingParameters();
      simulationTestingParameters.setKeepSCSUp(getKeepSCSUp());
      simulationTestingParameters.setUsePefectSensors(false);
      return simulationTestingParameters;
   }

   public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getInitialRobotSetup(Vector3D initalLinearVelocity, Vector3D initialAngularVelocity)
   {
      return new InitialRobotSetup(initalLinearVelocity, initialAngularVelocity);
   }

   private class InitialRobotSetup extends AtlasSimInitialSetup
   {
      public final Vector3D initialLinearVelocity;
      public final Vector3D initialAngularVelocity;
      private boolean robotInitialized = false;

      @SuppressWarnings("unused")
      public InitialRobotSetup()
      {
         super();
         this.initialLinearVelocity = new Vector3D();
         this.initialAngularVelocity = new Vector3D();
      }

      public InitialRobotSetup(Vector3D initialLinearVelocity, Vector3D initialAngularVelocity)
      {
         super();
         this.initialLinearVelocity = initialLinearVelocity;
         this.initialAngularVelocity = initialAngularVelocity;
      }

      @Override
      public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
      {
         super.initializeRobot(robot, jointMap);
         if (!robotInitialized)
         {
            robot.getRootJoint().setVelocity(initialLinearVelocity);
            robot.getRootJoint().setAngularVelocityInBody(initialAngularVelocity);
            robotInitialized = true;
         }
      }
   }

   private boolean keepSCSUp = false;
   
   private boolean getKeepSCSUp()
   {
      return keepSCSUp;
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.destroySimulation();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

}
