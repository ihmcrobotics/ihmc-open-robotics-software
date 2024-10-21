package us.ihmc.avatar;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class AvatarStepInPlaceTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;
   private DRCRobotModel robotModel;
   private FullHumanoidRobotModel fullRobotModel;

   private PushRobotControllerSCS2 pushRobotController;
   private SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();

   protected int getNumberOfSteps()
   {
      return 1;
   }

   protected double getStepLength()
   {
      return 0.0;
   }

   protected double getStepWidth()
   {
      return 0.08;
   }

   protected double getForcePointOffsetZInChestFrame()
   {
      return 0.3;
   }

   protected OffsetAndYawRobotInitialSetup getStartingLocation()
   {
      return new OffsetAndYawRobotInitialSetup();
   }

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();

      robotModel = getRobotModel();
      fullRobotModel = robotModel.createFullRobotModel();

      LogTools.debug("simulationTestingParameters.getKeepSCSUp " + simulationTestingParameters.getKeepSCSUp());
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             flatGround,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(getStartingLocation());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      double z = getForcePointOffsetZInChestFrame();

      pushRobotController = new PushRobotControllerSCS2(simulationTestHelper.getSimulationConstructionSet().getTime(),
                                                        simulationTestHelper.getRobot(),
                                                        fullRobotModel.getPelvis().getParentJoint().getName(),
                                                        new Vector3D(0, 0, z));
      simulationTestHelper.addYoGraphicDefinition(pushRobotController.getForceVizDefinition());

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         @SuppressWarnings("unchecked")
         final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) simulationTestHelper.findVariable(sidePrefix + "FootControlModule",
                                                                                                                      sidePrefix + "FootCurrentState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
      }
   }

   @AfterEach
   public void tearDown()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testStepInPlace()
   {
      setupCameraSideView();

      simulationTestHelper.simulateNow(0.5);

      FootstepDataListMessage footMessage = new FootstepDataListMessage();

      MovingReferenceFrame stepFrame = simulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.LEFT);

      FramePoint3D footLocation = new FramePoint3D(stepFrame);
      FrameQuaternion footOrientation = new FrameQuaternion(stepFrame);
      footLocation.changeFrame(ReferenceFrame.getWorldFrame());
      footOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      footMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, footLocation, footOrientation));
      footMessage.setAreFootstepsAdjustable(true);

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      int steps = footMessage.getFootstepDataList().size();

      simulationTestHelper.simulateNow(1.0);
      simulationTestHelper.publishToController(footMessage);
      double simulationTime = initialTransfer + (transfer + swing) * steps + 1.0;

      assertTrue(simulationTestHelper.simulateNow(simulationTime));
   }

   @Test
   public void testStepInPlaceWithPush()
   {
      setupCameraSideView();

      simulationTestHelper.simulateNow(0.5);

      FootstepDataListMessage footMessage = new FootstepDataListMessage();

      MovingReferenceFrame stepFrame = simulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.LEFT);

      FramePoint3D footLocation = new FramePoint3D(stepFrame);
      FrameQuaternion footOrientation = new FrameQuaternion(stepFrame);
      footLocation.changeFrame(ReferenceFrame.getWorldFrame());
      footLocation.setY(0.0);
      
      footOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, footLocation, footOrientation);

      footMessage.getFootstepDataList().add().set(footstep);
      footMessage.setAreFootstepsAdjustable(true);

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      int steps = footMessage.getFootstepDataList().size();

      simulationTestHelper.publishToController(footMessage);
      double simulationTime = initialTransfer + (transfer + swing) * steps + 1.0;

      FrameVector3D forceDirection = new FrameVector3D(stepFrame, new Vector3D(0.0, 1.0, 0.0));
      forceDirection.changeFrame(ReferenceFrame.getWorldFrame());

      double icpErrorDeadband = robotModel.getWalkingControllerParameters().getStepAdjustmentParameters().getMinICPErrorForStepAdjustment();
      double desiredICPError = icpErrorDeadband * 0.5;
      double omega = robotModel.getWalkingControllerParameters().getOmega0();
      double desiredVelocityError = desiredICPError * omega;
      double mass = fullRobotModel.getTotalMass();
      double pushDuration = 0.05;

      double magnitude = getPushForceScaler() * mass * desiredVelocityError / pushDuration;
      LogTools.info("Push 1 magnitude = " + magnitude);
      pushRobotController.applyForceDelayed(singleSupportStartConditions.get(RobotSide.LEFT), 0.0, forceDirection, magnitude, pushDuration);

      YoBoolean leftFootstepWasAdjusted = (YoBoolean) simulationTestHelper.findVariable("leftFootSwingFootstepWasAdjusted");

      double dt = 0.05;
      for (int i = 0; i < simulationTime / dt; i++)
      {
         assertTrue(simulationTestHelper.simulateNow(dt));
         assertFalse(leftFootstepWasAdjusted.getBooleanValue());
      }

      assertTrue(simulationTestHelper.simulateNow(0.5));

      simulationTestHelper.publishToController(footMessage);

      desiredICPError = icpErrorDeadband * 1.15;
      desiredVelocityError = desiredICPError * omega;

      magnitude = getPushForceScaler() * mass * desiredVelocityError / pushDuration;
      LogTools.info("Push 2 magnitude = " + magnitude);
      pushRobotController.applyForceDelayed(singleSupportStartConditions.get(RobotSide.LEFT), 0.0, forceDirection, magnitude, pushDuration);

      assertTrue(simulationTestHelper.simulateNow(initialTransfer));
      
      boolean adjusted = false;
      for (int i = 0; i < (simulationTime - initialTransfer) / dt; i++)
      {
         assertTrue(simulationTestHelper.simulateNow(dt));
         if(leftFootstepWasAdjusted.getBooleanValue())
         {
            adjusted = true;
         }
      }
      assertTrue("Footstep wasn't adjusted, when it should have been", adjusted);

      simulationTestHelper.simulateNow(0.5);
   }

   /**
    * Overwrite this if the force required to trigger a step adjustment is higher. This can happen for
    * example is step adjustment phase in is used.
    *
    * @return multiplier for the push force.
    */
   public double getPushForceScaler()
   {
      return 1.0;
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<ConstraintType> footConstraintType;

      public SingleSupportStartCondition(YoEnum<ConstraintType> footConstraintType)
      {
         this.footConstraintType = footConstraintType;
      }

      @Override
      public boolean testCondition(double time)
      {
         return footConstraintType.getEnumValue() == ConstraintType.SWING;
      }
   }
}
