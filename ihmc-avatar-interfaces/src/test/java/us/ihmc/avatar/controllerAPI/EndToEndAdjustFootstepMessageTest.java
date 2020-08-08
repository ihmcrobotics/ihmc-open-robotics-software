package us.ihmc.avatar.controllerAPI;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.AdjustFootstepMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class EndToEndAdjustFootstepMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Test
   public void testAdjustFootstepOnce() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>(fullRobotModel.getSoleFrames());
      drcSimulationTestHelper.publishToController(createFootsteps(soleFrames));

      final SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      final SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();
      final SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

      findWalkingStateVariables(scs, singleSupportStartConditions, doubleSupportStartConditions);

      final AtomicBoolean hasControllerAdjustedFootstep = new AtomicBoolean(false);

      scs.addScript(new Script()
      {
         private boolean adjustedFootstep = false;
         private double swingInitialTime = Double.NaN;
         private double delayBeforeAdjusting = 0.3;
         private RobotSide swingSideForAdjusting = RobotSide.RIGHT;

         private boolean checkedIfControllerAdjusted = false;
         private double delayBeforeChecking = 0.1;
         private Point3D adjustedLocation = new Point3D();

         @Override
         public void doScript(double t)
         {
            if (singleSupportStartConditions.get(swingSideForAdjusting).testCondition(Double.NaN))
            {
               if (Double.isNaN(swingInitialTime))
                  swingInitialTime = t;

               if (!adjustedFootstep)
               {
                  if (t >= swingInitialTime + delayBeforeAdjusting)
                  {
                     Quaternion orientation = new Quaternion();
                     Pose3D nextFootstepPose = findNextFootstepPose(scs);
                     adjustedLocation.set(nextFootstepPose.getPosition());
                     orientation.set(nextFootstepPose.getOrientation());
                     adjustedLocation.setX(adjustedLocation.getX() + 0.1);
                     adjustedLocation.setY(adjustedLocation.getY() - 0.15);
                     AdjustFootstepMessage adjustFootstepMessage = HumanoidMessageTools.createAdjustFootstepMessage(swingSideForAdjusting, adjustedLocation, orientation);
                     drcSimulationTestHelper.publishToController(adjustFootstepMessage);
                     adjustedFootstep = true;
                  }
               }
               else if (!checkedIfControllerAdjusted)
               {
                  if (t >= swingInitialTime + delayBeforeAdjusting + delayBeforeChecking)
                  {
                     Pose3D nextFootstepPose = findNextFootstepPose(scs);
                     boolean xEquals = MathTools.epsilonEquals(adjustedLocation.getX(), nextFootstepPose.getX(), 1.0e-10);
                     boolean yEquals = MathTools.epsilonEquals(adjustedLocation.getY(), nextFootstepPose.getY(), 1.0e-10);
                     boolean zEquals = MathTools.epsilonEquals(adjustedLocation.getZ(), nextFootstepPose.getZ(), 1.0e-10);
                     hasControllerAdjustedFootstep.set(xEquals && yEquals && zEquals);
                     checkedIfControllerAdjusted = true;
                  }
               }
            }
         }
      });

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(7.0);
      assertTrue(success);
      assertTrue("Controller did not adjust footstep", hasControllerAdjustedFootstep.get());
   }

   private void findWalkingStateVariables(SimulationConstructionSet scs, SideDependentList<StateTransitionCondition> singleSupportStartConditions,
         SideDependentList<StateTransitionCondition> doubleSupportStartConditions)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked")
         final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) scs.findVariable(sidePrefix + "FootControlModule",
               footPrefix + "CurrentState");
         @SuppressWarnings("unchecked")
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs.findVariable("WalkingHighLevelHumanoidController",
               "walkingCurrentState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }
   }

   private FootstepDataListMessage createFootsteps(SideDependentList<? extends ReferenceFrame> soleFrames)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();

      int numberOfFootsteps = 6;
      SteppingParameters steppingParameters = getRobotModel().getWalkingControllerParameters().getSteppingParameters();
      double stepLength = steppingParameters.getDefaultStepLength();
      double stepWidth = steppingParameters.getMinStepWidth();
      RobotSide side = RobotSide.LEFT;
      FramePoint3D framePosition = new FramePoint3D();
      framePosition.setToZero(soleFrames.get(RobotSide.RIGHT));
      framePosition.changeFrame(ReferenceFrame.getWorldFrame());

      for (int i = 0; i < numberOfFootsteps; i++)
      {
         framePosition.add(stepLength, side.negateIfRightSide(stepWidth), 0.0);
         Point3D position = new Point3D(framePosition);
         Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, position, orientation));
         side = side.getOppositeSide();
      }

      return footstepDataListMessage;
   }

   private static Pose3D findNextFootstepPose(SimulationConstructionSet scs)
   {
      String sidePrefix = findUpcomingFootstepSide(0, scs).getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Footstep0Pose";
      return new Pose3D(findYoFramePose(FootstepListVisualizer.class.getSimpleName(), namePrefix, scs));
   }

   @SuppressWarnings("unchecked")
   private static RobotSide findUpcomingFootstepSide(int index, SimulationConstructionSet scs)
   {
      return ((YoEnum<RobotSide>)scs.findVariable(WalkingMessageHandler.class.getSimpleName(), "upcomingFoostepSide" + index)).getEnumValue();
   }

   private static YoFramePose3D findYoFramePose(String namespace, String namePrefix, SimulationConstructionSet scs)
   {
      return findYoFramePose3D(namespace, namePrefix, "", scs);
   }

   private static YoFramePose3D findYoFramePose3D(String namespace, String namePrefix, String nameSuffix, SimulationConstructionSet scs)
   {
      YoDouble x = (YoDouble) scs.findVariable(namespace, YoGeometryNameTools.createXName(namePrefix, nameSuffix));
      YoDouble y = (YoDouble) scs.findVariable(namespace, YoGeometryNameTools.createYName(namePrefix, nameSuffix));
      YoDouble z = (YoDouble) scs.findVariable(namespace, YoGeometryNameTools.createZName(namePrefix, nameSuffix));
      YoFramePoint3D position = new YoFramePoint3D(x, y, z, ReferenceFrame.getWorldFrame());

      YoDouble qx = (YoDouble) scs.findVariable(namespace, YoGeometryNameTools.createQxName(namePrefix, nameSuffix));
      YoDouble qy = (YoDouble) scs.findVariable(namespace, YoGeometryNameTools.createQyName(namePrefix, nameSuffix));
      YoDouble qz = (YoDouble) scs.findVariable(namespace, YoGeometryNameTools.createQzName(namePrefix, nameSuffix));
      YoDouble qs = (YoDouble) scs.findVariable(namespace, YoGeometryNameTools.createQsName(namePrefix, nameSuffix));
      YoFrameQuaternion orientation = new YoFrameQuaternion(qx, qy, qz, qs, ReferenceFrame.getWorldFrame());
      return new YoFramePose3D(position, orientation);
   }

   private class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<ConstraintType> footConstraintType;

      public SingleSupportStartCondition(YoEnum<ConstraintType> footConstraintType)
      {
         this.footConstraintType = footConstraintType;
      }

      @Override
      public boolean testCondition(double timeInState)
      {
         return footConstraintType.getEnumValue() == ConstraintType.SWING;
      }
   }

   private class DoubleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<WalkingStateEnum> walkingState;

      private final RobotSide side;

      public DoubleSupportStartCondition(YoEnum<WalkingStateEnum> walkingState, RobotSide side)
      {
         this.walkingState = walkingState;
         this.side = side;
      }

      @Override
      public boolean testCondition(double timeInState)
      {
         if (side == RobotSide.LEFT)
         {
            return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_LEFT_SUPPORT);
         }
         else
         {
            return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_RIGHT_SUPPORT);
         }
      }
   }

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
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
