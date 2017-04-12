package us.ihmc.avatar.controllerAPI;

import static org.junit.Assert.*;

import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.transformables.Pose;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class EndToEndAdjustFootstepMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @ContinuousIntegrationTest(estimatedDuration = 18.1)
   @Test(timeout = 90000)
   public void testAdjustFootstepOnce() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(getClass().getSimpleName(), selectedLocation, simulationTestingParameters, getRobotModel());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      SideDependentList<ReferenceFrame> soleFrames = fullRobotModel.getSoleFrames();
      drcSimulationTestHelper.send(createFootsteps(soleFrames));

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
            if (singleSupportStartConditions.get(swingSideForAdjusting).checkCondition())
            {
               if (Double.isNaN(swingInitialTime))
                  swingInitialTime = t;

               if (!adjustedFootstep)
               {
                  if (t >= swingInitialTime + delayBeforeAdjusting)
                  {
                     Quaternion orientation = new Quaternion();
                     Pose nextFootstepPose = findNextFootstepPose(scs);
                     nextFootstepPose.getPosition(adjustedLocation);
                     nextFootstepPose.getOrientation(orientation);
                     adjustedLocation.setX(adjustedLocation.getX() + 0.1);
                     adjustedLocation.setY(adjustedLocation.getY() - 0.15);
                     AdjustFootstepMessage adjustFootstepMessage = new AdjustFootstepMessage(swingSideForAdjusting, adjustedLocation, orientation);
                     adjustFootstepMessage.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
                     drcSimulationTestHelper.send(adjustFootstepMessage);
                     adjustedFootstep = true;
                  }
               }
               else if (!checkedIfControllerAdjusted)
               {
                  if (t >= swingInitialTime + delayBeforeAdjusting + delayBeforeChecking)
                  {
                     Pose nextFootstepPose = findNextFootstepPose(scs);
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
         final EnumYoVariable<ConstraintType> footConstraintType = (EnumYoVariable<ConstraintType>) scs.getVariable(sidePrefix + "FootControlModule",
               footPrefix + "State");
         @SuppressWarnings("unchecked")
         final EnumYoVariable<WalkingStateEnum> walkingState = (EnumYoVariable<WalkingStateEnum>) scs.getVariable("WalkingHighLevelHumanoidController",
               "walkingState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }
   }

   private FootstepDataListMessage createFootsteps(SideDependentList<ReferenceFrame> soleFrames)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();

      int numberOfFootsteps = 6;
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepLength = walkingControllerParameters.getDefaultStepLength();
      double stepWidth = walkingControllerParameters.getMinStepWidth();
      RobotSide side = RobotSide.LEFT;
      FramePoint framePosition = new FramePoint();
      framePosition.setToZero(soleFrames.get(RobotSide.RIGHT));
      framePosition.changeFrame(ReferenceFrame.getWorldFrame());

      for (int i = 0; i < numberOfFootsteps; i++)
      {
         framePosition.add(stepLength, side.negateIfRightSide(stepWidth), 0.0);
         Point3D position = new Point3D();
         framePosition.get(position);
         Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footstepDataListMessage.add(new FootstepDataMessage(side, position, orientation));
         side = side.getOppositeSide();
      }

      return footstepDataListMessage;
   }

   private static Pose findNextFootstepPose(SimulationConstructionSet scs)
   {
      String sidePrefix = findUpcomingFootstepSide(0, scs).getCamelCaseNameForStartOfExpression();
      String namePrefix = sidePrefix + "Footstep0Pose";
      FramePose framePose = new FramePose();
      findYoFramePose(FootstepListVisualizer.class.getSimpleName(), namePrefix, scs).getFramePose(framePose);
      Pose pose = new Pose();
      framePose.get(pose);
      return pose;
   }

   @SuppressWarnings("unchecked")
   private static RobotSide findUpcomingFootstepSide(int index, SimulationConstructionSet scs)
   {
      return ((EnumYoVariable<RobotSide>)scs.getVariable(WalkingMessageHandler.class.getSimpleName(), "upcomingFoostepSide" + index)).getEnumValue();
   }

   private static YoFramePose findYoFramePose(String nameSpace, String namePrefix, SimulationConstructionSet scs)
   {
      return findYoFramePose(nameSpace, namePrefix, "", scs);
   }

   private static YoFramePose findYoFramePose(String nameSpace, String namePrefix, String nameSuffix, SimulationConstructionSet scs)
   {
      DoubleYoVariable x = (DoubleYoVariable) scs.getVariable(nameSpace, YoFrameVariableNameTools.createXName(namePrefix, nameSuffix));
      DoubleYoVariable y = (DoubleYoVariable) scs.getVariable(nameSpace, YoFrameVariableNameTools.createYName(namePrefix, nameSuffix));
      DoubleYoVariable z = (DoubleYoVariable) scs.getVariable(nameSpace, YoFrameVariableNameTools.createZName(namePrefix, nameSuffix));
      YoFramePoint position = new YoFramePoint(x, y, z, ReferenceFrame.getWorldFrame());

      DoubleYoVariable yaw = (DoubleYoVariable) scs.getVariable(nameSpace, YoFrameVariableNameTools.createName(namePrefix, "yaw", nameSuffix));
      DoubleYoVariable pitch = (DoubleYoVariable) scs.getVariable(nameSpace, YoFrameVariableNameTools.createName(namePrefix, "pitch", nameSuffix));
      DoubleYoVariable roll = (DoubleYoVariable) scs.getVariable(nameSpace, YoFrameVariableNameTools.createName(namePrefix, "roll", nameSuffix));
      YoFrameOrientation orientation = new YoFrameOrientation(yaw, pitch, roll, ReferenceFrame.getWorldFrame());
      return new YoFramePose(position, orientation);
   }

   private class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final EnumYoVariable<ConstraintType> footConstraintType;

      public SingleSupportStartCondition(EnumYoVariable<ConstraintType> footConstraintType)
      {
         this.footConstraintType = footConstraintType;
      }

      @Override
      public boolean checkCondition()
      {
         return footConstraintType.getEnumValue() == ConstraintType.SWING;
      }
   }

   private class DoubleSupportStartCondition implements StateTransitionCondition
   {
      private final EnumYoVariable<WalkingStateEnum> walkingState;

      private final RobotSide side;

      public DoubleSupportStartCondition(EnumYoVariable<WalkingStateEnum> walkingState, RobotSide side)
      {
         this.walkingState = walkingState;
         this.side = side;
      }

      @Override
      public boolean checkCondition()
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
}
