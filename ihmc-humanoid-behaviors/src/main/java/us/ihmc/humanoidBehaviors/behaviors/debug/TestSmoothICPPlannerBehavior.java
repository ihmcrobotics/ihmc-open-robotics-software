package us.ihmc.humanoidBehaviors.behaviors.debug;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.ResetRobotBehavior;
import us.ihmc.humanoidBehaviors.behaviors.debug.TestSmoothICPPlannerBehavior.TestSmoothICPPlannerBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class TestSmoothICPPlannerBehavior extends StateMachineBehavior<TestSmoothICPPlannerBehaviorState>
{
   public enum TestSmoothICPPlannerBehaviorState
   {
      STOPPED, SETUP_ROBOT, CONFIRM_WALK, WALK_FORWARD, RESET_ROBOT, DONE
   }

   private final ResetRobotBehavior resetRobotBehavior;

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   private final HumanoidReferenceFrames referenceFrames;

   // YoVariables
   private final YoInteger numberOfSteps;
   private final YoDouble stepLength;
   private final YoDouble stepWidth;
   private final YoDouble swingTime;
   private final YoDouble transferTime;
   private final YoDouble initialTransferTime;
   private final YoDouble finalTransferTime;

   // Default parameters
   private final int defaultNumberOfSteps = 5;
   private final double defaultStepLength = 0.35;
   private final double defaultStepWidth = 0.25;
   private final double defaultSwingTime = 1.0;
   private final double defaultTransferTime = 0.35;
   private final double defaultInitialTransferTime = 1.0;
   private final double defaultFinalTransferTime = 1.0;

   private RobotSide side = RobotSide.LEFT;

   public TestSmoothICPPlannerBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime, YoBoolean yoDoubleSupport,
                                       FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames,
                                       WholeBodyControllerParameters wholeBodyControllerParameters, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, "testSmoothICPPlannerBehavior", TestSmoothICPPlannerBehaviorState.class, yoTime, ros2Node);

      //      communicationBridge.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable()); // FIXME
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      this.referenceFrames = referenceFrames;

      numberOfSteps = new YoInteger("TestSmoothICPPlannerNumberOfSteps", registry);
      numberOfSteps.set(defaultNumberOfSteps);
      stepLength = new YoDouble("TestSmoothICPPlannerStepLength", registry);
      stepLength.set(defaultStepLength);
      stepWidth = new YoDouble("TestSmoothICPPlannerStepWidth", registry);
      stepWidth.set(defaultStepWidth);
      swingTime = new YoDouble("TestSmoothICPPlannerSwingTime", registry);
      swingTime.set(defaultSwingTime);
      transferTime = new YoDouble("TestSmoothICPPlannerTransferTime", registry);
      transferTime.set(defaultTransferTime);
      initialTransferTime = new YoDouble("TestSmoothICPPlannerInitialTransferTime", registry);
      initialTransferTime.set(defaultInitialTransferTime);
      finalTransferTime = new YoDouble("TestSmoothICPPlannerFinalTransferTime", registry);
      finalTransferTime.set(defaultFinalTransferTime);

      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);
      setupStateMachine();
   }

   @Override
   public void doControl()
   {
      super.doControl();
   }

   @Override
   protected TestSmoothICPPlannerBehaviorState configureStateMachineAndReturnInitialKey(StateMachineFactory<TestSmoothICPPlannerBehaviorState, BehaviorAction> factory)
   {
      BehaviorAction resetRobot = new BehaviorAction(resetRobotBehavior);

      BehaviorAction setup = new BehaviorAction(atlasPrimitiveActions.leftHandDesiredConfigurationBehavior,
                                                atlasPrimitiveActions.rightHandDesiredConfigurationBehavior, atlasPrimitiveActions.leftArmTrajectoryBehavior,
                                                atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            HandDesiredConfigurationMessage leftHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.LEFT,
                                                                                                                         HandConfiguration.CLOSE);
            HandDesiredConfigurationMessage rightHandMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.RIGHT,
                                                                                                                          HandConfiguration.CLOSE);

            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);

            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(leftHandMessage);

            double[] rightArmPose = new double[] {1.5708, 0.8226007082651046, 1.2241049170121854, -1.546127437107859, -0.8486641166791746, -1.3365746544030488,
                  1.3376930879072813};
            double[] leftArmPose = new double[] {-1.5383305366909918, -0.9340404711083553, 1.9634792241521146, 0.9236260708644913, -0.8710518130931819,
                  -0.8771109242461594, -1.336089159719967};

            ArmTrajectoryMessage rightPoseMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 2, rightArmPose);

            ArmTrajectoryMessage leftPoseMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, 2, leftArmPose);

            atlasPrimitiveActions.leftArmTrajectoryBehavior.setInput(leftPoseMessage);
            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);
         }
      };

      BehaviorAction waitForConfirmation = new BehaviorAction()
      {
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();
         }
      };

      BehaviorAction walkForward = new BehaviorAction(atlasPrimitiveActions.footstepListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FootstepDataListMessage message = setUpFootSteps();
            atlasPrimitiveActions.footstepListBehavior.set(message);
         }
      };

      BehaviorAction doneState = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Finished Walking Forward");
         }
      };

      factory.addStateAndDoneTransition(TestSmoothICPPlannerBehaviorState.SETUP_ROBOT, setup, TestSmoothICPPlannerBehaviorState.CONFIRM_WALK);
      factory.addStateAndDoneTransition(TestSmoothICPPlannerBehaviorState.CONFIRM_WALK, waitForConfirmation, TestSmoothICPPlannerBehaviorState.WALK_FORWARD);
      factory.addStateAndDoneTransition(TestSmoothICPPlannerBehaviorState.WALK_FORWARD, walkForward, TestSmoothICPPlannerBehaviorState.DONE);
      factory.addStateAndDoneTransition(TestSmoothICPPlannerBehaviorState.RESET_ROBOT, resetRobot, TestSmoothICPPlannerBehaviorState.DONE);
      factory.addState(TestSmoothICPPlannerBehaviorState.DONE, doneState);

      return TestSmoothICPPlannerBehaviorState.WALK_FORWARD;
   }

   public FootstepDataListMessage setUpFootSteps()
   {
      FootstepDataListMessage footstepMessage = HumanoidMessageTools.createFootstepDataListMessage(swingTime.getDoubleValue(), transferTime.getDoubleValue(),
                                                                                                   finalTransferTime.getDoubleValue());

      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();

      for (int currentStep = 0; currentStep < numberOfSteps.getIntegerValue(); currentStep++)
      {
         Point3D footLocation = new Point3D(stepLength.getDoubleValue() * currentStep, side.negateIfRightSide(stepWidth.getDoubleValue() / 2), 0.0);
         Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         addFootstep(midFeetZUpFrame, side, footLocation, footOrientation, footstepMessage);
         side = side.getOppositeSide();
      }
      Point3D footLocation = new Point3D(stepLength.getDoubleValue() * (numberOfSteps.getIntegerValue() - 1),
                                         side.negateIfRightSide(stepWidth.getDoubleValue() / 2), 0.0);
      Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      addFootstep(midFeetZUpFrame, side, footLocation, footOrientation, footstepMessage);

      return footstepMessage;

   }

   private void addFootstep(ReferenceFrame stepReferenceFrame, RobotSide robotSide, Point3D stepLocation, Quaternion stepOrientation,
                            FootstepDataListMessage footstepMessage)
   {
      FramePose3D stepPose = new FramePose3D();
      getRelativeFootstepInWorldFrame(stepReferenceFrame, stepLocation, stepOrientation, stepPose);
      FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, stepPose.getPosition(), stepPose.getOrientation());

      footstepMessage.getFootstepDataList().add().set(footstepData);
   }

   private void getRelativeFootstepInWorldFrame(ReferenceFrame frame, Point3D point3d, Quaternion quat4d, FramePose3D framePoseToPack)
   {
      FramePoint3D position = new FramePoint3D(frame, point3d);
      position.changeFrame(ReferenceFrame.getWorldFrame());
      FrameQuaternion orientation = new FrameQuaternion(frame, quat4d);
      orientation.changeFrame(ReferenceFrame.getWorldFrame());

      framePoseToPack.setIncludingFrame(position, orientation);
   }

   @Override
   public void onBehaviorExited()
   {

   }
}
