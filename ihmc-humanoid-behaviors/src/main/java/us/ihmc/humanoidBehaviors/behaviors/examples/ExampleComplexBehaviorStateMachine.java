package us.ihmc.humanoidBehaviors.behaviors.examples;

import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.ResetRobotBehavior;
import us.ihmc.humanoidBehaviors.behaviors.examples.ExampleComplexBehaviorStateMachine.ExampleStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.variable.YoDouble;

public class ExampleComplexBehaviorStateMachine extends StateMachineBehavior<ExampleStates>
{
   public enum ExampleStates
   {
      ENABLE_LIDAR, SETUP_ROBOT_PARALLEL_STATEMACHINE_EXAMPLE, RESET_ROBOT_PIPELINE_EXAMPLE, GET_LIDAR, GET_VIDEO, GET_USER_VALIDATION, WHOLEBODY_EXAMPLE,
   }

   CommunicationBridge coactiveBehaviorsNetworkManager;

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   private final GetLidarScanExampleBehavior getLidarScanExampleBehavior;
   private final GetVideoPacketExampleBehavior getVideoPacketExampleBehavior;
   private final GetUserValidationBehavior userValidationExampleBehavior;
   private final SimpleArmMotionBehavior simpleArmMotionBehavior;
   private final ResetRobotBehavior resetRobotBehavior;
   private final ReferenceFrame midZupFrame;

   public ExampleComplexBehaviorStateMachine(CommunicationBridge communicationBridge, YoDouble yoTime, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("ExampleStateMachine", ExampleStates.class, yoTime, communicationBridge);

      midZupFrame = atlasPrimitiveActions.referenceFrames.getMidFeetZUpFrame();
      coactiveBehaviorsNetworkManager = communicationBridge;
      //      coactiveBehaviorsNetworkManager.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable()); // FIXME

      this.atlasPrimitiveActions = atlasPrimitiveActions;

      //create your behaviors
      getLidarScanExampleBehavior = new GetLidarScanExampleBehavior(communicationBridge);
      getVideoPacketExampleBehavior = new GetVideoPacketExampleBehavior(communicationBridge);
      userValidationExampleBehavior = new GetUserValidationBehavior(communicationBridge);
      resetRobotBehavior = new ResetRobotBehavior(communicationBridge, yoTime);
      simpleArmMotionBehavior = new SimpleArmMotionBehavior(yoTime, atlasPrimitiveActions.referenceFrames, communicationBridge, atlasPrimitiveActions);

      // FIXME
      //      statemachine.getStateYoVariable().addVariableChangedListener(new VariableChangedListener()
      //      {
      //
      //         @Override
      //         public void notifyOfVariableChange(YoVariable<?> v)
      //         {
      //            System.out.println("ExampleComplexBehaviorStateMachine: Changing state to " + statemachine.getCurrentState());
      //         }
      //      });
   }

   @Override
   public void onBehaviorEntered()
   {
      TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Starting Example Behavior");
      sendPacket(p1);
      super.onBehaviorEntered();
   }

   @Override
   public void onBehaviorExited()
   {
      System.out.println("IM ALL DONE");
   }

   @Override
   protected ExampleStates configureStateMachineAndReturnInitialKey(StateMachineFactory<ExampleStates, BehaviorAction> factory)
   {
      //TODO setup search for ball behavior

      BehaviorAction enableLidar = new BehaviorAction(atlasPrimitiveActions.enableLidarBehavior) // ExampleStates.ENABLE_LIDAR
      {
         @Override
         protected void setBehaviorInput()
         {
            sendPacket(MessageTools.createTextToSpeechPacket("Enabling Lidar"));
            // FIXME atlasPrimitiveActions.enableLidarBehavior.setLidarState(LidarState.ENABLE);
         }
      };

      BehaviorAction resetRobot = new BehaviorAction(resetRobotBehavior) // ExampleStates.RESET_ROBOT_PIPELINE_EXAMPLE
      {
         @Override
         protected void setBehaviorInput()
         {
            sendPacket(MessageTools.createTextToSpeechPacket("Resetting Robot"));
            super.setBehaviorInput();
         }
      };

      BehaviorAction setupRobot = new BehaviorAction(simpleArmMotionBehavior) // ExampleStates.SETUP_ROBOT_PARALLEL_STATEMACHINE_EXAMPLE
      {
         @Override
         protected void setBehaviorInput()
         {
            sendPacket(MessageTools.createTextToSpeechPacket("Setting Up Robot Pose"));
            super.setBehaviorInput();
         }
      };

      BehaviorAction wholeBodyExample = new BehaviorAction(atlasPrimitiveActions.wholeBodyBehavior) // ExampleStates.WHOLEBODY_EXAMPLE
      {
         @Override
         protected void setBehaviorInput()
         {

            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Doing Whole Body Behavior");
            sendPacket(p1);
            FramePoint3D point = new FramePoint3D(midZupFrame, 0.2, 0.2, 0.3);
            point.changeFrame(ReferenceFrame.getWorldFrame());

            //the point in the world you want to move the hand to.
            //i set this high so that more solutions are accepted
            atlasPrimitiveActions.wholeBodyBehavior.setSolutionQualityThreshold(2.01);
            //how fast you want the action to be
            atlasPrimitiveActions.wholeBodyBehavior.setTrajectoryTime(3);

            FrameQuaternion tmpOr = new FrameQuaternion(point.getReferenceFrame(), Math.toRadians(45), Math.toRadians(90), 0);
            atlasPrimitiveActions.wholeBodyBehavior.setDesiredHandPose(RobotSide.LEFT, point, tmpOr);
         }
      };

      BehaviorAction getLidar = new BehaviorAction(getLidarScanExampleBehavior); // ExampleStates.GET_LIDAR
      BehaviorAction getVideo = new BehaviorAction(getVideoPacketExampleBehavior); // ExampleStates.GET_VIDEO
      BehaviorAction getUserValidation = new BehaviorAction(userValidationExampleBehavior); // ExampleStates.GET_USER_VALIDATION

      factory.addStateAndDoneTransition(ExampleStates.SETUP_ROBOT_PARALLEL_STATEMACHINE_EXAMPLE, setupRobot, ExampleStates.RESET_ROBOT_PIPELINE_EXAMPLE);
      factory.addStateAndDoneTransition(ExampleStates.RESET_ROBOT_PIPELINE_EXAMPLE, resetRobot, ExampleStates.ENABLE_LIDAR);
      factory.addStateAndDoneTransition(ExampleStates.ENABLE_LIDAR, enableLidar, ExampleStates.GET_LIDAR);
      factory.addStateAndDoneTransition(ExampleStates.GET_LIDAR, getLidar, ExampleStates.GET_VIDEO);
      factory.addStateAndDoneTransition(ExampleStates.GET_VIDEO, getVideo, ExampleStates.WHOLEBODY_EXAMPLE);
      factory.addStateAndDoneTransition(ExampleStates.WHOLEBODY_EXAMPLE, wholeBodyExample, ExampleStates.GET_USER_VALIDATION);
      factory.addState(ExampleStates.GET_USER_VALIDATION, getUserValidation);

      return ExampleStates.SETUP_ROBOT_PARALLEL_STATEMACHINE_EXAMPLE;
   }
}
