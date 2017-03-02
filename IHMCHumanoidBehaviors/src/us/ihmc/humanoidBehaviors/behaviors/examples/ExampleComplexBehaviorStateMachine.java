package us.ihmc.humanoidBehaviors.behaviors.examples;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.ResetRobotBehavior;
import us.ihmc.humanoidBehaviors.behaviors.examples.ExampleComplexBehaviorStateMachine.ExampleStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

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

   public ExampleComplexBehaviorStateMachine(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("ExampleStateMachine", ExampleStates.class, yoTime, communicationBridge);

      midZupFrame = atlasPrimitiveActions.referenceFrames.getMidFeetZUpFrame();
      coactiveBehaviorsNetworkManager = communicationBridge;
      coactiveBehaviorsNetworkManager.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable());

      this.atlasPrimitiveActions = atlasPrimitiveActions;

      //create your behaviors
      getLidarScanExampleBehavior = new GetLidarScanExampleBehavior(communicationBridge);
      getVideoPacketExampleBehavior = new GetVideoPacketExampleBehavior(communicationBridge);
      userValidationExampleBehavior = new GetUserValidationBehavior(communicationBridge);
      resetRobotBehavior = new ResetRobotBehavior(communicationBridge, yoTime);
      simpleArmMotionBehavior = new SimpleArmMotionBehavior(yoTime, atlasPrimitiveActions.referenceFrames, communicationBridge, atlasPrimitiveActions);

      setupStateMachine();

      statemachine.getStateYoVariable().addVariableChangedListener(new VariableChangedListener()
      {

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            System.out.println("ExampleComplexBehaviorStateMachine: Changing state to " + statemachine.getCurrentState());
         }
      });
   }

   @Override
   public void onBehaviorEntered()
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket("Starting Example Behavior");
      sendPacket(p1);
      super.onBehaviorEntered();
   }

   private void setupStateMachine()
   {

      //TODO setup search for ball behavior

      BehaviorAction<ExampleStates> enableLidar = new BehaviorAction<ExampleStates>(ExampleStates.ENABLE_LIDAR, atlasPrimitiveActions.enableLidarBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Enabling Lidar");
            sendPacket(p1);
            atlasPrimitiveActions.enableLidarBehavior.setLidarState(LidarState.ENABLE);
         }
      };

      BehaviorAction<ExampleStates> resetRobot = new BehaviorAction<ExampleStates>(ExampleStates.RESET_ROBOT_PIPELINE_EXAMPLE, resetRobotBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Resetting Robot");
            sendPacket(p1);
            super.setBehaviorInput();
         }
      };

      BehaviorAction<ExampleStates> setupRobot = new BehaviorAction<ExampleStates>(ExampleStates.SETUP_ROBOT_PARALLEL_STATEMACHINE_EXAMPLE,
            simpleArmMotionBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Setting Up Robot Pose");
            sendPacket(p1);
            super.setBehaviorInput();
         }
      };
      //      BehaviorAction<ExampleStates> setupRobot = new BehaviorAction<ExampleStates>(ExampleStates.SETUP_ROBOT_PARALLEL_STATEMACHINE_EXAMPLE,
      //            atlasPrimitiveActions.rightArmTrajectoryBehavior, atlasPrimitiveActions.leftHandTrajectoryBehavior,
      //            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior)
      //      {
      //         @Override
      //         protected void setBehaviorInput()
      //         {
      //
      //            TextToSpeechPacket p1 = new TextToSpeechPacket("Setting Up Robot Pose");
      //            sendPacket(p1);
      //
      //            double[] armConfig = new double[] {-0.5067668142160446, -0.3659876546358431, 1.7973796317575155, -1.2398714600960365, -0.005510224629709242,
      //                  0.6123343067479899, 0.12524505635696856};
      //
      //            ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
      //            armTrajectoryMessage.jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[armConfig.length];
      //            armTrajectoryMessage.robotSide = RobotSide.RIGHT;
      //
      //            for (int i = 0; i < armConfig.length; i++)
      //            {
      //               TrajectoryPoint1DMessage trajectoryPoint = new TrajectoryPoint1DMessage();
      //               trajectoryPoint.position = armConfig[i];
      //               trajectoryPoint.time = 1.0;
      //               OneDoFJointTrajectoryMessage jointTrajectory = new OneDoFJointTrajectoryMessage();
      //               jointTrajectory.trajectoryPoints = new TrajectoryPoint1DMessage[] {trajectoryPoint};
      //               armTrajectoryMessage.jointTrajectoryMessages[i] = jointTrajectory;
      //            }
      //
      //            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(armTrajectoryMessage);
      //
      //            FramePoint point1 = new FramePoint(ReferenceFrame.getWorldFrame(), .5, .5, 1);
      //            point1.changeFrame(ReferenceFrame.getWorldFrame());
      //            FrameOrientation orient = new FrameOrientation(ReferenceFrame.getWorldFrame(),1.5708, 1.5708, -3.1415);
      //            orient.changeFrame(ReferenceFrame.getWorldFrame());
      //
      //            FramePose pose = new FramePose(point1, orient);
      //            
      //            HandTrajectoryMessage handmessage = new HandTrajectoryMessage(RobotSide.LEFT, 2, pose.getFramePointCopy().getPoint(),pose.getFrameOrientationCopy().getQuaternion());
      //            
      //            atlasPrimitiveActions.leftHandTrajectoryBehavior.setInput(handmessage);
      //
      //            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE));
      //         }
      //      };

      BehaviorAction<ExampleStates> wholeBodyExample = new BehaviorAction<ExampleStates>(ExampleStates.WHOLEBODY_EXAMPLE,
            atlasPrimitiveActions.wholeBodyBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            TextToSpeechPacket p1 = new TextToSpeechPacket("Doing Whole Body Behavior");
            sendPacket(p1);
            FramePoint point = new FramePoint(midZupFrame, 0.2, 0.2, 0.3);
            point.changeFrame(ReferenceFrame.getWorldFrame());

            //the point in the world you want to move the hand to.
            //i set this high so that more solutions are accepted
            atlasPrimitiveActions.wholeBodyBehavior.setSolutionQualityThreshold(2.01);
            //how fast you want the action to be
            atlasPrimitiveActions.wholeBodyBehavior.setTrajectoryTime(3);

            FrameOrientation tmpOr = new FrameOrientation(point.getReferenceFrame(), Math.toRadians(45), Math.toRadians(90), 0);
            atlasPrimitiveActions.wholeBodyBehavior.setDesiredHandPose(RobotSide.LEFT, point, tmpOr);

         }
      };

      BehaviorAction<ExampleStates> getLidar = new BehaviorAction<ExampleStates>(ExampleStates.GET_LIDAR, getLidarScanExampleBehavior);
      BehaviorAction<ExampleStates> getVideo = new BehaviorAction<ExampleStates>(ExampleStates.GET_VIDEO, getVideoPacketExampleBehavior);
      BehaviorAction<ExampleStates> getUserValidation = new BehaviorAction<ExampleStates>(ExampleStates.GET_USER_VALIDATION, userValidationExampleBehavior);

      //setup the state machine

      statemachine.addStateWithDoneTransition(setupRobot, ExampleStates.RESET_ROBOT_PIPELINE_EXAMPLE);

      statemachine.addStateWithDoneTransition(resetRobot, ExampleStates.ENABLE_LIDAR);

      statemachine.addStateWithDoneTransition(enableLidar, ExampleStates.GET_LIDAR);
      statemachine.addStateWithDoneTransition(getLidar, ExampleStates.GET_VIDEO);
      statemachine.addStateWithDoneTransition(getVideo, ExampleStates.WHOLEBODY_EXAMPLE);
      statemachine.addStateWithDoneTransition(wholeBodyExample, ExampleStates.GET_USER_VALIDATION);
      statemachine.addState(getUserValidation);
      
      
      //set the starting state

      statemachine.setStartState(ExampleStates.SETUP_ROBOT_PARALLEL_STATEMACHINE_EXAMPLE);




   }

   @Override
   public void onBehaviorExited()
   {
      System.out.println("IM ALL DONE");
   }

}
