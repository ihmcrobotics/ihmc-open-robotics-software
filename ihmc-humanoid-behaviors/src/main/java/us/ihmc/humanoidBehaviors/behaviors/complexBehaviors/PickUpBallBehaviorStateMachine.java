package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElement.PickUpBallBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElementBehaviorSide;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class PickUpBallBehaviorStateMachine extends StateMachineBehavior<PickUpBallBehaviorState> implements CoactiveDataListenerInterface
{

   private final PickUpBallBehaviorCoactiveElementBehaviorSide coactiveElement;

   private final SearchFarForSphereBehavior searchFarForSphereBehavior;
   private final SearchNearForSphereBehavior searchNearForSphereBehavior;
   private final WalkToPickObjectOffGroundLocationBehavior walkToPickUpLocationBehavior;
   private final PickObjectOffGroundBehavior pickObjectOffGroundBehavior;
   private final PutBallInBucketBehavior putBallInBucketBehavior;
   private final ResetRobotBehavior resetRobotBehavior;

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   public PickUpBallBehaviorStateMachine(String robotName, Ros2Node ros2Node, YoDouble yoTime,
                                         YoBoolean yoDoubleSupport, FullHumanoidRobotModel fullRobotModel,
                                         HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, "pickUpBallStateMachine", PickUpBallBehaviorState.class, yoTime, ros2Node);

//      ros2Node.addListeners(this); I kinda broke the coactive thingy when switching to pub-sub
      //      communicationBridge.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable());

      this.atlasPrimitiveActions = atlasPrimitiveActions;

      coactiveElement = new PickUpBallBehaviorCoactiveElementBehaviorSide();
      coactiveElement.setPickUpBallBehavior(this);
      registry.addChild(coactiveElement.getUserInterfaceWritableYoVariableRegistry());
      registry.addChild(coactiveElement.getMachineWritableYoVariableRegistry());

      // create sub-behaviors:

      //NEW
      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);
      searchFarForSphereBehavior = new SearchFarForSphereBehavior(robotName, yoTime, coactiveElement, referenceFrames, ros2Node, false, atlasPrimitiveActions);
      searchNearForSphereBehavior = new SearchNearForSphereBehavior(robotName, yoTime, coactiveElement, referenceFrames, fullRobotModel, ros2Node,
                                                                    false, atlasPrimitiveActions);
      walkToPickUpLocationBehavior = new WalkToPickObjectOffGroundLocationBehavior(robotName, yoTime, referenceFrames, ros2Node,
                                                                                   wholeBodyControllerParameters, fullRobotModel, atlasPrimitiveActions);
      pickObjectOffGroundBehavior = new PickObjectOffGroundBehavior(robotName, yoTime, coactiveElement, referenceFrames, ros2Node, atlasPrimitiveActions);
      putBallInBucketBehavior = new PutBallInBucketBehavior(robotName, yoTime, coactiveElement, referenceFrames, ros2Node, atlasPrimitiveActions);
      setupStateMachine();
   }

   @Override
   public void doControl()
   {
      super.doControl();
   }

   @Override
   public CoactiveElement getCoactiveElement()
   {
      return coactiveElement;
   }

   @Override
   protected PickUpBallBehaviorState configureStateMachineAndReturnInitialKey(StateMachineFactory<PickUpBallBehaviorState, BehaviorAction> factory)
   {
      //TODO setup search for ball behavior

      BehaviorAction searchForBallFar = new BehaviorAction(searchFarForSphereBehavior); // PickUpBallBehaviorState.SEARCHING_FOR_BALL_FAR

      // WALK TO THE BALL *******************************************

      BehaviorAction walkToBallTask = new BehaviorAction(walkToPickUpLocationBehavior) // PickUpBallBehaviorState.WALKING_TO_BALL
      {

         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Walking To The Ball");
            coactiveElement.currentState.set(PickUpBallBehaviorState.WALKING_TO_BALL);
            coactiveElement.searchingForBall.set(false);
            coactiveElement.waitingForValidation.set(false);
            coactiveElement.foundBall.set(true);
            walkToPickUpLocationBehavior.setPickUpLocation(searchFarForSphereBehavior.getBallLocation());
         }
      };

      //LOOK DOWN AND FIND BALL AGAIN

      BehaviorAction searchForBallNear = new BehaviorAction(searchNearForSphereBehavior) // PickUpBallBehaviorState.SEARCHING_FOR_BALL_NEAR
      {

         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Looking For The Ball Again");
            coactiveElement.currentState.set(PickUpBallBehaviorState.SEARCHING_FOR_BALL_NEAR);
            coactiveElement.searchingForBall.set(true);
            coactiveElement.waitingForValidation.set(false);
            coactiveElement.foundBall.set(false);
         }
      };

      //PICK UP THE BALL

      BehaviorAction pickUpBall = new BehaviorAction(pickObjectOffGroundBehavior) // PickUpBallBehaviorState.PICKING_UP_BALL
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Picking Up The Ball");
            coactiveElement.currentState.set(PickUpBallBehaviorState.PICKING_UP_BALL);
            coactiveElement.searchingForBall.set(false);
            coactiveElement.waitingForValidation.set(false);
            coactiveElement.foundBall.set(true);
            pickObjectOffGroundBehavior.setGrabLocation(searchNearForSphereBehavior.getBallLocation(), searchNearForSphereBehavior.getSphereRadius());
         }
      };

      BehaviorAction putBallInBucket = new BehaviorAction(putBallInBucketBehavior); // PickUpBallBehaviorState.PUTTING_BALL_IN_BASKET

      BehaviorAction resetRobot = new BehaviorAction(resetRobotBehavior); // PickUpBallBehaviorState.RESET_ROBOT

      BehaviorAction setup = new BehaviorAction(atlasPrimitiveActions.rightArmGoHomeBehavior, atlasPrimitiveActions.leftHandDesiredConfigurationBehavior) // PickUpBallBehaviorState.SETUP_ROBOT
      {
         @Override
         protected void setBehaviorInput()
         {
            GoHomeMessage goHomeRightArmMessage = HumanoidMessageTools.createGoHomeMessage(HumanoidBodyPart.ARM, RobotSide.RIGHT, 2);
            atlasPrimitiveActions.rightArmGoHomeBehavior.setInput(goHomeRightArmMessage);

            HandDesiredConfigurationMessage handMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(handMessage);
         }
      };

      factory.addStateAndDoneTransition(PickUpBallBehaviorState.SETUP_ROBOT, setup, PickUpBallBehaviorState.SEARCHING_FOR_BALL_FAR);
      factory.addStateAndDoneTransition(PickUpBallBehaviorState.SEARCHING_FOR_BALL_FAR, searchForBallFar, PickUpBallBehaviorState.WALKING_TO_BALL);
      factory.addStateAndDoneTransition(PickUpBallBehaviorState.WALKING_TO_BALL, walkToBallTask, PickUpBallBehaviorState.SEARCHING_FOR_BALL_NEAR);
      factory.addStateAndDoneTransition(PickUpBallBehaviorState.SEARCHING_FOR_BALL_NEAR, searchForBallNear, PickUpBallBehaviorState.PICKING_UP_BALL);
      factory.addStateAndDoneTransition(PickUpBallBehaviorState.PICKING_UP_BALL, pickUpBall, PickUpBallBehaviorState.PUTTING_BALL_IN_BASKET);
      factory.addStateAndDoneTransition(PickUpBallBehaviorState.PUTTING_BALL_IN_BASKET, putBallInBucket, PickUpBallBehaviorState.RESET_ROBOT);
      factory.addState(PickUpBallBehaviorState.RESET_ROBOT, resetRobot);
      
      return PickUpBallBehaviorState.SETUP_ROBOT;
   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("YAY IM ALL DONE");
      coactiveElement.currentState.set(PickUpBallBehaviorState.STOPPED);

      coactiveElement.searchingForBall.set(false);
      coactiveElement.waitingForValidation.set(false);
      coactiveElement.foundBall.set(false);
      coactiveElement.ballX.set(0);
      coactiveElement.ballY.set(0);
      coactiveElement.ballZ.set(0);
   }

   @Override
   public void onBehaviorAborted()
   {
      super.onBehaviorAborted();
      onBehaviorExited();

   }

   @Override
   public void coactiveDataRecieved(SimpleCoactiveBehaviorDataPacket data)
   {
      System.out.println("BEHAVIOR RECIEVED " + data.getKeyAsString() + " " + data.getValue());
   }

}
