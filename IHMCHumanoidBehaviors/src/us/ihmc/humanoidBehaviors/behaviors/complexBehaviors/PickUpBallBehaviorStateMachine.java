package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElement.PickUpBallBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElementBehaviorSide;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

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
   private CommunicationBridge communicationBridge;

   public PickUpBallBehaviorStateMachine(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport,
         FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("pickUpBallStateMachine", PickUpBallBehaviorState.class, yoTime, communicationBridge);
      
      System.out.println("PickUpBallBehaviorStateMachine queue size "+communicationBridge.getListeningNetworkQueues().size());
      
      this.communicationBridge = communicationBridge;
      communicationBridge.addListeners(this);
//      communicationBridge.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable());

      this.atlasPrimitiveActions = atlasPrimitiveActions;

      coactiveElement = new PickUpBallBehaviorCoactiveElementBehaviorSide();
      coactiveElement.setPickUpBallBehavior(this);
      registry.addChild(coactiveElement.getUserInterfaceWritableYoVariableRegistry());
      registry.addChild(coactiveElement.getMachineWritableYoVariableRegistry());

      // create sub-behaviors:

      //NEW
      resetRobotBehavior = new ResetRobotBehavior(communicationBridge, yoTime);
      searchFarForSphereBehavior = new SearchFarForSphereBehavior(yoTime, coactiveElement, referenceFrames, communicationBridge, false, atlasPrimitiveActions);
      searchNearForSphereBehavior = new SearchNearForSphereBehavior(yoTime, coactiveElement, referenceFrames, communicationBridge, false,
            atlasPrimitiveActions);
      walkToPickUpLocationBehavior = new WalkToPickObjectOffGroundLocationBehavior(yoTime, referenceFrames, communicationBridge, wholeBodyControllerParameters,
            fullRobotModel, atlasPrimitiveActions);
      pickObjectOffGroundBehavior = new PickObjectOffGroundBehavior(yoTime, coactiveElement, referenceFrames, communicationBridge, atlasPrimitiveActions);
      putBallInBucketBehavior = new PutBallInBucketBehavior(yoTime, coactiveElement, referenceFrames, communicationBridge, atlasPrimitiveActions);
      setupStateMachine();
   }

   @Override
   public void doControl()
   {
      // TODO Auto-generated method stub
      super.doControl();
   }

   @Override
   public CoactiveElement getCoactiveElement()
   {
      return coactiveElement;
   }

   private void setupStateMachine()
   {

      //TODO setup search for ball behavior

      BehaviorAction<PickUpBallBehaviorState> searchForBallFar = new BehaviorAction<PickUpBallBehaviorState>(PickUpBallBehaviorState.SEARCHING_FOR_BALL_FAR,
            searchFarForSphereBehavior);

      // WALK TO THE BALL *******************************************

      BehaviorAction<PickUpBallBehaviorState> walkToBallTask = new BehaviorAction<PickUpBallBehaviorState>(PickUpBallBehaviorState.WALKING_TO_BALL,
            walkToPickUpLocationBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Walking To The Ball");
            sendPacket(p1);
            coactiveElement.currentState.set(PickUpBallBehaviorState.WALKING_TO_BALL);
            coactiveElement.searchingForBall.set(false);
            coactiveElement.waitingForValidation.set(false);
            coactiveElement.foundBall.set(true);
            walkToPickUpLocationBehavior.setPickUpLocation(searchFarForSphereBehavior.getBallLocation());
         }
      };

      //LOOK DOWN AND FIND BALL AGAIN

      BehaviorAction<PickUpBallBehaviorState> searchForBallNear = new BehaviorAction<PickUpBallBehaviorState>(PickUpBallBehaviorState.SEARCHING_FOR_BALL_NEAR,
            searchNearForSphereBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Looking For The Ball Again");
            sendPacket(p1);
            coactiveElement.currentState.set(PickUpBallBehaviorState.SEARCHING_FOR_BALL_NEAR);
            coactiveElement.searchingForBall.set(true);
            coactiveElement.waitingForValidation.set(false);
            coactiveElement.foundBall.set(false);
         }
      };

      //PICK UP THE BALL

      BehaviorAction<PickUpBallBehaviorState> pickUpBall = new BehaviorAction<PickUpBallBehaviorState>(PickUpBallBehaviorState.PICKING_UP_BALL,
            pickObjectOffGroundBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Picking Up The Ball");
            sendPacket(p1);
            coactiveElement.currentState.set(PickUpBallBehaviorState.PICKING_UP_BALL);
            coactiveElement.searchingForBall.set(false);
            coactiveElement.waitingForValidation.set(false);
            coactiveElement.foundBall.set(true);
            pickObjectOffGroundBehavior.setGrabLocation(searchNearForSphereBehavior.getBallLocation(), searchNearForSphereBehavior.getSphereRadius());
         }
      };

      BehaviorAction<PickUpBallBehaviorState> putBallInBucket = new BehaviorAction<PickUpBallBehaviorState>(PickUpBallBehaviorState.PUTTING_BALL_IN_BASKET,
            putBallInBucketBehavior);

      BehaviorAction<PickUpBallBehaviorState> resetRobot = new BehaviorAction<PickUpBallBehaviorState>(PickUpBallBehaviorState.RESET_ROBOT, resetRobotBehavior);

      BehaviorAction<PickUpBallBehaviorState> setup = new BehaviorAction<PickUpBallBehaviorState>(PickUpBallBehaviorState.SETUP_ROBOT,
            atlasPrimitiveActions.rightArmGoHomeBehavior, atlasPrimitiveActions.leftHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            GoHomeMessage goHomeRightArmMessage = new GoHomeMessage(BodyPart.ARM, RobotSide.RIGHT, 2);
            atlasPrimitiveActions.rightArmGoHomeBehavior.setInput(goHomeRightArmMessage);

            HandDesiredConfigurationMessage handMessage = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(handMessage);
         }
      };

      statemachine.addStateWithDoneTransition(setup, PickUpBallBehaviorState.SEARCHING_FOR_BALL_FAR);

      statemachine.addStateWithDoneTransition(searchForBallFar, PickUpBallBehaviorState.WALKING_TO_BALL);
      statemachine.addStateWithDoneTransition(walkToBallTask, PickUpBallBehaviorState.SEARCHING_FOR_BALL_NEAR);
      statemachine.addStateWithDoneTransition(searchForBallNear, PickUpBallBehaviorState.PICKING_UP_BALL);
      statemachine.addStateWithDoneTransition(pickUpBall, PickUpBallBehaviorState.PUTTING_BALL_IN_BASKET);
      statemachine.addStateWithDoneTransition(putBallInBucket, PickUpBallBehaviorState.RESET_ROBOT);
      statemachine.addState(resetRobot);
      statemachine.setStartState(PickUpBallBehaviorState.SETUP_ROBOT);
   }

   @Override
   public void onBehaviorExited()
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket("YAY IM ALL DONE");
      sendPacket(p1);

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
      System.out.println("BEHAVIOR RECIEVED " + data.key + " " + data.value);
   }

}
