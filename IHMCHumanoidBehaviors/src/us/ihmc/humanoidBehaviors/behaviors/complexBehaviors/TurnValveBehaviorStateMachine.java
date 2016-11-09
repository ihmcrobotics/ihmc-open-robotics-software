package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import javax.vecmath.Vector3f;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.TurnValveBehaviorStateMachine.TurnValveBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.ValveLocationPacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class TurnValveBehaviorStateMachine extends StateMachineBehavior<TurnValveBehaviorState> implements CoactiveDataListenerInterface
{
   public enum TurnValveBehaviorState
   {
      STOPPED,
      SETUP_ROBOT,
      SEARCHING_FOR_VAVLE,
      WALKING_TO_VALVE,
      SEARCHING_FOR_VALVE_FINAL,
      MOVE_ARM_TO_INITAL_GRAB_LOCATION,
      GRABBING_VALVE,
      TURNING_VALVE,
      WAITING_FOR_USER_CONFIRMATION,
      RESET_ROBOT,
      BACK_AWAY_FROM_VALVE

   }

   private Vector3f valveWalkOffsetPoint1 = new Vector3f(-0.39f, 0.85f, 0.0f);
   private Vector3f valveWalkOffsetPoint2 = new Vector3f(-0.38f, 0.75f, 0.0f);

   private final SearchForValveBehavior searchForValveBehavior;
   private final WalkToInteractableObjectBehavior walkToInteractableObjectBehavior;
   //move arm to top of valve
   //touch valve
   //grabvalve

   private final ResetRobotBehavior resetRobotBehavior;

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   public TurnValveBehaviorStateMachine(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport,
         FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("turnValveStateMachine", TurnValveBehaviorState.class, yoTime, communicationBridge);

      communicationBridge.addListeners(this);
      communicationBridge.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable());
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      searchForValveBehavior = new SearchForValveBehavior(communicationBridge);
      walkToInteractableObjectBehavior = new WalkToInteractableObjectBehavior(yoTime, communicationBridge, atlasPrimitiveActions);
      resetRobotBehavior = new ResetRobotBehavior(communicationBridge, yoTime);
      setupStateMachine();
   }

   @Override
   public void doControl()
   {
      //should constantly be searching for valve and updating its location here
      super.doControl();
   }

   private void setupStateMachine()
   {

      //TODO setup search for ball behavior

      BehaviorAction<TurnValveBehaviorState> searchForValveFar = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.SEARCHING_FOR_VAVLE,
            searchForValveBehavior)
      {
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();
            //found the valve location, inform the UI of its location

            ValveLocationPacket valveLocationPacket = new ValveLocationPacket(searchForValveBehavior.getLocation(), searchForValveBehavior.getValveRadius());
            communicationBridge.sendPacketToUI(valveLocationPacket);

         }
      };

      BehaviorAction<TurnValveBehaviorState> searchForValveNear = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.SEARCHING_FOR_VALVE_FINAL,
            searchForValveBehavior);

      BehaviorAction<TurnValveBehaviorState> resetRobot = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.RESET_ROBOT, resetRobotBehavior);

      BehaviorAction<TurnValveBehaviorState> setup = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.SETUP_ROBOT,
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

      BehaviorAction<TurnValveBehaviorState> walkToValveAction = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.WALKING_TO_VALVE,
            walkToInteractableObjectBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            ReferenceFrame valveFrame = new ReferenceFrame("valveFrame", false, false, true)
            {

               @Override
               protected void updateTransformToParent(RigidBodyTransform transformToParent)
               {
                  transformToParent = searchForValveBehavior.getLocation();
               }
            };

            valveFrame.update();
            FramePoint2d point1 = new FramePoint2d(valveFrame, valveWalkOffsetPoint1.x, valveWalkOffsetPoint1.y);
            FramePoint2d point2 = new FramePoint2d(valveFrame, valveWalkOffsetPoint2.x, valveWalkOffsetPoint2.y);

            //            searchForValveBehavior.getLocation().

            walkToInteractableObjectBehavior.setWalkPoints(point1, point2);

         }
      };

      statemachine.addStateWithDoneTransition(setup, TurnValveBehaviorState.SEARCHING_FOR_VAVLE);

      statemachine.addStateWithDoneTransition(searchForValveFar, TurnValveBehaviorState.RESET_ROBOT);
      statemachine.addStateWithDoneTransition(resetRobot, TurnValveBehaviorState.WALKING_TO_VALVE);
      statemachine.addState(walkToValveAction);
      statemachine.setCurrentState(TurnValveBehaviorState.SETUP_ROBOT);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      super.doPostBehaviorCleanup();
      TextToSpeechPacket p1 = new TextToSpeechPacket("YAY IM ALL DONE");
      sendPacket(p1);
   }

   @Override
   public void abort()
   {
      super.abort();
      doPostBehaviorCleanup();
   }

   @Override
   public void pause()
   {
      super.pause();
   }

   @Override
   public void coactiveDataRecieved(SimpleCoactiveBehaviorDataPacket data)
   {
      System.out.println("BEHAVIOR RECIEVED " + data.key + " " + data.value);
   }

}
