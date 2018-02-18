package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.TurnValveBehaviorStateMachine.TurnValveBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.examples.GetUserValidationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.SimpleCoactiveBehaviorDataPacket;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.ValveLocationPacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class TurnValveBehaviorStateMachine extends StateMachineBehavior<TurnValveBehaviorState> implements CoactiveDataListenerInterface
{
   public enum TurnValveBehaviorState
   {
      STOPPED,
      SETUP_ROBOT,
      SEARCHING_FOR_VALVE,
      WALKING_TO_VALVE,
      SEARCHING_FOR_VALVE_FINAL,
      TURNING_VALVE,
      WAITING_FOR_USER_CONFIRMATION,
      RESET_ROBOT,
      BACK_AWAY_FROM_VALVE
   }

   private Vector3D32 valveWalkOffsetPoint1 = new Vector3D32(-0.39f, 0.0f, 0.85f);
   private Vector3D32 valveWalkOffsetPoint2 = new Vector3D32(-0.38f, 0.0f, 0.75f);

   private final SearchForValveBehavior searchForValveBehavior;
   private final WalkToInteractableObjectBehavior walkToInteractableObjectBehavior;
   private final GraspAndTurnValveBehavior graspAndTurnValveBehavior;
   //move arm to top of valve
   //touch valve
   //grabvalve

   private final ResetRobotBehavior resetRobotBehavior;

   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final GetUserValidationBehavior userValidationExampleBehavior;

   RobotSide side = RobotSide.RIGHT;

   public TurnValveBehaviorStateMachine(CommunicationBridge communicationBridge, YoDouble yoTime, YoBoolean yoDoubleSupport,
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
      graspAndTurnValveBehavior = new GraspAndTurnValveBehavior(yoTime, communicationBridge, atlasPrimitiveActions);
      userValidationExampleBehavior = new GetUserValidationBehavior(communicationBridge);

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
      BehaviorAction<TurnValveBehaviorState> resetRobot = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.RESET_ROBOT, resetRobotBehavior);

      BehaviorAction<TurnValveBehaviorState> setup = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.SETUP_ROBOT,
            atlasPrimitiveActions.rightArmGoHomeBehavior, atlasPrimitiveActions.leftHandDesiredConfigurationBehavior)
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

      //TODO setup search for ball behavior
      BehaviorAction<TurnValveBehaviorState> searchForValveFar = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.SEARCHING_FOR_VALVE,
            searchForValveBehavior)
      {
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();
            //found the valve location, inform the UI of its location

            ValveLocationPacket valveLocationPacket = HumanoidMessageTools.createValveLocationPacket(searchForValveBehavior.getLocation(), searchForValveBehavior.getValveRadius());
            communicationBridge.sendPacketToUI(valveLocationPacket);

         }
      };

      BehaviorAction<TurnValveBehaviorState> searchForValveNear = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.SEARCHING_FOR_VALVE_FINAL,
            searchForValveBehavior)
      {
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();
            //found the valve location, inform the UI of its location

            ValveLocationPacket valveLocationPacket = HumanoidMessageTools.createValveLocationPacket(searchForValveBehavior.getLocation(), searchForValveBehavior.getValveRadius());
            communicationBridge.sendPacketToUI(valveLocationPacket);

         }
      };

      BehaviorAction<TurnValveBehaviorState> walkToValveAction = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.WALKING_TO_VALVE,
            walkToInteractableObjectBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint3D point1 = offsetPointFromValve(valveWalkOffsetPoint1);
            FramePoint3D point2 = offsetPointFromValve(valveWalkOffsetPoint2);

            walkToInteractableObjectBehavior.setWalkPoints(point1, point2);
         }
      };

      BehaviorAction<TurnValveBehaviorState> graspAndTurnValve = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.TURNING_VALVE,
            graspAndTurnValveBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PoseReferenceFrame valvePose = new PoseReferenceFrame("TurnValveReferenceFrame", ReferenceFrame.getWorldFrame());
            valvePose.setPoseAndUpdate(new Pose3D(searchForValveBehavior.getLocation()));
            graspAndTurnValveBehavior.setGrabLocation(valvePose, searchForValveBehavior.getValveRadius());
         }
      };

      BehaviorAction<TurnValveBehaviorState> doneState = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.BACK_AWAY_FROM_VALVE,
            new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Finished Turning Valve");
            sendPacket(p1);
         }
      };

      BehaviorAction<TurnValveBehaviorState> getUserValidation = new BehaviorAction<TurnValveBehaviorState>(
            TurnValveBehaviorState.WAITING_FOR_USER_CONFIRMATION, userValidationExampleBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Did I Turn It Far Enough?");
            sendPacket(p1);
            super.setBehaviorInput();
         }
      };

      StateTransitionCondition notValidatedCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return userValidationExampleBehavior.isDone() && !userValidationExampleBehavior.isValidated();
         }
      };
      StateTransitionCondition validatedCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return userValidationExampleBehavior.isDone() && userValidationExampleBehavior.isValidated();
         }
      };

      statemachine.addStateWithDoneTransition(setup, TurnValveBehaviorState.SEARCHING_FOR_VALVE);
      statemachine.addStateWithDoneTransition(searchForValveFar, TurnValveBehaviorState.WALKING_TO_VALVE);
      statemachine.addStateWithDoneTransition(walkToValveAction, TurnValveBehaviorState.SEARCHING_FOR_VALVE_FINAL);
      statemachine.addStateWithDoneTransition(searchForValveNear, TurnValveBehaviorState.TURNING_VALVE);
      statemachine.addStateWithDoneTransition(graspAndTurnValve, TurnValveBehaviorState.WAITING_FOR_USER_CONFIRMATION);
      statemachine.addState(getUserValidation);

      getUserValidation.addStateTransition(TurnValveBehaviorState.TURNING_VALVE, notValidatedCondition);
      getUserValidation.addStateTransition(TurnValveBehaviorState.BACK_AWAY_FROM_VALVE, validatedCondition);

      statemachine.addState(doneState);
      statemachine.setStartState(TurnValveBehaviorState.SETUP_ROBOT);

   }

   private FramePoint3D offsetPointFromValve(Vector3D32 point)
   {
      PoseReferenceFrame valvePose = new PoseReferenceFrame("valveFrame", ReferenceFrame.getWorldFrame());
      valvePose.setPoseAndUpdate(new Pose3D(searchForValveBehavior.getLocation()));

      FramePoint3D point1 = new FramePoint3D(valvePose, point);
      return point1;
   }

   @Override
   public void onBehaviorExited()
   {

   }

   

   @Override
   public void coactiveDataRecieved(SimpleCoactiveBehaviorDataPacket data)
   {
      System.out.println("BEHAVIOR RECIEVED " + data.key + " " + data.value);
   }

}
