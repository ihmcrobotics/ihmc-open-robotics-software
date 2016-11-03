package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import javax.vecmath.Vector3f;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.TurnValveBehaviorStateMachine.TurnValveBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.communication.CoactiveBehaviorsNetworkManager;
import us.ihmc.humanoidBehaviors.communication.CoactiveDataListenerInterface;
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

   Vector3f robotInteractionPointOffset = new Vector3f(-0.39f, -1.1811f, 0.85f);
   Vector3f robotInteractionDirectionPointOffset = new Vector3f(-0.38f, -1.1811f, 0.75f);

   CoactiveBehaviorsNetworkManager coactiveBehaviorsNetworkManager;

   private final SearchForValveBehavior searchForValveBehavior;
   //walk to location
   //move arm to top of valve
   //touch valve
   //grabvalve

   private final ResetRobotBehavior resetRobotBehavior;

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   public TurnValveBehaviorStateMachine(BehaviorCommunicationBridge communicationBridge, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport,
         FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("turnValveStateMachine", TurnValveBehaviorState.class, yoTime, communicationBridge);
      coactiveBehaviorsNetworkManager = new CoactiveBehaviorsNetworkManager(communicationBridge, communicationBridge);
      coactiveBehaviorsNetworkManager.addListeners(this);
      coactiveBehaviorsNetworkManager.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable());
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      searchForValveBehavior = new SearchForValveBehavior(communicationBridge);
      addChildBehavior(searchForValveBehavior);

      resetRobotBehavior = new ResetRobotBehavior(communicationBridge, yoTime);
      addChildBehavior(resetRobotBehavior);
      addChildBehaviors(atlasPrimitiveActions.getAllPrimitiveBehaviors());
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
            System.out.println(
                  "BEHAVIOR: found the valve at " + searchForValveBehavior.getValveCenter() + " with a radius of " + searchForValveBehavior.getValveRadius());

            double[] data = new double[5];
            data[0] = searchForValveBehavior.getValveCenter().x;
            data[1] = searchForValveBehavior.getValveCenter().y;
            data[2] = searchForValveBehavior.getValveCenter().z;
            data[3] = searchForValveBehavior.getValveRotation();
            data[4] = searchForValveBehavior.getValveRadius();
            coactiveBehaviorsNetworkManager.sendToUI("ValveLocation", data);

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

      statemachine.addStateWithDoneTransition(setup, TurnValveBehaviorState.SEARCHING_FOR_VAVLE);

      statemachine.addStateWithDoneTransition(searchForValveFar, TurnValveBehaviorState.RESET_ROBOT);

      statemachine.addState(resetRobot);
      statemachine.setCurrentState(TurnValveBehaviorState.SETUP_ROBOT);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      super.doPostBehaviorCleanup();
      TextToSpeechPacket p1 = new TextToSpeechPacket("YAY IM ALL DONE");
      sendPacketToNetworkProcessor(p1);
      System.out.println("found the valve it is located at " + searchForValveBehavior.getValveCenter());
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
