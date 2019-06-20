package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.SimpleCoactiveBehaviorDataPacket;
import controller_msgs.msg.dds.ValveLocationPacket;
import us.ihmc.communication.IHMCROS2Publisher;
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
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
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
   private final IHMCROS2Publisher<ValveLocationPacket> publisher;

   public TurnValveBehaviorStateMachine(String robotName, Ros2Node ros2Node, YoDouble yoTime, YoBoolean yoDoubleSupport,
                                        FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames,
                                        WholeBodyControllerParameters wholeBodyControllerParameters, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, "turnValveStateMachine", TurnValveBehaviorState.class, yoTime, ros2Node);

      //      ros2Node.addListeners(this); // FIXME I broke it when switching to pub-sub (Sylvain)
      //      communicationBridge.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable()); // FIXME
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      searchForValveBehavior = new SearchForValveBehavior(robotName, ros2Node);
      walkToInteractableObjectBehavior = new WalkToInteractableObjectBehavior(robotName, yoTime, ros2Node, atlasPrimitiveActions);
      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);
      graspAndTurnValveBehavior = new GraspAndTurnValveBehavior(robotName, yoTime, ros2Node, atlasPrimitiveActions);
      userValidationExampleBehavior = new GetUserValidationBehavior(robotName, ros2Node);
      publisher = createBehaviorOutputPublisher(ValveLocationPacket.class);
      setupStateMachine();
   }

   @Override
   public void doControl()
   {
      //should constantly be searching for valve and updating its location here
      super.doControl();
   }

   @Override
   protected TurnValveBehaviorState configureStateMachineAndReturnInitialKey(StateMachineFactory<TurnValveBehaviorState, BehaviorAction> factory)
   {
      BehaviorAction resetRobot = new BehaviorAction(resetRobotBehavior);

      BehaviorAction setup = new BehaviorAction(atlasPrimitiveActions.rightArmGoHomeBehavior, atlasPrimitiveActions.leftHandDesiredConfigurationBehavior)
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
      BehaviorAction searchForValveFar = new BehaviorAction(searchForValveBehavior)
      {
         @Override
         public void onExit()
         {
            super.onExit();
            //found the valve location, inform the UI of its location
            publisher.publish(HumanoidMessageTools.createValveLocationPacket(searchForValveBehavior.getLocation(), searchForValveBehavior.getValveRadius()));

         }
      };

      BehaviorAction searchForValveNear = new BehaviorAction(searchForValveBehavior)
      {
         @Override
         public void onExit()
         {
            super.onExit();
            //found the valve location, inform the UI of its location
            publisher.publish(HumanoidMessageTools.createValveLocationPacket(searchForValveBehavior.getLocation(), searchForValveBehavior.getValveRadius()));

         }
      };

      BehaviorAction walkToValveAction = new BehaviorAction(walkToInteractableObjectBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint3D point1 = offsetPointFromValve(valveWalkOffsetPoint1);
            FramePoint3D point2 = offsetPointFromValve(valveWalkOffsetPoint2);

            walkToInteractableObjectBehavior.setWalkPoints(point1, point2);
         }
      };

      BehaviorAction graspAndTurnValve = new BehaviorAction(graspAndTurnValveBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            PoseReferenceFrame valvePose = new PoseReferenceFrame("TurnValveReferenceFrame", ReferenceFrame.getWorldFrame());
            valvePose.setPoseAndUpdate(new Pose3D(searchForValveBehavior.getLocation()));
            graspAndTurnValveBehavior.setGrabLocation(valvePose, searchForValveBehavior.getValveRadius());
         }
      };

      BehaviorAction doneState = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Finished Turning Valve");
         }
      };

      BehaviorAction getUserValidation = new BehaviorAction(userValidationExampleBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Did I Turn It Far Enough?");
         }
      };

      factory.addStateAndDoneTransition(TurnValveBehaviorState.SETUP_ROBOT, setup, TurnValveBehaviorState.SEARCHING_FOR_VALVE);
      factory.addStateAndDoneTransition(TurnValveBehaviorState.SEARCHING_FOR_VALVE, searchForValveFar, TurnValveBehaviorState.WALKING_TO_VALVE);
      factory.addStateAndDoneTransition(TurnValveBehaviorState.WALKING_TO_VALVE, walkToValveAction, TurnValveBehaviorState.SEARCHING_FOR_VALVE_FINAL);
      factory.addStateAndDoneTransition(TurnValveBehaviorState.SEARCHING_FOR_VALVE_FINAL, searchForValveNear, TurnValveBehaviorState.TURNING_VALVE);
      factory.addStateAndDoneTransition(TurnValveBehaviorState.TURNING_VALVE, graspAndTurnValve, TurnValveBehaviorState.WAITING_FOR_USER_CONFIRMATION);

      factory.addState(TurnValveBehaviorState.WAITING_FOR_USER_CONFIRMATION, getUserValidation);
      factory.addTransition(TurnValveBehaviorState.WAITING_FOR_USER_CONFIRMATION, TurnValveBehaviorState.BACK_AWAY_FROM_VALVE,
                            time -> isValidationDone() && hasUserValidated());
      factory.addTransition(TurnValveBehaviorState.WAITING_FOR_USER_CONFIRMATION, TurnValveBehaviorState.TURNING_VALVE,
                            time -> isValidationDone() && !hasUserValidated());

      factory.addState(TurnValveBehaviorState.BACK_AWAY_FROM_VALVE, doneState);

      return TurnValveBehaviorState.SETUP_ROBOT;
   }

   private boolean isValidationDone()
   {
      return userValidationExampleBehavior.isDone();
   }

   private boolean hasUserValidated()
   {
      return userValidationExampleBehavior.isValidated();
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
      System.out.println("BEHAVIOR RECIEVED " + data.getKeyAsString() + " " + data.getValue());
   }

}
