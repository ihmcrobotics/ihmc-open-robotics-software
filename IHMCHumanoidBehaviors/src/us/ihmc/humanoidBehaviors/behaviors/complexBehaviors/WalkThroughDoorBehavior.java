package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3f;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkThroughDoorBehavior.WalkThroughDoorBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.DoorLocationPacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class WalkThroughDoorBehavior extends StateMachineBehavior<WalkThroughDoorBehaviorState>
{
   public enum WalkThroughDoorBehaviorState
   {
      STOPPED,
      SETUP_ROBOT,
      SEARCHING_FOR_DOOR,
      WALKING_TO_DOOR,
      SEARCHING_FOR_DOOR_FINAL,
      SET_UP_ROBOT_FOR_DOOR_WALK,
      WAITING_FOR_USER_CONFIRMATION,
      WALK_THROUGH_DOOR,
      RESET_ROBOT,
      FAILED,
      DONE
   }

   private final boolean setUpArms = true;

   private Vector3f doorOffsetPoint1 = new Vector3f(0.5f, 0.9f, 0f);
   private Vector3f doorOffsetPoint2 = new Vector3f(0.5f, 0.7f, 0f);

   private final SearchForDoorBehavior searchForDoorBehavior;
   private final WalkToInteractableObjectBehavior walkToInteractableObjectBehavior;

   private final ResetRobotBehavior resetRobotBehavior;

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   RobotSide side = RobotSide.RIGHT;

   public WalkThroughDoorBehavior(CommunicationBridge communicationBridge, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport,
         FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("walkThroughDoorBehavior", WalkThroughDoorBehaviorState.class, yoTime, communicationBridge);

      communicationBridge.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable());
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      searchForDoorBehavior = new SearchForDoorBehavior(communicationBridge);
      walkToInteractableObjectBehavior = new WalkToInteractableObjectBehavior(yoTime, communicationBridge, atlasPrimitiveActions);
      resetRobotBehavior = new ResetRobotBehavior(communicationBridge, yoTime);
      setupStateMachine();
   }

   @Override
   public void doControl()
   {
      //should constantly be searching for door and updating its location here
      super.doControl();
   }

   private void setupStateMachine()
   {
      BehaviorAction<WalkThroughDoorBehaviorState> resetRobot = new BehaviorAction<WalkThroughDoorBehaviorState>(WalkThroughDoorBehaviorState.RESET_ROBOT,
            resetRobotBehavior);

      BehaviorAction<WalkThroughDoorBehaviorState> setup = new BehaviorAction<WalkThroughDoorBehaviorState>(WalkThroughDoorBehaviorState.SETUP_ROBOT,
            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior, atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            HandDesiredConfigurationMessage leftHandMessage = new HandDesiredConfigurationMessage(RobotSide.LEFT, HandConfiguration.CLOSE);
            HandDesiredConfigurationMessage rightHandMessage = new HandDesiredConfigurationMessage(RobotSide.RIGHT, HandConfiguration.CLOSE);

            atlasPrimitiveActions.rightHandDesiredConfigurationBehavior.setInput(rightHandMessage);

            atlasPrimitiveActions.leftHandDesiredConfigurationBehavior.setInput(leftHandMessage);
         }
      };

      BehaviorAction<WalkThroughDoorBehaviorState> searchForDoorFar = new BehaviorAction<WalkThroughDoorBehaviorState>(
            WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR, searchForDoorBehavior)
      {
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();
            //found the door location, inform the UI of its location

            DoorLocationPacket doorLocationPacket = new DoorLocationPacket(searchForDoorBehavior.getLocation());
            communicationBridge.sendPacketToUI(doorLocationPacket);

         }
      };

      BehaviorAction<WalkThroughDoorBehaviorState> searchForDoorNear = new BehaviorAction<WalkThroughDoorBehaviorState>(
            WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL, searchForDoorBehavior)
      {
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();
            //found the door location, inform the UI of its location

            DoorLocationPacket doorLocationPacket = new DoorLocationPacket(searchForDoorBehavior.getLocation());
            communicationBridge.sendPacketToUI(doorLocationPacket);

         }
      };

      BehaviorAction<WalkThroughDoorBehaviorState> walkToDoorAction = new BehaviorAction<WalkThroughDoorBehaviorState>(
            WalkThroughDoorBehaviorState.WALKING_TO_DOOR, walkToInteractableObjectBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint point1 = offsetPointFromDoor(doorOffsetPoint1);
            FramePoint point2 = offsetPointFromDoor(doorOffsetPoint2);

            walkToInteractableObjectBehavior.setWalkPoints(point1, point2);
         }
      };

      BehaviorAction<WalkThroughDoorBehaviorState> setUpForWalk = new BehaviorAction<WalkThroughDoorBehaviorState>(
            WalkThroughDoorBehaviorState.SET_UP_ROBOT_FOR_DOOR_WALK, atlasPrimitiveActions.leftArmTrajectoryBehavior,
            atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            double[] rightArmPose = new double[] {1.5708, 0.8226007082651046, 1.2241049170121854, -1.546127437107859, -0.8486641166791746, -1.3365746544030488,
                  1.3376930879072813};
            double[] leftArmPose = new double[] {-1.5383305366909918, -0.9340404711083553, 1.9634792241521146, 0.9236260708644913, -0.8710518130931819,
                  -0.8771109242461594, -1.336089159719967};

            ArmTrajectoryMessage rightPoseMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, 2, rightArmPose);

            ArmTrajectoryMessage leftPoseMessage = new ArmTrajectoryMessage(RobotSide.LEFT, 2, leftArmPose);

            atlasPrimitiveActions.leftArmTrajectoryBehavior.setInput(leftPoseMessage);
            atlasPrimitiveActions.rightArmTrajectoryBehavior.setInput(rightPoseMessage);
         }
      };

      BehaviorAction<WalkThroughDoorBehaviorState> walkThroughDoor = new BehaviorAction<WalkThroughDoorBehaviorState>(
            WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR, atlasPrimitiveActions.footstepListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FootstepDataListMessage message = setUpFootSteps();
            atlasPrimitiveActions.footstepListBehavior.set(message);
         }
      };

      BehaviorAction<WalkThroughDoorBehaviorState> failedState = new BehaviorAction<WalkThroughDoorBehaviorState>(WalkThroughDoorBehaviorState.FAILED,
            new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Walking Through Door Failed");
            sendPacket(p1);
         }
      };

      BehaviorAction<WalkThroughDoorBehaviorState> doneState = new BehaviorAction<WalkThroughDoorBehaviorState>(WalkThroughDoorBehaviorState.DONE,
            new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Finished Walking Through Door");
            sendPacket(p1);
         }
      };

      StateTransitionCondition planFailedCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return walkToInteractableObjectBehavior.isDone() && !walkToInteractableObjectBehavior.succeded();
         }
      };
      StateTransitionCondition planSuccededCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return walkToInteractableObjectBehavior.isDone() && walkToInteractableObjectBehavior.succeded();
         }
      };

      statemachine.addStateWithDoneTransition(setup, WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR);
      statemachine.addStateWithDoneTransition(searchForDoorFar, WalkThroughDoorBehaviorState.WALKING_TO_DOOR);
      statemachine.addState(walkToDoorAction);

      walkToDoorAction.addStateTransition(WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL, planSuccededCondition);
      walkToDoorAction.addStateTransition(WalkThroughDoorBehaviorState.FAILED, planFailedCondition);

      if (setUpArms)
         statemachine.addStateWithDoneTransition(searchForDoorNear, WalkThroughDoorBehaviorState.SET_UP_ROBOT_FOR_DOOR_WALK);
      else
         statemachine.addStateWithDoneTransition(searchForDoorNear, WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR);
      statemachine.addStateWithDoneTransition(setUpForWalk, WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR);
      statemachine.addStateWithDoneTransition(walkThroughDoor, WalkThroughDoorBehaviorState.RESET_ROBOT);
      statemachine.addStateWithDoneTransition(resetRobot, WalkThroughDoorBehaviorState.DONE);
      statemachine.addStateWithDoneTransition(failedState, WalkThroughDoorBehaviorState.DONE);
      statemachine.addState(doneState);
      statemachine.setStartState(WalkThroughDoorBehaviorState.SETUP_ROBOT);

   }

   private FramePoint offsetPointFromDoor(Vector3f point)
   {

      PoseReferenceFrame doorPose = new PoseReferenceFrame("doorFrame", ReferenceFrame.getWorldFrame());
      doorPose.setPoseAndUpdate(new RigidBodyTransform(searchForDoorBehavior.getLocation()));

      FramePoint point1 = new FramePoint(doorPose, point.x, point.y, point.z);
      return point1;
   }

   public FootstepDataListMessage setUpFootSteps()
   {

      PoseReferenceFrame doorPose = new PoseReferenceFrame("DoorReferenceFrame", ReferenceFrame.getWorldFrame());
      doorPose.setPoseAndUpdate(new RigidBodyTransform(searchForDoorBehavior.getLocation()));

      RobotSide startStep = RobotSide.LEFT;

      FootstepDataListMessage message = new FootstepDataListMessage(atlasPrimitiveActions.footstepListBehavior.getDefaultSwingTime(),
            atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());

      FootstepDataMessage fs1 = createRelativeFootStep(doorPose, startStep, new Point3d(0.5864031335585762, 0.592160790421584, 0.11833316262205451),
            new Quat4d(-4.624094786785623E-5, 3.113506928734585E-6, -0.7043244487834723, 0.7098782069467541));

      FootstepDataMessage fs2 = createRelativeFootStep(doorPose, startStep.getOppositeSide(),
            new Point3d(0.4053278408799188, 0.23597592988662308, 0.11830896252372711),
            new Quat4d(-1.5943418991263463E-13, 2.75059506574629E-13, -0.7043243641759355, 0.7098782924052293));
      FootstepDataMessage fs3 = createRelativeFootStep(doorPose, startStep, new Point3d(0.5924372369454293, -0.26851462759487155, 0.11830896252392731),
            new Quat4d(-3.236982396751798E-13, 3.899712427026468E-14, -0.7043243760613419, 0.7098782806128114));
      FootstepDataMessage fs4 = createRelativeFootStep(doorPose, startStep.getOppositeSide(),
            new Point3d(0.36887783182356804, -0.7234607322382425, 0.118308962523932),
            new Quat4d(1.7351711631778928E-14, -1.6924263791365571E-13, -0.7043243760613419, 0.7098782806128114));
      FootstepDataMessage fs5 = createRelativeFootStep(doorPose, startStep, new Point3d(0.5896714303877739, -0.7199905519593679, 0.11830896252393555),
            new Quat4d(2.5501844493298926E-13, -3.0463423083022023E-13, -0.7043243760613419, 0.7098782806128114));

      message.add(fs1);
      message.add(fs2);
      message.add(fs3);
      message.add(fs4);
      message.add(fs5);

      return message;

   }

   private FootstepDataMessage createRelativeFootStep(PoseReferenceFrame frame, RobotSide side, Point3d location, Quat4d orientation)
   {

      FramePose pose = offsetPointFromFrameInWorldFrame(frame, location, orientation);
      FootstepDataMessage message = new FootstepDataMessage(side, pose.getFramePointCopy().getPoint(), pose.getFrameOrientationCopy().getQuaternion());
      return message;
   }

   private FramePose offsetPointFromFrameInWorldFrame(PoseReferenceFrame frame, Point3d point3d, Quat4d quat4d)
   {
      FramePoint point1 = new FramePoint(frame, point3d);
      point1.changeFrame(ReferenceFrame.getWorldFrame());
      FrameOrientation orient = new FrameOrientation(frame, quat4d);
      orient.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose pose = new FramePose(point1, orient);

      return pose;
   }

   @Override
   public void onBehaviorExited()
   {
      
   }



}
