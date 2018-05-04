package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkThroughDoorBehavior.WalkThroughDoorBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

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

   private Vector3D32 doorOffsetPoint1 = new Vector3D32(0.5f, 0.9f, 0f);
   private Vector3D32 doorOffsetPoint2 = new Vector3D32(0.5f, 0.7f, 0f);

   private final SearchForDoorBehavior searchForDoorBehavior;
   private final WalkToInteractableObjectBehavior walkToInteractableObjectBehavior;

   private final ResetRobotBehavior resetRobotBehavior;

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   RobotSide side = RobotSide.RIGHT;

   public WalkThroughDoorBehavior(CommunicationBridge communicationBridge, YoDouble yoTime, YoBoolean yoDoubleSupport, FullHumanoidRobotModel fullRobotModel,
                                  HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters,
                                  AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("walkThroughDoorBehavior", WalkThroughDoorBehaviorState.class, yoTime, communicationBridge);

      //      communicationBridge.registerYovaribleForAutoSendToUI(statemachine.getStateYoVariable()); // FIXME
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      searchForDoorBehavior = new SearchForDoorBehavior(communicationBridge);
      walkToInteractableObjectBehavior = new WalkToInteractableObjectBehavior(yoTime, communicationBridge, atlasPrimitiveActions);
      resetRobotBehavior = new ResetRobotBehavior(communicationBridge, yoTime);
   }

   @Override
   public void doControl()
   {
      //should constantly be searching for door and updating its location here
      super.doControl();
   }

   @Override
   protected WalkThroughDoorBehaviorState configureStateMachineAndReturnInitialKey(StateMachineFactory<WalkThroughDoorBehaviorState, BehaviorAction> factory)
   {
      BehaviorAction resetRobot = new BehaviorAction(resetRobotBehavior);

      BehaviorAction setup = new BehaviorAction(atlasPrimitiveActions.leftHandDesiredConfigurationBehavior,
                                                atlasPrimitiveActions.rightHandDesiredConfigurationBehavior)
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
         }
      };

      BehaviorAction searchForDoorFar = new BehaviorAction(searchForDoorBehavior)
      {
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();
            //found the door location, inform the UI of its location

            DoorLocationPacket doorLocationPacket = HumanoidMessageTools.createDoorLocationPacket(searchForDoorBehavior.getLocation());
            communicationBridge.sendPacketToUI(doorLocationPacket);

         }
      };

      BehaviorAction searchForDoorNear = new BehaviorAction(searchForDoorBehavior)
      {
         @Override
         public void doTransitionOutOfAction()
         {
            super.doTransitionOutOfAction();
            //found the door location, inform the UI of its location

            DoorLocationPacket doorLocationPacket = HumanoidMessageTools.createDoorLocationPacket(searchForDoorBehavior.getLocation());
            communicationBridge.sendPacketToUI(doorLocationPacket);

         }
      };

      BehaviorAction walkToDoorAction = new BehaviorAction(walkToInteractableObjectBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint3D point1 = offsetPointFromDoor(doorOffsetPoint1);
            FramePoint3D point2 = offsetPointFromDoor(doorOffsetPoint2);

            walkToInteractableObjectBehavior.setWalkPoints(point1, point2);
         }
      };

      BehaviorAction setUpForWalk = new BehaviorAction(atlasPrimitiveActions.leftArmTrajectoryBehavior, atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

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

      BehaviorAction walkThroughDoor = new BehaviorAction(atlasPrimitiveActions.footstepListBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FootstepDataListMessage message = setUpFootSteps();
            atlasPrimitiveActions.footstepListBehavior.set(message);
         }
      };

      BehaviorAction failedState = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Walking Through Door Failed");
            sendPacket(p1);
         }
      };

      BehaviorAction doneState = new BehaviorAction(new SimpleDoNothingBehavior(communicationBridge))
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Finished Walking Through Door");
            sendPacket(p1);
         }
      };

      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.SETUP_ROBOT, setup, WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR);
      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR, searchForDoorFar, WalkThroughDoorBehaviorState.WALKING_TO_DOOR);

      factory.addState(WalkThroughDoorBehaviorState.WALKING_TO_DOOR, walkToDoorAction);
      factory.addTransition(WalkThroughDoorBehaviorState.WALKING_TO_DOOR, WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL, t -> isWalkingDone() && hasWalkingSucceded());
      factory.addTransition(WalkThroughDoorBehaviorState.WALKING_TO_DOOR, WalkThroughDoorBehaviorState.FAILED, t -> isWalkingDone() && !hasWalkingSucceded());

      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.SEARCHING_FOR_DOOR_FINAL, searchForDoorNear,
                                 setUpArms ? WalkThroughDoorBehaviorState.SET_UP_ROBOT_FOR_DOOR_WALK : WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR);
      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.SET_UP_ROBOT_FOR_DOOR_WALK, setUpForWalk, WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR);
      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.WALK_THROUGH_DOOR, walkThroughDoor, WalkThroughDoorBehaviorState.RESET_ROBOT);
      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.RESET_ROBOT, resetRobot, WalkThroughDoorBehaviorState.DONE);
      factory.addStateAndDoneTransition(WalkThroughDoorBehaviorState.FAILED, failedState, WalkThroughDoorBehaviorState.DONE);
      factory.addState(WalkThroughDoorBehaviorState.DONE, doneState);

      return WalkThroughDoorBehaviorState.SETUP_ROBOT;
   }

   private FramePoint3D offsetPointFromDoor(Vector3D32 point)
   {

      PoseReferenceFrame doorPose = new PoseReferenceFrame("doorFrame", ReferenceFrame.getWorldFrame());
      doorPose.setPoseAndUpdate(new Pose3D(searchForDoorBehavior.getLocation()));

      FramePoint3D point1 = new FramePoint3D(doorPose, point);
      return point1;
   }

   public FootstepDataListMessage setUpFootSteps()
   {

      PoseReferenceFrame doorPose = new PoseReferenceFrame("DoorReferenceFrame", ReferenceFrame.getWorldFrame());
      doorPose.setPoseAndUpdate(new Pose3D(searchForDoorBehavior.getLocation()));

      RobotSide startStep = RobotSide.LEFT;

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(atlasPrimitiveActions.footstepListBehavior.getDefaultSwingTime(),
                                                                                           atlasPrimitiveActions.footstepListBehavior.getDefaultTranferTime());

      FootstepDataMessage fs1 = createRelativeFootStep(doorPose, startStep, new Point3D(0.5864031335585762, 0.592160790421584, 0.11833316262205451),
                                                       new Quaternion(-4.624094786785623E-5, 3.113506928734585E-6, -0.7043244487834723, 0.7098782069467541));

      FootstepDataMessage fs2 = createRelativeFootStep(doorPose, startStep.getOppositeSide(),
                                                       new Point3D(0.4053278408799188, 0.23597592988662308, 0.11830896252372711),
                                                       new Quaternion(-1.5943418991263463E-13, 2.75059506574629E-13, -0.7043243641759355, 0.7098782924052293));
      FootstepDataMessage fs3 = createRelativeFootStep(doorPose, startStep, new Point3D(0.5924372369454293, -0.26851462759487155, 0.11830896252392731),
                                                       new Quaternion(-3.236982396751798E-13, 3.899712427026468E-14, -0.7043243760613419, 0.7098782806128114));
      FootstepDataMessage fs4 = createRelativeFootStep(doorPose, startStep.getOppositeSide(),
                                                       new Point3D(0.36887783182356804, -0.7234607322382425, 0.118308962523932),
                                                       new Quaternion(1.7351711631778928E-14, -1.6924263791365571E-13, -0.7043243760613419,
                                                                      0.7098782806128114));
      FootstepDataMessage fs5 = createRelativeFootStep(doorPose, startStep, new Point3D(0.5896714303877739, -0.7199905519593679, 0.11830896252393555),
                                                       new Quaternion(2.5501844493298926E-13, -3.0463423083022023E-13, -0.7043243760613419,
                                                                      0.7098782806128114));

      message.getFootstepDataList().add().set(fs1);
      message.getFootstepDataList().add().set(fs2);
      message.getFootstepDataList().add().set(fs3);
      message.getFootstepDataList().add().set(fs4);
      message.getFootstepDataList().add().set(fs5);

      return message;

   }

   private FootstepDataMessage createRelativeFootStep(PoseReferenceFrame frame, RobotSide side, Point3D location, Quaternion orientation)
   {

      FramePose3D pose = offsetPointFromFrameInWorldFrame(frame, location, orientation);
      FootstepDataMessage message = HumanoidMessageTools.createFootstepDataMessage(side, pose.getPosition(), pose.getOrientation());
      return message;
   }

   private FramePose3D offsetPointFromFrameInWorldFrame(PoseReferenceFrame frame, Point3D point3d, Quaternion quat4d)
   {
      FramePoint3D point1 = new FramePoint3D(frame, point3d);
      point1.changeFrame(ReferenceFrame.getWorldFrame());
      FrameQuaternion orient = new FrameQuaternion(frame, quat4d);
      orient.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D pose = new FramePose3D(point1, orient);

      return pose;
   }

   @Override
   public void onBehaviorExited()
   {

   }

   private boolean isWalkingDone()
   {
      return walkToInteractableObjectBehavior.isDone();
   }

   private boolean hasWalkingSucceded()
   {
      return walkToInteractableObjectBehavior.succeded();
   }

}
