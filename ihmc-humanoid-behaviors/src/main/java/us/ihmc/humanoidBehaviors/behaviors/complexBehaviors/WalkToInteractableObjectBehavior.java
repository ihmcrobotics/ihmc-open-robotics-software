package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import com.jme3.math.Quaternion;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkToInteractableObjectBehavior.WalkToObjectState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkToInteractableObjectBehavior extends StateMachineBehavior<WalkToObjectState>
{

   protected FramePoint3D walkToPoint1;
   protected FramePoint3D walkToPoint2;
   ResetRobotBehavior reset;
   private boolean succeded = true;

   public enum WalkToObjectState
   {
      GET_READY_TO_WALK, WALK_TO_POINT_1, WALK_TO_POINT_2, FAILED, DONE
   }

   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final ReferenceFrame midZupFrame;

   public WalkToInteractableObjectBehavior(YoDouble yoTime, CommunicationBridge outgoingCommunicationBridge,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("WalkState", WalkToObjectState.class, yoTime, outgoingCommunicationBridge);
      midZupFrame = atlasPrimitiveActions.referenceFrames.getMidFeetZUpFrame();

      reset = new ResetRobotBehavior(false, false, false, false, outgoingCommunicationBridge, yoTime);
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      setupStateMachine();
   }

   @Override
   public void onBehaviorEntered()
   {
      super.onBehaviorEntered();
      succeded = true;
   }

   public boolean succeded()
   {
      return succeded;
   }

   private void setupStateMachine()
   {

      BehaviorAction<WalkToObjectState> resetRobot = new BehaviorAction<WalkToObjectState>(WalkToObjectState.GET_READY_TO_WALK, reset)
      {
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Getting Ready To Walk");
            sendPacket(p1);
         }
      };

      BehaviorAction<WalkToObjectState> walkToPoint1Task = new BehaviorAction<WalkToObjectState>(WalkToObjectState.WALK_TO_POINT_1,
            atlasPrimitiveActions.walkToLocationPlannedBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Walking To Point One");
            sendPacket(p1);
            walkToPoint1.changeFrame(ReferenceFrame.getWorldFrame());
            FramePoint3D walkPosition2d = new FramePoint3D(ReferenceFrame.getWorldFrame(), walkToPoint1.getX(), walkToPoint1.getY(), 0);
            FramePoint3D robotPosition = new FramePoint3D(midZupFrame, 0.0, 0.0, 0.0);
            robotPosition.changeFrame(ReferenceFrame.getWorldFrame());
            FrameVector3D walkingDirection = new FrameVector3D(ReferenceFrame.getWorldFrame());
            walkingDirection.set(walkPosition2d);
            walkingDirection.sub(robotPosition);
            walkingDirection.normalize();
            float walkingYaw = (float) Math.atan2(walkingDirection.getY(), walkingDirection.getX());

            Quaternion q = new Quaternion(new float[] {0, 0, walkingYaw});

            FramePose3D poseToWalkTo = new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(walkToPoint1.getX(), walkToPoint1.getY(), 0),
                  JMEDataTypeUtils.jMEQuaternionToVecMathQuat4d(q));
            atlasPrimitiveActions.walkToLocationPlannedBehavior.setTarget(poseToWalkTo);
         }
      };

      BehaviorAction<WalkToObjectState> walkToPoint2Task = new BehaviorAction<WalkToObjectState>(WalkToObjectState.WALK_TO_POINT_2,
            atlasPrimitiveActions.walkToLocationPlannedBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Walking To Point Two");
            sendPacket(p1);

            walkToPoint2.changeFrame(ReferenceFrame.getWorldFrame());
            FramePoint2D walkPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(), walkToPoint2.getX(), walkToPoint2.getY());
            FramePoint2D robotPosition = new FramePoint2D(midZupFrame, 0.0, 0.0);
            robotPosition.changeFrame(ReferenceFrame.getWorldFrame());
            FrameVector2D walkingDirection = new FrameVector2D(ReferenceFrame.getWorldFrame());
            walkingDirection.set(walkPosition2d);
            walkingDirection.sub(robotPosition);
            walkingDirection.normalize();
            float walkingYaw = (float) Math.atan2(walkingDirection.getY(), walkingDirection.getX());
            Quaternion q = new Quaternion(new float[] {0, 0, walkingYaw});

            FramePose3D poseToWalkTo = new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(walkToPoint2.getX(), walkToPoint2.getY(), 0),
                  JMEDataTypeUtils.jMEQuaternionToVecMathQuat4d(q));
            atlasPrimitiveActions.walkToLocationPlannedBehavior.setTarget(poseToWalkTo);
         }
      };

      BehaviorAction<WalkToObjectState> failedState = new BehaviorAction<WalkToObjectState>(WalkToObjectState.FAILED,
            new SimpleDoNothingBehavior(communicationBridge))
      {
         protected void setBehaviorInput()
         {
            succeded = false;
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Walk Failed");
            sendPacket(p1);
         }
      };

      BehaviorAction<WalkToObjectState> doneState = new BehaviorAction<WalkToObjectState>(WalkToObjectState.DONE,
            new SimpleDoNothingBehavior(communicationBridge))
      {
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = MessageTools.createTextToSpeechPacket("Walk Complete");
            sendPacket(p1);
         }
      };

      StateTransitionCondition planFailedCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return atlasPrimitiveActions.walkToLocationPlannedBehavior.isDone() && !atlasPrimitiveActions.walkToLocationPlannedBehavior.walkSucceded();
         }
      };

      StateTransitionCondition planSuccededCondition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return atlasPrimitiveActions.walkToLocationPlannedBehavior.isDone() && atlasPrimitiveActions.walkToLocationPlannedBehavior.walkSucceded();
         }
      };

      statemachine.addStateWithDoneTransition(resetRobot, WalkToObjectState.WALK_TO_POINT_1);
      statemachine.addState(walkToPoint1Task);
      walkToPoint1Task.addStateTransition(WalkToObjectState.FAILED, planFailedCondition);
      walkToPoint1Task.addStateTransition(WalkToObjectState.WALK_TO_POINT_2, planSuccededCondition);

      statemachine.addState(walkToPoint2Task);

      walkToPoint2Task.addStateTransition(WalkToObjectState.FAILED, planFailedCondition);
      walkToPoint2Task.addStateTransition(WalkToObjectState.DONE, planSuccededCondition);

      statemachine.addStateWithDoneTransition(failedState, WalkToObjectState.DONE);
      statemachine.addState(doneState);

      statemachine.setStartState(WalkToObjectState.GET_READY_TO_WALK);
   }

   public void setWalkPoints(FramePoint3D walkToPoint1, FramePoint3D walkToPoint2)
   {
      this.walkToPoint1 = walkToPoint1;
      this.walkToPoint2 = walkToPoint2;
   }

   @Override
   public void onBehaviorExited()
   {
   }
}