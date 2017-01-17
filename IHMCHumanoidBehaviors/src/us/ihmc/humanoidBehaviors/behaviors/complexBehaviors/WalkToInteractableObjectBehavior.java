package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import com.jme3.math.Quaternion;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkToInteractableObjectBehavior.WalkToObjectState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationPlannedBehavior.WalkToLocationStates;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;

public class WalkToInteractableObjectBehavior extends StateMachineBehavior<WalkToObjectState>
{

   protected FramePoint walkToPoint1;
   protected FramePoint walkToPoint2;
   ResetRobotBehavior reset;
   private boolean succeded = true;

   public enum WalkToObjectState
   {
      GET_READY_TO_WALK, WALK_TO_POINT_1, WALK_TO_POINT_2, FAILED, DONE
   }

   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final ReferenceFrame midZupFrame;

   public WalkToInteractableObjectBehavior(DoubleYoVariable yoTime, CommunicationBridge outgoingCommunicationBridge,
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
            TextToSpeechPacket p1 = new TextToSpeechPacket("Getting Ready To Walk");
            sendPacket(p1);
         }
      };

      BehaviorAction<WalkToObjectState> walkToPoint1Task = new BehaviorAction<WalkToObjectState>(WalkToObjectState.WALK_TO_POINT_1,
            atlasPrimitiveActions.walkToLocationPlannedBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Walking To Point One");
            sendPacket(p1);
            walkToPoint1.changeFrame(ReferenceFrame.getWorldFrame());
            FramePoint walkPosition2d = new FramePoint(ReferenceFrame.getWorldFrame(), walkToPoint1.getX(), walkToPoint1.getY(), 0);
            FramePoint robotPosition = new FramePoint(midZupFrame, 0.0, 0.0, 0.0);
            robotPosition.changeFrame(ReferenceFrame.getWorldFrame());
            FrameVector walkingDirection = new FrameVector(ReferenceFrame.getWorldFrame());
            walkingDirection.set(walkPosition2d);
            walkingDirection.sub(robotPosition);
            walkingDirection.normalize();
            float walkingYaw = (float) Math.atan2(walkingDirection.getY(), walkingDirection.getX());

            Quaternion q = new Quaternion(new float[] {0, 0, walkingYaw});

            FramePose poseToWalkTo = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(walkToPoint1.getX(), walkToPoint1.getY(), 0),
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
            TextToSpeechPacket p1 = new TextToSpeechPacket("Walking To Point Two");
            sendPacket(p1);

            walkToPoint2.changeFrame(ReferenceFrame.getWorldFrame());
            FramePoint2d walkPosition2d = new FramePoint2d(ReferenceFrame.getWorldFrame(), walkToPoint2.getX(), walkToPoint2.getY());
            FramePoint2d robotPosition = new FramePoint2d(midZupFrame, 0.0, 0.0);
            robotPosition.changeFrame(ReferenceFrame.getWorldFrame());
            FrameVector2d walkingDirection = new FrameVector2d(ReferenceFrame.getWorldFrame());
            walkingDirection.set(walkPosition2d);
            walkingDirection.sub(robotPosition);
            walkingDirection.normalize();
            float walkingYaw = (float) Math.atan2(walkingDirection.getY(), walkingDirection.getX());
            Quaternion q = new Quaternion(new float[] {0, 0, walkingYaw});

            FramePose poseToWalkTo = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(walkToPoint2.getX(), walkToPoint2.getY(), 0),
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
            TextToSpeechPacket p1 = new TextToSpeechPacket("Walk Failed");
            sendPacket(p1);
         }
      };

      BehaviorAction<WalkToObjectState> doneState = new BehaviorAction<WalkToObjectState>(WalkToObjectState.DONE,
            new SimpleDoNothingBehavior(communicationBridge))
      {
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("Walk Complete");
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

   public void setWalkPoints(FramePoint walkToPoint1, FramePoint walkToPoint2)
   {
      this.walkToPoint1 = walkToPoint1;
      this.walkToPoint2 = walkToPoint2;
   }

   @Override
   public void onBehaviorExited()
   {
   }
}