package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import com.jme3.math.Quaternion;

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
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
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

   public WalkToInteractableObjectBehavior(String robotName, YoDouble yoTime, Ros2Node ros2Node, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, "WalkState", WalkToObjectState.class, yoTime, ros2Node);
      midZupFrame = atlasPrimitiveActions.referenceFrames.getMidFeetZUpFrame();

      reset = new ResetRobotBehavior(robotName, false, false, false, false, ros2Node, yoTime);
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

   @Override
   protected WalkToObjectState configureStateMachineAndReturnInitialKey(StateMachineFactory<WalkToObjectState, BehaviorAction> factory)
   {
      BehaviorAction resetRobot = new BehaviorAction(reset)
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeack("Getting Ready To Walk");
         }
      };

      BehaviorAction walkToPoint1Task = new BehaviorAction(atlasPrimitiveActions.walkToLocationPlannedBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeack("Walking To Point One");
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

      BehaviorAction walkToPoint2Task = new BehaviorAction(atlasPrimitiveActions.walkToLocationPlannedBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeack("Walking To Point Two");
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

      BehaviorAction failedState = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            succeded = false;
            publishTextToSpeack("Walk Failed");
         }
      };

      BehaviorAction doneState = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeack("Walk Complete");
         }
      };

      factory.addStateAndDoneTransition(WalkToObjectState.GET_READY_TO_WALK, resetRobot, WalkToObjectState.WALK_TO_POINT_1);
      factory.addState(WalkToObjectState.WALK_TO_POINT_1, walkToPoint1Task);
      factory.addTransition(WalkToObjectState.WALK_TO_POINT_1, WalkToObjectState.WALK_TO_POINT_2, t -> isDone() && hasWalkingSucceded());
      factory.addTransition(WalkToObjectState.WALK_TO_POINT_1, WalkToObjectState.FAILED, t -> isDone() && !hasWalkingSucceded());

      factory.addState(WalkToObjectState.WALK_TO_POINT_2, walkToPoint2Task);
      factory.addTransition(WalkToObjectState.WALK_TO_POINT_2, WalkToObjectState.DONE, t -> isDone() && hasWalkingSucceded());
      factory.addTransition(WalkToObjectState.WALK_TO_POINT_2, WalkToObjectState.FAILED, t -> isDone() && !hasWalkingSucceded());

      factory.addStateAndDoneTransition(WalkToObjectState.FAILED, failedState, WalkToObjectState.DONE);
      factory.addState(WalkToObjectState.DONE, doneState);

      return WalkToObjectState.GET_READY_TO_WALK;
   }

   private boolean isDoneWalking()
   {
      return atlasPrimitiveActions.walkToLocationPlannedBehavior.isDone();
   }

   private boolean hasWalkingSucceded()
   {
      return atlasPrimitiveActions.walkToLocationPlannedBehavior.walkSucceded();
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