package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import com.jme3.math.Quaternion;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkToInteractableObjectBehavior.WalkToObjectState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SimpleDoNothingBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.jMonkeyEngineToolkit.jme.util.JMEDataTypeUtils;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkToInteractableObjectBehavior extends StateMachineBehavior<WalkToObjectState>
{

   protected FramePoint3D walkToPoint1;
   protected FramePoint3D walkToPoint2;
   ResetRobotBehavior reset;
   private boolean succeded = true;
   private boolean behaviorComplete = false;
   private double proximityToGoalToKeepOrientation = Double.MIN_VALUE;

   public enum WalkToObjectState
   {
      GET_READY_TO_WALK, WALK_TO_POINT_1, WALK_TO_POINT_2, FAILED, DONE
   }

   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final ReferenceFrame midUnderPelvisFrame;

   public WalkToInteractableObjectBehavior(String robotName, YoDouble yoTime, ROS2Node ros2Node, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, "WalkState", WalkToObjectState.class, yoTime, ros2Node);
      midUnderPelvisFrame = atlasPrimitiveActions.referenceFrames.getMidFeetUnderPelvisFrame();

      reset = new ResetRobotBehavior(robotName, false, false, false, false, ros2Node, yoTime);
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      setupStateMachine();
   }

   @Override
   public void onBehaviorEntered()
   {
      super.onBehaviorEntered();
      succeded = false;
      behaviorComplete = false;
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
         }
      };

      BehaviorAction walkToPoint1Task = new BehaviorAction(atlasPrimitiveActions.walkToLocationPlannedBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            Pair<FramePose3D, Double> desiredGoalAndHeading = computeDesiredGoalAndHeading(walkToPoint1,true);
            atlasPrimitiveActions.walkToLocationPlannedBehavior.setAssumeFlatGround(true);
            atlasPrimitiveActions.walkToLocationPlannedBehavior.setPlanBodyPath(false);
            atlasPrimitiveActions.walkToLocationPlannedBehavior.setTarget(desiredGoalAndHeading.getLeft());

            atlasPrimitiveActions.walkToLocationPlannedBehavior.setHeading(desiredGoalAndHeading.getRight());
         }
      };

      BehaviorAction walkToPoint2Task = new BehaviorAction(atlasPrimitiveActions.walkToLocationPlannedBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            Pair<FramePose3D, Double> desiredGoalAndHeading = computeDesiredGoalAndHeading(walkToPoint2,false);
            atlasPrimitiveActions.walkToLocationPlannedBehavior.setPlanBodyPath(false);
            atlasPrimitiveActions.walkToLocationPlannedBehavior.setAssumeFlatGround(true);

            atlasPrimitiveActions.walkToLocationPlannedBehavior.setTarget(desiredGoalAndHeading.getLeft());
            atlasPrimitiveActions.walkToLocationPlannedBehavior.setHeading(desiredGoalAndHeading.getRight());
            atlasPrimitiveActions.walkToLocationPlannedBehavior.setSquareUpEndSteps(true);
            atlasPrimitiveActions.walkToLocationPlannedBehavior.setSquareUpEndSteps(false);
         }
      };

      BehaviorAction failedState = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            succeded = false;
            behaviorComplete = true;
            publishTextToSpeech("Walk Failed");
         }
      };

      BehaviorAction doneState = new BehaviorAction(new SimpleDoNothingBehavior(robotName, ros2Node))
      {
         @Override
         protected void setBehaviorInput()
         {
            succeded = true;
            behaviorComplete = true;
            publishTextToSpeech("Walk Complete");
         }
      };

      factory.addStateAndDoneTransition(WalkToObjectState.GET_READY_TO_WALK, resetRobot, WalkToObjectState.WALK_TO_POINT_1);
      factory.addState(WalkToObjectState.WALK_TO_POINT_1, walkToPoint1Task);
      factory.addTransition(WalkToObjectState.WALK_TO_POINT_1, WalkToObjectState.WALK_TO_POINT_2, t -> walkToPoint1Task.isDone() && hasWalkingSucceded());
      factory.addTransition(WalkToObjectState.WALK_TO_POINT_1, WalkToObjectState.FAILED, t -> walkToPoint1Task.isDone() && !hasWalkingSucceded());

      factory.addState(WalkToObjectState.WALK_TO_POINT_2, walkToPoint2Task);
      factory.addTransition(WalkToObjectState.WALK_TO_POINT_2, WalkToObjectState.DONE, t -> walkToPoint2Task.isDone() && hasWalkingSucceded());
      factory.addTransition(WalkToObjectState.WALK_TO_POINT_2, WalkToObjectState.FAILED, t -> walkToPoint2Task.isDone() && !hasWalkingSucceded());

      factory.addState(WalkToObjectState.FAILED, failedState);
      factory.addState(WalkToObjectState.DONE, doneState);

      return WalkToObjectState.GET_READY_TO_WALK;
   }

   private Pair<FramePose3D, Double> computeDesiredGoalAndHeading(FramePoint3D pointToWalkTo, boolean keepHeadingIfClose)
   {
      pointToWalkTo.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D poseToWalkTo = new FramePose3D();
      poseToWalkTo.getPosition().set(pointToWalkTo);

      FramePoint3D walkPosition2d = new FramePoint3D(ReferenceFrame.getWorldFrame(), pointToWalkTo.getX(), pointToWalkTo.getY(), 0);
      FramePoint3D robotPosition = new FramePoint3D(midUnderPelvisFrame, 0.0, 0.0, 0.0);
      robotPosition.changeFrame(ReferenceFrame.getWorldFrame());
      FrameVector3D walkingDirection = new FrameVector3D(ReferenceFrame.getWorldFrame());
      walkingDirection.set(walkPosition2d);
      walkingDirection.sub(robotPosition);
      double distanceToGoal = walkPosition2d.distanceXY(robotPosition);
      walkingDirection.normalize();

      double pathToGoalYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());

      if (keepHeadingIfClose && distanceToGoal < proximityToGoalToKeepOrientation)
      {

         double robotYaw = midUnderPelvisFrame.getTransformToWorldFrame().getRotation().getYaw();
         poseToWalkTo.getOrientation().setToYawOrientation(robotYaw);

         double desiredHeading = AngleTools.computeAngleDifferenceMinusPiToPi(robotYaw, pathToGoalYaw);
        // publishTextToSpeech("Door is close, keeping orientation "+distanceToGoal+"<"+proximityToGoalToKeepOrientation+" "+robotYaw);

         return Pair.of(poseToWalkTo, desiredHeading);
      }
      else
      {

         Quaternion goalOrientation = new Quaternion(new float[] {0, 0, (float) pathToGoalYaw});
         poseToWalkTo.getOrientation().set(JMEDataTypeUtils.jMEQuaternionToVecMathQuat4d(goalOrientation));
        // publishTextToSpeech("Door is far, changing orientation "+distanceToGoal+">="+proximityToGoalToKeepOrientation+" 0.0");

         return Pair.of(poseToWalkTo, 0.0);
      }
   }
   
   public boolean isDone()
   {
      return behaviorComplete;
   }

   private boolean hasWalkingSucceded()
   {
      return atlasPrimitiveActions.walkToLocationPlannedBehavior.isDone()&&atlasPrimitiveActions.walkToLocationPlannedBehavior.walkSucceded();
   }

   public void setWalkPoints(FramePoint3D walkToPoint1, FramePoint3D walkToPoint2)
   {
      this.walkToPoint1 = walkToPoint1;
      this.walkToPoint2 = walkToPoint2;
   }

   public void setProximityToGoalToKeepOrientation(double proximityToGoalToKeepOrientation)
   {
      this.proximityToGoalToKeepOrientation = proximityToGoalToKeepOrientation;
   }

   @Override
   public void onBehaviorExited()
   {
   }
}