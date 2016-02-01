package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.DropDebrisTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GraspPieceOfDebrisTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class RemoveSingleDebrisBehavior extends BehaviorInterface
{
   private static final double OPTIMAL_DISTANCE_TO_GRAB_OBJECT = 0.80;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final ArrayList<BehaviorInterface> childBehaviors = new ArrayList<BehaviorInterface>();
   private final GraspPieceOfDebrisBehavior graspPieceOfDebris;
   private final DropDebrisBehavior dropPieceOfDebris;
   private final WalkToLocationBehavior walkCloseToObjectBehavior;

   private final BooleanYoVariable haveInputsBeenSet = new BooleanYoVariable("hasInputsBeenSet", registry);
   private final BooleanYoVariable isObjectTooFar = new BooleanYoVariable("isObjectTooFar", registry);
   private final DoubleYoVariable yoTime;
   private final WalkingControllerParameters walkingControllerParameters;
   private final SDFFullRobotModel fullRobotModel;
   private final ReferenceFrame midZupFrame;

   private RobotSide grabSide;
   private FramePoint graspPosition;

   public RemoveSingleDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullHumanoidRobotModel fullRobotModel,
         HumanoidReferenceFrames referenceFrames, DoubleYoVariable yoTime, WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(outgoingCommunicationBridge);

      this.fullRobotModel = fullRobotModel;
      this.walkingControllerParameters = wholeBodyControllerParameters.getWalkingControllerParameters();
      this.yoTime = yoTime;
      this.midZupFrame = referenceFrames.getMidFeetZUpFrame();

      // set up child behaviors
      graspPieceOfDebris = new GraspPieceOfDebrisBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames.getMidFeetZUpFrame(),
            wholeBodyControllerParameters, yoTime);
      childBehaviors.add(graspPieceOfDebris);
      
      dropPieceOfDebris = new DropDebrisBehavior(outgoingCommunicationBridge, referenceFrames, wholeBodyControllerParameters, yoTime);
      childBehaviors.add(dropPieceOfDebris);
      
      walkCloseToObjectBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      childBehaviors.add(walkCloseToObjectBehavior);

      for (BehaviorInterface behavior : childBehaviors)
      {
         registry.addChild(behavior.getYoVariableRegistry());
      }
   }

   @Override
   public void doControl()
   {
      if (isPaused())
      {
         return;
      }
      
      pipeLine.doControl();
   }

   public void setInputs(RigidBodyTransform debrisTransform, Point3d graspPosition, Vector3d graspVector)
   {
      this.graspPosition = new FramePoint(worldFrame, graspPosition);
      
      submitWalkingTask();
      grabSide = determineSideToUse(graspPosition);

      pipeLine.submitSingleTaskStage(new GraspPieceOfDebrisTask(graspPieceOfDebris, debrisTransform, graspPosition, graspVector, grabSide, yoTime));
      pipeLine.submitSingleTaskStage(new DropDebrisTask(dropPieceOfDebris, grabSide, yoTime));

      haveInputsBeenSet.set(true);
   }

   private RobotSide determineSideToUse(Point3d position)
   {
      FramePose pose = new FramePose(worldFrame);
      pose.setPose(position, new Quat4d(0, 0, 0, 1));
      pose.changeFrame(fullRobotModel.getPelvis().getBodyFixedFrame());
      if (pose.getY() <= 0.0)
         return RobotSide.RIGHT;
      else
         return RobotSide.LEFT;
   }

   public RobotSide getSideToUse()
   {
      return grabSide;
   }
   
   private void submitWalkingTask()
   {
      FramePoint2d graspPosition2d = new FramePoint2d(worldFrame, graspPosition.getX(), graspPosition.getY());
      FramePoint2d robotPosition = new FramePoint2d(midZupFrame, 0.0, 0.0);
      robotPosition.changeFrame(worldFrame);
      FrameVector2d walkingDirection = new FrameVector2d(worldFrame);
      walkingDirection.set(graspPosition2d);
      walkingDirection.sub(robotPosition);
      walkingDirection.normalize();
      double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());
      double x = graspPosition2d.getX() - walkingDirection.getX() * OPTIMAL_DISTANCE_TO_GRAB_OBJECT;
      double y = graspPosition2d.getY() - walkingDirection.getY() * OPTIMAL_DISTANCE_TO_GRAB_OBJECT;
      FramePose2d poseToWalkTo = new FramePose2d(worldFrame, new Point2d(x, y), walkingYaw);

      double stepLength = walkingControllerParameters.getDefaultStepLength();
      pipeLine.submitSingleTaskStage(new WalkToLocationTask(poseToWalkTo, walkCloseToObjectBehavior, 0.0, stepLength, yoTime));
      pipeLine.requestNewStage();
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      for (BehaviorInterface behavior : childBehaviors)
      {
         behavior.consumeObjectFromNetworkProcessor(object);
      }
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      for (BehaviorInterface behavior : childBehaviors)
      {
         behavior.consumeObjectFromController(object);
      }
   }

   @Override
   public void stop()
   {
      for (BehaviorInterface behavior : childBehaviors)
      {
         behavior.stop();
      }
   }

   @Override
   public void enableActions()
   {
      for (BehaviorInterface behavior : childBehaviors)
      {
         behavior.enableActions();
      }
   }
   
   @Override
   public void pause()
   {
      for (BehaviorInterface behavior : childBehaviors)
      {
         behavior.pause();
      }
   }

   @Override
   public void resume()
   {
      for (BehaviorInterface behavior : childBehaviors)
      {
         behavior.resume();
      }
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone() && hasInputBeenSet();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      haveInputsBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      isObjectTooFar.set(false);
      haveInputsBeenSet.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }

}
