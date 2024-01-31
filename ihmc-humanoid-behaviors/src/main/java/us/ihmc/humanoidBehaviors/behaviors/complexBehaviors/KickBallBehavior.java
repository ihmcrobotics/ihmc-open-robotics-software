package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.color.HSVValue;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BlobFilteredSphereDetectionBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SphereDetectionBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.WaitForUserValidationBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateMachine;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class KickBallBehavior extends AbstractBehavior
{
   private static final boolean CREATE_COACTIVE_ELEMENT = true;
   private static final boolean USE_BLOB_FILTERING = true;

   private enum KickState
   {
      SEARCH, APPROACH, VERIFY_LOCATION, FINAL_APPROACH, VERIFY_KICK_LOCATION, KICK_BALL
   }

   private static final boolean DEBUG = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoDouble yoTime;
   private final ReferenceFrame midZupFrame;

   private final ArrayList<AbstractBehavior> behaviors = new ArrayList<AbstractBehavior>();

   private final SphereDetectionBehavior sphereDetectionBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final KickBehavior kickBehavior;
   private WaitForUserValidationBehavior waitForUserValidationBehavior;

   private BehaviorStateMachine<KickState> stateMachine;

   private final PipeLine<AbstractBehavior> pipeLine;

   private final double initalWalkDistance = 1.0;
   private final double standingDistance = 0.4;
   private boolean pipelineSetUp = false;


   public KickBallBehavior(String robotName, ROS2Node ros2Node, YoDouble yoTime, YoBoolean yoDoubleSupport,
                           FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(robotName, ros2Node);
      pipeLine = new PipeLine<>(yoTime);
      this.yoTime = yoTime;
      midZupFrame = referenceFrames.getMidFeetZUpFrame();

      // create sub-behaviors:
      if (USE_BLOB_FILTERING)
      {
         BlobFilteredSphereDetectionBehavior sphereDetectionBehavior = new BlobFilteredSphereDetectionBehavior(robotName, ros2Node, referenceFrames, fullRobotModel); // new SphereDetectionBehavior(outgoingCommunicationBridge, referenceFrames);
//         sphereDetectionBehavior.addHSVRange(new HSVRange(new HSVValue(55, 80, 80), new HSVValue(139, 255, 255)));
         this.sphereDetectionBehavior = sphereDetectionBehavior;
      }
      else
      {
         sphereDetectionBehavior = new SphereDetectionBehavior(robotName, ros2Node, referenceFrames);
      }

      behaviors.add(sphereDetectionBehavior);

      walkToLocationBehavior = new WalkToLocationBehavior(robotName, ros2Node, fullRobotModel,
                                                          referenceFrames, wholeBodyControllerParameters.getWalkingControllerParameters());
      behaviors.add(walkToLocationBehavior);

      kickBehavior = new KickBehavior(robotName, ros2Node, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames);
      behaviors.add(kickBehavior);

      for (AbstractBehavior behavior : behaviors)
      {
         registry.addChild(behavior.getYoRegistry());
      }
   }

  
   boolean locationSet = false;

   @Override
   public void doControl()
   {
      pipeLine.doControl();
   }

   private void setupPipelineForKick()
   {

      BehaviorAction findBallTask = new BehaviorAction(sphereDetectionBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            /*if (CREATE_COACTIVE_ELEMENT)
            {
               coactiveElement.searchingForBall.set(true);
               coactiveElement.foundBall.set(false);
               coactiveElement.ballPositions.get(0).setToZero();
            }*/
         }
      };

      pipeLine.submitSingleTaskStage(findBallTask);

      BehaviorAction validateBallTask = new BehaviorAction(waitForUserValidationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
           /* if (CREATE_COACTIVE_ELEMENT)
            {
               coactiveElement.searchingForBall.set(false);
               coactiveElement.waitingForValidation.set(true);
               coactiveElement.validAcknowledged.set(false);
               coactiveElement.foundBall.set(false);
               coactiveElement.ballPositions.get(0).set(sphereDetectionBehavior.getBallLocation());
               coactiveElement.ballRadii[0].set(sphereDetectionBehavior.getSpehereRadius());
            }*/
         }
      };

      pipeLine.submitSingleTaskStage(validateBallTask);

      BehaviorAction walkToBallTask = new BehaviorAction(walkToLocationBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            /*
            if (CREATE_COACTIVE_ELEMENT)
            {
               coactiveElement.searchingForBall.set(false);
               coactiveElement.waitingForValidation.set(false);
               coactiveElement.foundBall.set(true);
               coactiveElement.ballPositions.get(0).set(sphereDetectionBehavior.getBallLocation());
               coactiveElement.ballRadii[0].set(sphereDetectionBehavior.getSpehereRadius());
            }*/
            walkToLocationBehavior.setTarget(getoffsetPoint());
         }
      };

      pipeLine.submitSingleTaskStage(walkToBallTask);

      BehaviorAction kickBallTask = new BehaviorAction(kickBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint2D ballToKickLocation = new FramePoint2D(getoffsetPoint().getPosition());
            kickBehavior.setObjectToKickPoint(ballToKickLocation);
         }
      };

      pipeLine.submitSingleTaskStage(kickBallTask);

      pipeLine.requestNewStage();
      pipelineSetUp = true;

   }

   private FramePose2D getoffsetPoint()
   {
      FramePoint2D ballPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(), sphereDetectionBehavior.getBallLocation().getX(),
                                                     sphereDetectionBehavior.getBallLocation().getY());
      FramePoint2D robotPosition = new FramePoint2D(midZupFrame, 0.0, 0.0);
      robotPosition.changeFrame(worldFrame);
      FrameVector2D walkingDirection = new FrameVector2D(worldFrame);
      walkingDirection.set(ballPosition2d);
      walkingDirection.sub(robotPosition);
      walkingDirection.normalize();
      double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());
      double x = ballPosition2d.getX() - walkingDirection.getX() * standingDistance;
      double y = ballPosition2d.getY() - walkingDirection.getY() * standingDistance;
      FramePose2D poseToWalkTo = new FramePose2D(worldFrame, new Point2D(x, y), walkingYaw);
      return poseToWalkTo;
   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipelineForKick();
   }

   @Override
   public void onBehaviorExited()
   {

      pipelineSetUp = false;

         /*coactiveElement.searchingForBall.set(false);
         coactiveElement.waitingForValidation.set(false);
         coactiveElement.foundBall.set(false);
         coactiveElement.ballPositions.get(0).setToZero();*/
      
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorExited();
      }
   }

   @Override
   public void onBehaviorAborted()
   {
      onBehaviorExited();
      this.pipeLine.clearAll();

      for (AbstractBehavior behavior : behaviors)
      {
         behavior.onBehaviorAborted();
      }

   }

   @Override
   public void onBehaviorPaused()
   {
      for (AbstractBehavior behavior : behaviors)
      {
         behavior.pause();
      }
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   public boolean isUseBlobFiltering()
   {
      return USE_BLOB_FILTERING;
   }

   public Point2D getBlobLocation()
   {
      if (USE_BLOB_FILTERING)
      {
         return ((BlobFilteredSphereDetectionBehavior) sphereDetectionBehavior).getLatestBallPosition();
      }
      else
      {
         return null;
      }
   }

   public int getNumBlobsDetected()
   {
      if (USE_BLOB_FILTERING)
      {
         return ((BlobFilteredSphereDetectionBehavior) sphereDetectionBehavior).getNumBallsDetected();
      }
      else
      {
         return 0;
      }
   }

   @Override
   public void onBehaviorResumed()
   {
   }
}
