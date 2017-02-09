package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import java.util.ArrayList;

import javax.vecmath.Point2d;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.KickBallBehaviorCoactiveElementBehaviorSide;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BlobFilteredSphereDetectionBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SphereDetectionBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.WaitForUserValidationBehavior;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateMachine;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ihmcPerception.vision.HSVValue;
import us.ihmc.ihmcPerception.vision.shapes.HSVRange;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

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
   private final DoubleYoVariable yoTime;
   private final ReferenceFrame midZupFrame;

   private final ArrayList<AbstractBehavior> behaviors = new ArrayList<AbstractBehavior>();

   private final SphereDetectionBehavior sphereDetectionBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final KickBehavior kickBehavior;
   private WaitForUserValidationBehavior waitForUserValidationBehavior;

   private BehaviorStateMachine<KickState> stateMachine;

   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<>();

   private final double initalWalkDistance = 1.0;
   private final double standingDistance = 0.4;
   private boolean pipelineSetUp = false;

   private final KickBallBehaviorCoactiveElementBehaviorSide coactiveElement;

   public KickBallBehavior(CommunicationBridge behaviorCommunicationBridge, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport,
         FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(behaviorCommunicationBridge);

      this.yoTime = yoTime;
      midZupFrame = referenceFrames.getMidFeetZUpFrame();

      // create sub-behaviors:
      if (USE_BLOB_FILTERING)
      {
         BlobFilteredSphereDetectionBehavior sphereDetectionBehavior = new BlobFilteredSphereDetectionBehavior(behaviorCommunicationBridge, referenceFrames,
               fullRobotModel); // new SphereDetectionBehavior(outgoingCommunicationBridge, referenceFrames);
         sphereDetectionBehavior.addHSVRange(new HSVRange(new HSVValue(55, 80, 80), new HSVValue(139, 255, 255)));
         this.sphereDetectionBehavior = sphereDetectionBehavior;
      }
      else
      {
         sphereDetectionBehavior = new SphereDetectionBehavior(communicationBridge, referenceFrames);
      }

      behaviors.add(sphereDetectionBehavior);

      walkToLocationBehavior = new WalkToLocationBehavior(communicationBridge, fullRobotModel, referenceFrames,
            wholeBodyControllerParameters.getWalkingControllerParameters());
      behaviors.add(walkToLocationBehavior);

      kickBehavior = new KickBehavior(communicationBridge, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames);
      behaviors.add(kickBehavior);

      for (AbstractBehavior behavior : behaviors)
      {
         registry.addChild(behavior.getYoVariableRegistry());
      }

      if (CREATE_COACTIVE_ELEMENT)
      {
         coactiveElement = new KickBallBehaviorCoactiveElementBehaviorSide();
         coactiveElement.setKickBallBehavior(this);
         registry.addChild(coactiveElement.getUserInterfaceWritableYoVariableRegistry());
         registry.addChild(coactiveElement.getMachineWritableYoVariableRegistry());

         waitForUserValidationBehavior = new WaitForUserValidationBehavior(communicationBridge, coactiveElement.validClicked,
               coactiveElement.validAcknowledged);
         behaviors.add(waitForUserValidationBehavior);

      }
      else
      {
         coactiveElement = null;
      }
   }

   @Override
   public CoactiveElement getCoactiveElement()
   {
      return null;//coactiveElement;
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
            if (CREATE_COACTIVE_ELEMENT)
            {
               coactiveElement.searchingForBall.set(true);
               coactiveElement.foundBall.set(false);
               coactiveElement.ballPositions.get(0).setToZero();
            }
         }
      };

      pipeLine.submitSingleTaskStage(findBallTask);

      BehaviorAction validateBallTask = new BehaviorAction(waitForUserValidationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            if (CREATE_COACTIVE_ELEMENT)
            {
               coactiveElement.searchingForBall.set(false);
               coactiveElement.waitingForValidation.set(true);
               coactiveElement.validAcknowledged.set(false);
               coactiveElement.foundBall.set(false);
               coactiveElement.ballPositions.get(0).set(sphereDetectionBehavior.getBallLocation());
               coactiveElement.ballRadii[0].set(sphereDetectionBehavior.getSpehereRadius());
            }
         }
      };

      pipeLine.submitSingleTaskStage(validateBallTask);

      BehaviorAction walkToBallTask = new BehaviorAction(walkToLocationBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            if (CREATE_COACTIVE_ELEMENT)
            {
               coactiveElement.searchingForBall.set(false);
               coactiveElement.waitingForValidation.set(false);
               coactiveElement.foundBall.set(true);
               coactiveElement.ballPositions.get(0).set(sphereDetectionBehavior.getBallLocation());
               coactiveElement.ballRadii[0].set(sphereDetectionBehavior.getSpehereRadius());
            }
            walkToLocationBehavior.setTarget(getoffsetPoint());
         }
      };

      pipeLine.submitSingleTaskStage(walkToBallTask);

      BehaviorAction kickBallTask = new BehaviorAction(kickBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            FramePoint2d ballToKickLocation = new FramePoint2d();
            getoffsetPoint().getPosition(ballToKickLocation);
            kickBehavior.setObjectToKickPoint(ballToKickLocation);
         }
      };

      pipeLine.submitSingleTaskStage(kickBallTask);

      pipeLine.requestNewStage();
      pipelineSetUp = true;

   }

   private FramePose2d getoffsetPoint()
   {
      FramePoint2d ballPosition2d = new FramePoint2d(ReferenceFrame.getWorldFrame(), sphereDetectionBehavior.getBallLocation().getX(),
            sphereDetectionBehavior.getBallLocation().getY());
      FramePoint2d robotPosition = new FramePoint2d(midZupFrame, 0.0, 0.0);
      robotPosition.changeFrame(worldFrame);
      FrameVector2d walkingDirection = new FrameVector2d(worldFrame);
      walkingDirection.set(ballPosition2d);
      walkingDirection.sub(robotPosition);
      walkingDirection.normalize();
      double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());
      double x = ballPosition2d.getX() - walkingDirection.getX() * standingDistance;
      double y = ballPosition2d.getY() - walkingDirection.getY() * standingDistance;
      FramePose2d poseToWalkTo = new FramePose2d(worldFrame, new Point2d(x, y), walkingYaw);
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
      if (CREATE_COACTIVE_ELEMENT)
      {
         coactiveElement.searchingForBall.set(false);
         coactiveElement.waitingForValidation.set(false);
         coactiveElement.foundBall.set(false);
         coactiveElement.ballPositions.get(0).setToZero();
      }
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

   public Point2d getBlobLocation()
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
