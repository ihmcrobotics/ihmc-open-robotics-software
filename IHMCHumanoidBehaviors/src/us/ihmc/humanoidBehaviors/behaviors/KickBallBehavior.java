package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import javax.vecmath.Point2d;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.stateMachine.BehaviorStateMachine;
import us.ihmc.humanoidBehaviors.taskExecutor.BehaviorTask;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class KickBallBehavior extends BehaviorInterface
{
   private enum KickState
   {
      SEARCH, APPROACH, VERIFY_LOCATION, FINAL_APPROACH, VERIFY_KICK_LOCATION, KICK_BALL
   }

   private static final boolean DEBUG = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final DoubleYoVariable yoTime;
   private final ReferenceFrame midZupFrame;

   private final ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();

   private final LocalizeBallBehavior localizeBallBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final KickBehavior kickBehavior;

   private BehaviorStateMachine<KickState> stateMachine;

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final double initalWalkDistance = 1.0;
   private final double standingDistance = 0.4;
   private boolean pipelineSetUp = false;

   private final KickBallBehaviorCoactiveElement coactiveElement = new KickBallBehaviorCoactiveElement();
   
   public KickBallBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport,
         SDFFullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(outgoingCommunicationBridge);
      this.yoTime = yoTime;
      midZupFrame = referenceFrames.getMidFeetZUpFrame();

      // create sub-behaviors:

      localizeBallBehavior = new LocalizeBallBehavior(outgoingCommunicationBridge, referenceFrames);
      behaviors.add(localizeBallBehavior);

      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames,
            wholeBodyControllerParameters.getWalkingControllerParameters());
      behaviors.add(walkToLocationBehavior);

      kickBehavior = new KickBehavior(outgoingCommunicationBridge, yoTime, yoDoubleSupport, fullRobotModel, referenceFrames);
      behaviors.add(kickBehavior);

      for (BehaviorInterface behavior : behaviors)
      {
         registry.addChild(behavior.getYoVariableRegistry());
      }

      registry.addChild(coactiveElement.getUserInterfaceWritableYoVariableRegistry());
      registry.addChild(coactiveElement.getMachineWritableYoVariableRegistry());
   }
   
   @Override
   public CoactiveElement getCoactiveElement()
   {
      return coactiveElement;
   }

   boolean locationSet = false;

   @Override
   public void doControl()
   {
      pipeLine.doControl();
   }

   private void setupPipelineForKick()
   {
	   
      BehaviorTask findBallTask = new BehaviorTask(localizeBallBehavior, yoTime, 0)
      {

         @Override
         protected void setBehaviorInput()
         {

         }
      };

      pipeLine.submitSingleTaskStage(findBallTask);
//
      BehaviorTask walkToBallTask = new BehaviorTask(walkToLocationBehavior, yoTime, 0)
      {

         @Override
         protected void setBehaviorInput()
         {
            walkToLocationBehavior.setTarget(getoffsetPoint());
         }
      };

      pipeLine.submitSingleTaskStage(walkToBallTask);

      BehaviorTask kickBallTask = new BehaviorTask(kickBehavior, yoTime, 0)
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
      
      System.out.println("BALL LOCATION IN GET OFFSET POINT METHOD "+localizeBallBehavior.getBallLocation());
      FramePoint2d ballPosition2d = new FramePoint2d(ReferenceFrame.getWorldFrame(), localizeBallBehavior.getBallLocation().x,
            localizeBallBehavior.getBallLocation().y);
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
   public void initialize()
   {
      defaultInitialize();
      setupPipelineForKick();


   }

   @Override
   public void doPostBehaviorCleanup()
   {
      

      defaultPostBehaviorCleanup();
      pipelineSetUp = false;
      for (BehaviorInterface behavior : behaviors)
      {
         behavior.doPostBehaviorCleanup();
      }
   }

   @Override
   public void stop()
   {
      defaultStop();
      for (BehaviorInterface behavior : behaviors)
      {
         behavior.stop();
      }
   }

   @Override
   public void pause()
   {
      defaultPause();
      for (BehaviorInterface behavior : behaviors)
      {
         behavior.pause();
      }
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   @Override
   public void enableActions()
   {
   }

   @Override
   public void resume()
   {
      defaultResume();
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      for (BehaviorInterface behavior : behaviors)
      {
         behavior.consumeObjectFromNetworkProcessor(object);
      }
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      for (BehaviorInterface behavior : behaviors)
      {
         behavior.consumeObjectFromController(object);
      }
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return true;
   }

   public void abort()
   {
      this.pipeLine.clearAll();
   }
}
