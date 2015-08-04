package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.behaviors.DrillPacket;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ObjectWeightBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyIKTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.ObjectWeightTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WholeBodyIKTrajectoryTask;
import us.ihmc.robotics.humanoidRobot.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class DrillPickUpBehavior extends BehaviorInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final DoubleYoVariable yoTime;
   private final ConcurrentListeningQueue<DrillPacket> drillPickUpBehaviorInputPacketListener;
   private final ReferenceFrame midZupFrame;
   private BooleanYoVariable hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);

   private FramePose drillPose;

   private final ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final FingerStateBehavior fingerStateBehavior;
   private final ObjectWeightBehavior objectWeightBehavior;
   
   private final WholeBodyIKTrajectoryBehavior wholeBodyIKTrajectoryBehavior;

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private RobotSide grabSide = RobotSide.RIGHT;
   private final double standingDistance = 0.7;
   private final double drillHeight = 0.3;
   private final double drillWeight = 0.75;

   public DrillPickUpBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime, SDFFullRobotModel fullRobotModel,
                            HumanoidReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(outgoingCommunicationBridge);
      this.yoTime = yoTime;
      midZupFrame = referenceFrames.getMidFeetZUpFrame();

      // create sub-behaviors:
      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames,
              wholeBodyControllerParameters.getWalkingControllerParameters());
      behaviors.add(walkToLocationBehavior);

      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
      behaviors.add(fingerStateBehavior);
      
      objectWeightBehavior = new ObjectWeightBehavior(outgoingCommunicationBridge);
      behaviors.add(objectWeightBehavior);
      
      wholeBodyIKTrajectoryBehavior = new WholeBodyIKTrajectoryBehavior(outgoingCommunicationBridge, wholeBodyControllerParameters, fullRobotModel, yoTime);
      behaviors.add(wholeBodyIKTrajectoryBehavior);

      for (BehaviorInterface behavior : behaviors)
      {
         registry.addChild(behavior.getYoVariableRegistry());
      }

      drillPickUpBehaviorInputPacketListener = new ConcurrentListeningQueue<DrillPacket>();
      super.attachNetworkProcessorListeningQueue(drillPickUpBehaviorInputPacketListener, DrillPacket.class);
   }

   @Override
   public void doControl()
   {
      if (!hasInputBeenSet.getBooleanValue())
      {
         checkInput();
      }
      else
      {
         pipeLine.doControl();
      }
   }

   private void checkInput()
   {
      if (drillPickUpBehaviorInputPacketListener.isNewPacketAvailable())
      {
         RigidBodyTransform drillTransform = drillPickUpBehaviorInputPacketListener.getNewestPacket().getDrillTransform();
         drillPickUpBehaviorInputPacketListener.clear();

         if (drillTransform != null)
         {
            // move the drill transform from the top of the drill to the middle since we want to grab there:
            Vector3d drillPositionOffset = new Vector3d(0.0, 0.0, - drillHeight / 2.0);
            Vector3d drillPosition = new Vector3d();
            drillTransform.getTranslation(drillPosition);
            drillPosition.add(drillPositionOffset);
            drillTransform.setTranslation(drillPosition);
            
            drillPose = new FramePose(worldFrame, drillTransform);
            hasInputBeenSet.set(true);
            setupPipeline();
         }
      }
   }

   private void setupPipeline()
   {
      // stage: position robot on front of drill
      FramePose2d drillPose2d = new FramePose2d(worldFrame);
      drillPose.getPose2dIncludingFrame(drillPose2d);
      FramePoint2d drillPosition2d = new FramePoint2d();
      drillPose2d.getPosition(drillPosition2d);
      FramePoint2d robotPosition = new FramePoint2d(midZupFrame, 0.0, 0.0);
      robotPosition.changeFrame(worldFrame);
      FrameVector2d walkingDirection = new FrameVector2d(worldFrame);
      walkingDirection.set(drillPosition2d);
      walkingDirection.sub(robotPosition);
      walkingDirection.normalize();
      double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());
      double x = drillPosition2d.getX() - walkingDirection.getX() * standingDistance;
      double y = drillPosition2d.getY() - walkingDirection.getY() * standingDistance;
      FramePose2d poseToWalkTo = new FramePose2d(worldFrame, new Point2d(x, y), walkingYaw);

      pipeLine.submitSingleTaskStage(new WalkToLocationTask(poseToWalkTo, walkToLocationBehavior, 0.0, 0.3, yoTime));
      pipeLine.requestNewStage();

      // stage: bring hand in position close to drill
      double approachYaw = walkingYaw + ((grabSide == RobotSide.RIGHT) ? 1.0 : -1.0) * Math.PI / 4;
      FrameVector approachDirection = new FrameVector(worldFrame, Math.cos(approachYaw), Math.sin(approachYaw), 0.0);

      FramePoint drillPosition = drillPose.getFramePointCopy();
      FramePoint handPosition = offsetPosition(drillPosition, approachDirection, 0.3);
      FrameOrientation handOrientation = new FrameOrientation(worldFrame);
      handOrientation.setYawPitchRoll(approachYaw, 0.0, -((grabSide == RobotSide.RIGHT) ? 1.0 : -1.0) * Math.PI / 2);

      FramePose handPose = new FramePose(worldFrame);
      handPose.setPosition(handPosition);
      handPose.setOrientation(handOrientation);

      pipeLine.submitTaskForPallelPipesStage(wholeBodyIKTrajectoryBehavior, new WholeBodyIKTrajectoryTask(wholeBodyIKTrajectoryBehavior, yoTime,
            grabSide, handPose, ControlledDoF.DOF_3P3R));
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(grabSide, FingerState.OPEN, fingerStateBehavior, yoTime));
      pipeLine.requestNewStage();

      // stage: move hand closer to drill
      handPose.setPosition(drillPosition);
      handPose.setOrientation(handOrientation);

      pipeLine.submitSingleTaskStage(new WholeBodyIKTrajectoryTask(wholeBodyIKTrajectoryBehavior, yoTime, grabSide, handPose, ControlledDoF.DOF_3P3R));
      pipeLine.requestNewStage();

      // stage: close hand
      pipeLine.submitSingleTaskStage(new FingerStateTask(grabSide, FingerState.CLOSE, fingerStateBehavior, yoTime));
      pipeLine.requestNewStage();
      
      // stage: tell controller about weight in hand
      pipeLine.submitSingleTaskStage(new ObjectWeightTask(grabSide, drillWeight, objectWeightBehavior, yoTime));
      pipeLine.requestNewStage();
      
      // stage: turn drill if necessary
      
      // stage: lift up drill
      handPose.setZ(handPose.getZ() + 0.2);
      pipeLine.submitSingleTaskStage(new WholeBodyIKTrajectoryTask(wholeBodyIKTrajectoryBehavior, yoTime, grabSide, handPose, ControlledDoF.DOF_3P3R));
      pipeLine.requestNewStage();
      
      // stage: walk back a little
      walkingDirection.normalize();
      x = drillPosition2d.getX() - walkingDirection.getX() * (standingDistance + 0.5);
      y = drillPosition2d.getY() - walkingDirection.getY() * (standingDistance + 0.5);
      poseToWalkTo.setPoseIncludingFrame(worldFrame, x, y, walkingYaw);
      
      pipeLine.submitSingleTaskStage(new WalkToLocationTask(poseToWalkTo, walkToLocationBehavior, Math.PI, 0.3, yoTime));
      pipeLine.requestNewStage();
   }

   private FramePoint offsetPosition(FramePoint position, FrameVector direction, double distance)
   {
      FramePoint ret = new FramePoint(position);
      direction.changeFrame(position.getReferenceFrame());
      direction.normalize();
      direction.scale(distance);
      ret.sub(direction);

      return ret;
   }

   public void setGrabSide(RobotSide side)
   {
      grabSide = side;
   }

   @Override
   public void initialize()
   {
      defaultInitialize();

      for (BehaviorInterface behavior : behaviors)
      {
         behavior.initialize();
      }
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      defaultPostBehaviorCleanup();

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
      return hasInputBeenSet() && defaultIsDone() && pipeLine.isDone();
   }

   @Override
   public void enableActions()
   {
   }

   @Override
   public void resume()
   {
      defaultResume();

      for (BehaviorInterface behavior : behaviors)
      {
         behavior.resume();
      }
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

   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
