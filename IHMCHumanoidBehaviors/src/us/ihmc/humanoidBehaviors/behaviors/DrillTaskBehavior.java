package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.behaviors.DrillTaskPacket;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.CoMHeightTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.utilities.humanoidRobot.frames.ReferenceFrames;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FramePose2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class DrillTaskBehavior extends BehaviorInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final DoubleYoVariable yoTime;
   private final ConcurrentListeningQueue<DrillTaskPacket> drillTaskBehaviorInputPacketListener;
   private final ReferenceFrame midZupFrame;
   private BooleanYoVariable hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);
   
   private FramePose drillPose;
   
   private final WalkToLocationBehavior walkToLocationBehavior;
   private final FramePose2d pickUpPoseToWalkTo;
   
   private final ArrayList<BehaviorInterface> behaviors = new ArrayList<BehaviorInterface>();
   private final FingerStateBehavior fingerStateBehavior;
   private final ComHeightBehavior comHeightBehavior;
   private final HandPoseBehavior handPoseBehavior;
   
   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();
   
   public RobotSide grabSide = RobotSide.RIGHT;
   private final double standingDistance = 0.7;
   private final double comHeightTrajectoryTime = 1.0;
   private final double handPoseTrajectoryTime = 1.0;
   public final double wristHandDistance = 0.17;
   public final double drillOffsetInZ = 0.15;
   
   public DrillTaskBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime,
         SDFFullRobotModel fullRobotModel, ReferenceFrames referenceFrames, WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(outgoingCommunicationBridge);
      drillTaskBehaviorInputPacketListener = new ConcurrentListeningQueue<DrillTaskPacket>();
      this.yoTime = yoTime;
      
      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames,
            wholeBodyControllerParameters.getWalkingControllerParameters());
      registry.addChild(walkToLocationBehavior.getYoVariableRegistry());
      behaviors.add(walkToLocationBehavior);
      
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);
      registry.addChild(fingerStateBehavior.getYoVariableRegistry());
      behaviors.add(fingerStateBehavior);
      
      comHeightBehavior = new ComHeightBehavior(outgoingCommunicationBridge, yoTime);
      registry.addChild(comHeightBehavior.getYoVariableRegistry());
      behaviors.add(comHeightBehavior);
      
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      registry.addChild(handPoseBehavior.getYoVariableRegistry());
      behaviors.add(handPoseBehavior);
      
      midZupFrame = referenceFrames.getMidFeetZUpFrame();
      pickUpPoseToWalkTo = new FramePose2d(worldFrame);
      
      super.attachNetworkProcessorListeningQueue(drillTaskBehaviorInputPacketListener, DrillTaskPacket.class);
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
      if (drillTaskBehaviorInputPacketListener.isNewPacketAvailable())
      {
         RigidBodyTransform drillTransform = drillTaskBehaviorInputPacketListener.getNewestPacket().getDrillTransform();
         drillTaskBehaviorInputPacketListener.clear();
         
         if(drillTransform != null)
         {
            drillPose = new FramePose(worldFrame, drillTransform);
            hasInputBeenSet.set(true);
            setupPipeline();
         }
      }
   }
   
   private void setupPipeline()
   {
      pipeLine.requestNewStage();
      
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
      pickUpPoseToWalkTo.setPoseIncludingFrame(worldFrame, x, y, walkingYaw);
      
      pipeLine.submitSingleTaskStage(new WalkToLocationTask(pickUpPoseToWalkTo, walkToLocationBehavior, 0.0, 0.3, yoTime));
      pipeLine.requestNewStage();      
      
      // stage: move center of mass and open hand
      FramePoint drillPosition = drillPose.getFramePointCopy();
      drillPosition.add(0.0, 0.0, drillOffsetInZ);
      double desiredCom = drillPosition.getZ() - 1.15;
      
      pipeLine.submitTaskForPallelPipesStage(comHeightBehavior, new CoMHeightTask(desiredCom, yoTime, comHeightBehavior, comHeightTrajectoryTime));
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, new FingerStateTask(grabSide, FingerState.OPEN, fingerStateBehavior, yoTime));
      pipeLine.requestNewStage();
      
      // stage: bring hand in position close to drill
      double approachYaw = walkingYaw + (grabSide == RobotSide.RIGHT ? 1.0 : -1.0) * Math.PI / 4;
      FrameVector approachDirection = new FrameVector(worldFrame, Math.cos(approachYaw), Math.sin(approachYaw), 0.0);
      
      FramePoint handPosition = offsetPosition(drillPosition, approachDirection, 0.3 + wristHandDistance);
      FramePose handPose = new FramePose(worldFrame);
      handPose.setPosition(handPosition);
      
      FrameOrientation handOrientation = new FrameOrientation(worldFrame);
      handOrientation.setYawPitchRoll(approachYaw, 0.0, - (grabSide == RobotSide.RIGHT ? 1.0 : -1.0) * Math.PI/2);
      handPose.setOrientation(handOrientation);
      
      RigidBodyTransform initialGrabTransform = new RigidBodyTransform();
      handPose.getPose(initialGrabTransform);
      HandPosePacket handPosePacket = PacketControllerTools.createHandPosePacket(Frame.WORLD, initialGrabTransform, grabSide, handPoseTrajectoryTime);
      
      pipeLine.submitSingleTaskStage(new HandPoseTask(grabSide, handPosePacket, handPoseBehavior, yoTime));
      pipeLine.requestNewStage();
      
      // stage: move hand closer to drill
      handPosition = offsetPosition(drillPosition, approachDirection, wristHandDistance);
      handPose.setPosition(handPosition);
      
      RigidBodyTransform handTransform = new RigidBodyTransform();
      handPose.getPose(handTransform);
      handPosePacket = PacketControllerTools.createHandPosePacket(Frame.WORLD, handTransform, grabSide, handPoseTrajectoryTime);
      
      pipeLine.submitSingleTaskStage(new HandPoseTask(grabSide, handPosePacket, handPoseBehavior, yoTime));
      pipeLine.requestNewStage();
      
      // stage: close hand
      pipeLine.submitSingleTaskStage(new FingerStateTask(grabSide, FingerState.CLOSE, fingerStateBehavior, yoTime));
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
   public void finalize()
   {
      defaultFinalize();
      for (BehaviorInterface behavior : behaviors)
      {
         behavior.finalize();
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
