package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.communication.packets.behaviors.GraspCylinderPacket;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.CoMHeightTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GraspCylinderTask;
import us.ihmc.humanoidBehaviors.taskExecutor.OrientPalmToGraspCylinderTask;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspCylinderBehavior extends BehaviorInterface
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FullHumanoidRobotModel fullRobotModel;

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();
   private final ComHeightBehavior comHeightBehavior;
   private final HandPoseBehavior handPoseBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   private final BooleanYoVariable haveInputsBeenSet;
   private final DoubleYoVariable yoTime;

   private final ConcurrentListeningQueue<GraspCylinderPacket> graspCylinderPacketListener;

   private RobotSide robotSide = null;

   public GraspCylinderBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullHumanoidRobotModel fullRobotModel,
         DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
      this.fullRobotModel = fullRobotModel;

      comHeightBehavior = new ComHeightBehavior(outgoingCommunicationBridge, yoTime);
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);

      graspCylinderPacketListener = new ConcurrentListeningQueue<GraspCylinderPacket>();
      super.attachNetworkProcessorListeningQueue(graspCylinderPacketListener, GraspCylinderPacket.class);

   }

   public void setInput(GraspCylinderPacket graspCylinderPacket)
   {
      FramePoint graspPoint = new FramePoint(worldFrame, graspCylinderPacket.graspPointInWorld);
      FrameVector graspCylinderLongAxis = new FrameVector(worldFrame, graspCylinderPacket.getCylinderLongAxisInWorld());
      
      setGraspPose(graspCylinderPacket.getRobotSide(), graspPoint, graspCylinderLongAxis, false);
   }

   public void setGraspPose(RobotSide robotSide, FramePoint graspPoint, FrameVector graspedCylinderLongAxis, boolean stopHandIfCollision)
   {
      this.robotSide = robotSide;

      graspPoint.changeFrame(worldFrame);

      CoMHeightTask comHeightTask = new CoMHeightTask(graspPoint.getZ() - 0.9, yoTime, comHeightBehavior, 1.0);
      FingerStateTask openHandTask = new FingerStateTask(robotSide, FingerState.OPEN, fingerStateBehavior, yoTime);

      OrientPalmToGraspCylinderTask orientPalmForGraspingTask = new OrientPalmToGraspCylinderTask(robotSide, graspPoint, graspedCylinderLongAxis,
            fullRobotModel, yoTime, handPoseBehavior, 3.0);

      GraspCylinderTask movePalmToContactCylinder = new GraspCylinderTask(robotSide, graspPoint, graspedCylinderLongAxis, fullRobotModel, yoTime,
            handPoseBehavior, 1.0);

      FingerStateTask closeHandTask = new FingerStateTask(robotSide, FingerState.CLOSE, fingerStateBehavior, yoTime);

      pipeLine.clearAll();
      pipeLine.submitSingleTaskStage(comHeightTask);
      pipeLine.submitSingleTaskStage(orientPalmForGraspingTask);
      pipeLine.submitSingleTaskStage(openHandTask);
      pipeLine.submitSingleTaskStage(movePalmToContactCylinder);
      pipeLine.submitSingleTaskStage(closeHandTask);

      haveInputsBeenSet.set(true);
   }

   @Override
   public void doControl()
   {
      if (graspCylinderPacketListener.isNewPacketAvailable() && !haveInputsBeenSet.getBooleanValue())
      {
         setInput(graspCylinderPacketListener.getNewestPacket());
      }
      
      pipeLine.doControl();
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromNetworkProcessor(object);
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      handPoseBehavior.consumeObjectFromController(object);
   }

   public RobotSide getSideToUse()
   {
      return robotSide;
   }

   @Override
   public void stop()
   {
      handPoseBehavior.stop();
      fingerStateBehavior.stop();
      comHeightBehavior.stop();
   }

   @Override
   public void enableActions()
   {
      handPoseBehavior.enableActions();
   }

   @Override
   public void pause()
   {
      handPoseBehavior.pause();
      fingerStateBehavior.pause();
      comHeightBehavior.pause();
   }

   @Override
   public void resume()
   {
      handPoseBehavior.resume();
      fingerStateBehavior.resume();
      comHeightBehavior.resume();
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      haveInputsBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      haveInputsBeenSet.set(false);

   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }
}
