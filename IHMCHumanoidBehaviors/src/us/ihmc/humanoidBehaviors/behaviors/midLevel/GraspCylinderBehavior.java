package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.CoMHeightTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GraspCylinderTask;
import us.ihmc.humanoidBehaviors.taskExecutor.OrientPalmToGraspCylinderTask;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspCylinderBehavior extends BehaviorInterface
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final SDFFullRobotModel fullRobotModel;

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();
   private final ComHeightBehavior comHeightBehavior;
   private final HandPoseBehavior handPoseBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   private final BooleanYoVariable haveInputsBeenSet;
   private final DoubleYoVariable yoTime;

   private RobotSide robotSide = null;

   private final boolean DEBUG = false;

   public GraspCylinderBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullRobotModel fullRobotModel,
         WholeBodyControllerParameters wholeBodyControllerParameters, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
      this.fullRobotModel = fullRobotModel;

      comHeightBehavior = new ComHeightBehavior(outgoingCommunicationBridge, yoTime);
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);
   }

   public void setGraspPose(RobotSide robotSide, FramePoint graspPoint, FrameVector graspedCylinderLongAxis, boolean stopHandIfCollision)
   {
      this.robotSide = robotSide;

      graspPoint.changeFrame(worldFrame);

      CoMHeightTask comHeightTask = new CoMHeightTask(graspPoint.getZ() - 0.9, yoTime, comHeightBehavior, 1.0);
      FingerStateTask openHandTask = new FingerStateTask(robotSide, FingerState.OPEN, fingerStateBehavior, yoTime);

      OrientPalmToGraspCylinderTask orientPalmForGraspingTask = new OrientPalmToGraspCylinderTask(robotSide, graspPoint, graspedCylinderLongAxis,
            fullRobotModel, yoTime, handPoseBehavior, 1.0);
      
      GraspCylinderTask movePalmToContactCylinder = new GraspCylinderTask(robotSide, graspPoint, graspedCylinderLongAxis,
            fullRobotModel, yoTime, handPoseBehavior, 1.0);
      
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
   public void finalize()
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
