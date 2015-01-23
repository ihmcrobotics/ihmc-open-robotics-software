package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.TaskExecutor;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspPieceOfDebrisBehavior extends BehaviorInterface
{
   private final TaskExecutor taskExecutor = new TaskExecutor();

   private final HandPoseBehavior handPoseBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   private final BooleanYoVariable haveInputsBeenSet;
   private final DoubleYoVariable offsetToThePointOfGrabbing;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame pelvisFrame;

   private RobotSide robotSide;
   private double trajectoryTime = 2.5;

   private final FullRobotModel fullRobotModel;

   private static final double WRIST_OFFSET = 0.14;

   private DoubleYoVariable yoTime;

   private Quat4d rotationToBePerformedInWorldFrame = new Quat4d();

   public GraspPieceOfDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.fullRobotModel = fullRobotModel;
      this.yoTime = yoTime;

      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);

      pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);

      offsetToThePointOfGrabbing = new DoubleYoVariable("offsetToThePointOfGrabbing", registry);
      offsetToThePointOfGrabbing.set(0.3);
   }

   public void setGraspPose(RigidBodyTransform debrisTransform, Point3d graspPosition, Vector3d graspVector)
   {
      if (graspVector.length() == 0.0)
      {
         throw new RuntimeException("graspVector has not been set!");
      }
      robotSide = determineSideToUse(graspPosition);
      setTasks(debrisTransform, graspPosition, graspVector);
      haveInputsBeenSet.set(true);
   }

   private void setTasks(RigidBodyTransform debrisTransform, Point3d graspPosition, Vector3d graspVector)
   {
      RigidBodyTransform tempPose = new RigidBodyTransform();
      FramePose desiredGrabPose = new FramePose();
      FramePose midGrabPose = new FramePose();
      
      computeDesiredGraspOrientation(debrisTransform, fullRobotModel.getHandControlFrame(robotSide), rotationToBePerformedInWorldFrame, graspVector);
      computeDesiredPoses(midGrabPose,rotationToBePerformedInWorldFrame, graspPosition, graspVector, offsetToThePointOfGrabbing.getDoubleValue());
      computeDesiredPoses(desiredGrabPose,rotationToBePerformedInWorldFrame, graspPosition, graspVector, WRIST_OFFSET);
      
      midGrabPose.getPose(tempPose);
      taskExecutor.submit(new HandPoseTask(robotSide, yoTime, handPoseBehavior, Frame.WORLD, tempPose, trajectoryTime));

      taskExecutor.submit(new FingerStateTask(robotSide, FingerState.OPEN, fingerStateBehavior));

      desiredGrabPose.getPose(tempPose);
      taskExecutor.submit(new HandPoseTask(robotSide, yoTime, handPoseBehavior, Frame.WORLD, tempPose, trajectoryTime));

      taskExecutor.submit(new FingerStateTask(robotSide, FingerState.CLOSE, fingerStateBehavior));
   }

   private void computeDesiredPoses(FramePose desiredPoseToPack, Quat4d rotationToBePerformedInWorldFrame, Point3d graspPosition, Vector3d graspVector,
         double wristOffset)
   {
      Vector3d translation = new Vector3d(graspPosition);
      Vector3d tempGraspVector = new Vector3d(graspVector);
      
      desiredPoseToPack.setToZero(fullRobotModel.getHandControlFrame(robotSide));
      desiredPoseToPack.changeFrame(worldFrame);

      desiredPoseToPack.setOrientation(rotationToBePerformedInWorldFrame);

      tempGraspVector.normalize();
      tempGraspVector.scale(wristOffset);
      translation.add(tempGraspVector);
      
      desiredPoseToPack.setPosition(translation);
   }

   private void computeDesiredGraspOrientation(RigidBodyTransform debrisTransform, ReferenceFrame handFrame, Quat4d desiredGraspOrientationToPack,
         Vector3d graspVector)
   {
      PoseReferenceFrame handFrameBeforeRotation = new PoseReferenceFrame("handFrameBeforeRotation", worldFrame);
      handFrameBeforeRotation.setPoseAndUpdate(debrisTransform);

      FramePose handPoseSolution1 = new FramePose(handFrameBeforeRotation);
      handPoseSolution1.changeFrame(handFrame);

      FramePose handPoseSolution2 = new FramePose(handFrameBeforeRotation);
      handPoseSolution2.setOrientation(0.0, 0.0, Math.PI);
      handPoseSolution2.changeFrame(handFrame);

      double rollOfSolution1 = handPoseSolution1.getRoll();
      double rollOfSolution2 = handPoseSolution2.getRoll();

      FramePose handPose = new FramePose(handFrameBeforeRotation);
      if (Math.abs(rollOfSolution1) <= Math.abs(rollOfSolution2))
      {
         handPose.setPoseIncludingFrame(handPoseSolution1);
      }
      else
      {
         handPose.setPoseIncludingFrame(handPoseSolution2);
      }

      handPose.changeFrame(worldFrame);
      handPose.getOrientation(desiredGraspOrientationToPack);
   }

   private RobotSide determineSideToUse(Point3d position)
   {
      FramePose pose = new FramePose(worldFrame);
      pose.setPose(position, new Quat4d(0, 0, 0, 1));
      pose.changeFrame(pelvisFrame);
      if (pose.getY() <= 0.0)
         return RobotSide.RIGHT;
      else
         return RobotSide.LEFT;
   }

   @Override
   public void doControl()
   {
      taskExecutor.doControl();
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
   }

   @Override
   public void resume()
   {
      handPoseBehavior.resume();
   }

   @Override
   public boolean isDone()
   {
      return taskExecutor.isDone();
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
