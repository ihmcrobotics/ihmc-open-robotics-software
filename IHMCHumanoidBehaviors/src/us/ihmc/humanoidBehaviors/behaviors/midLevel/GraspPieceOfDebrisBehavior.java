package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
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
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.TaskExecutor;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspPieceOfDebrisBehavior extends BehaviorInterface
{

   private final TaskExecutor taskExecutor = new TaskExecutor();

   private final HandPoseBehavior handPoseBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   //   private final BooleanYoVariable isDone;
   private final BooleanYoVariable haveInputsBeenSet;
   private final BooleanYoVariable reachedMidPoint;
   private final DoubleYoVariable offsetToThePointOfGrabbing;

   private final ReferenceFrame pelvisFrame;

   private RobotSide robotSide = RobotSide.LEFT;
   private double trajectoryTime = 2.5;//2.0;

   private final FramePose desiredGrabPose = new FramePose();
   private final FramePose midGrabPose = new FramePose();
   private FullRobotModel fullRobotModel;

   private static final double WRIST_OFFSET = 0.14; // 0.11 , wrist to hand 0.11

   private DoubleYoVariable yoTime;

   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private Quat4d rotationToBePerformedInWorldFrame = new Quat4d();

   private static final double MAXIMUM_Z_BEFORE_KNEEL = 0.41;

   public GraspPieceOfDebrisBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      this.fullRobotModel = fullRobotModel;

      this.yoTime = yoTime;
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);

      pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);
      reachedMidPoint = new BooleanYoVariable("reachedMidPoint", registry);

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

      computeDesiredGraspOrientation(debrisTransform, fullRobotModel.getHandControlFrame(robotSide), rotationToBePerformedInWorldFrame, graspVector);

      getMidPose(rotationToBePerformedInWorldFrame, graspPosition, graspVector);
      midGrabPose.getPose(tempPose);
      taskExecutor.submit(new HandPoseTask(robotSide, yoTime, handPoseBehavior, Frame.WORLD, tempPose, trajectoryTime));

      taskExecutor.submit(new FingerStateTask(robotSide, FingerState.OPEN, fingerStateBehavior));

      getGraspLocation(rotationToBePerformedInWorldFrame, graspPosition, graspVector);
      desiredGrabPose.getPose(tempPose);
      taskExecutor.submit(new HandPoseTask(robotSide, yoTime, handPoseBehavior, Frame.WORLD, tempPose, trajectoryTime));

      taskExecutor.submit(new FingerStateTask(robotSide, FingerState.CLOSE, fingerStateBehavior));

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

   private void getGraspLocation(Quat4d rotationToBePerformed, Point3d graspPosition, Vector3d graspVector)
   {
      Vector3d translation = new Vector3d(graspPosition);
      Vector3d tempGraspVector = new Vector3d(graspVector);
      desiredGrabPose.setToZero(fullRobotModel.getHandControlFrame(robotSide));
      desiredGrabPose.changeFrame(ReferenceFrame.getWorldFrame());

      desiredGrabPose.setOrientation(rotationToBePerformed);

      
      tempGraspVector.normalize();
      tempGraspVector.scale(WRIST_OFFSET);
      translation.add(tempGraspVector);
      
      desiredGrabPose.setPosition(translation);
   }

   private void getMidPose(Quat4d rotationToBePerformed, Point3d graspPosition, Vector3d graspVector)
   {
      Vector3d translation = new Vector3d(graspPosition);
      Vector3d tempGraspVector = new Vector3d(graspVector);
      midGrabPose.setToZero(fullRobotModel.getHandControlFrame(robotSide));
      midGrabPose.changeFrame(ReferenceFrame.getWorldFrame());

      midGrabPose.setOrientation(rotationToBePerformed);
      
      tempGraspVector.normalize();
      tempGraspVector.scale(offsetToThePointOfGrabbing.getDoubleValue());
      translation.add(tempGraspVector);
      midGrabPose.setPosition(translation);
   }

   private RobotSide determineSideToUse(Point3d position)
   {
      FramePose pose = new FramePose(ReferenceFrame.getWorldFrame());
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
      reachedMidPoint.set(false);

   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }
}
