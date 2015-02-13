package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.TransformTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.TaskExecutor;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspValveBehavior extends BehaviorInterface
{
   private final double MIDPOSE_OFFSET_FROM_FINALPOSE = 0.3;
   private final double WRIST_OFFSET_FROM_HAND = 0.1; // 0.05
   private final double HAND_POSE_TRAJECTORY_TIME = 1.0;//2.0;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame pelvisFrame;
   private final FullRobotModel fullRobotModel;

   private final TaskExecutor taskExecutor = new TaskExecutor();
   private final HandPoseBehavior handPoseBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   private final BooleanYoVariable haveInputsBeenSet;
   private final BooleanYoVariable reachedMidPoint;
   private final DoubleYoVariable midPoseOffsetFromFinalPose;
   private final DoubleYoVariable yoTime;

   private RobotSide robotSideOfGraspingHand = null;

   public GraspValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime)
   {
      this(outgoingCommunicationBridge, new HandPoseBehavior(outgoingCommunicationBridge, yoTime), fullRobotModel, yoTime);
   }
   
   public GraspValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, HandPoseBehavior handPoseBehavior, FullRobotModel fullRobotModel, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      this.fullRobotModel = fullRobotModel;

      this.yoTime = yoTime;
      this.handPoseBehavior = handPoseBehavior;
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);

      pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);
      reachedMidPoint = new BooleanYoVariable("reachedMidPoint", registry);

      midPoseOffsetFromFinalPose = new DoubleYoVariable("offsetToThePointOfGrabbing", registry);
   }

   public void setGraspPose(ValveType valveType, RigidBodyTransform valveTransformToWorld, Vector3d approachDirectionInValveFrame, boolean graspValveRim)
   {
      Vector3d worldToValveVec = new Vector3d();
      valveTransformToWorld.getTranslation(worldToValveVec);

      Point3d finalGraspPosInWorld = new Point3d(worldToValveVec);

      if (graspValveRim)
      {
         double valveRadius = valveType.getValveRadius();
         Vector3d valveCenterToRimVecInValveFrame = new Vector3d(0, -valveRadius, 0);
         Vector3d valveCenterToRimVecInWorldFrame = TransformTools.getTransformedVector(valveCenterToRimVecInValveFrame, valveTransformToWorld);

         finalGraspPosInWorld.add(valveCenterToRimVecInWorldFrame);
      }

      Vector3d approachDirectionInWorld = new Vector3d(approachDirectionInValveFrame);
      valveTransformToWorld.transform(approachDirectionInWorld);

      setGraspPose(valveType, valveTransformToWorld, finalGraspPosInWorld, approachDirectionInWorld);
   }

   public void setGraspPose(ValveType valveType, RigidBodyTransform valveTransformToWorld, Point3d finalGraspPosInWorld, Vector3d approachDirectionInWorld)
   {
      setGraspPose(valveType, valveTransformToWorld, finalGraspPosInWorld, approachDirectionInWorld, MIDPOSE_OFFSET_FROM_FINALPOSE);
   }

   public void setGraspPose(ValveType valveType, RigidBodyTransform valveTransformToWorld, Point3d finalGraspPosInWorld, Vector3d approachDirectionInWorld,
         double midPoseOffsetFromFinalPose)
   {
      this.midPoseOffsetFromFinalPose.set(midPoseOffsetFromFinalPose);

      if (approachDirectionInWorld.length() == 0.0)
      {
         throw new RuntimeException("finalToMidGraspVec has not been set!");
      }
      robotSideOfGraspingHand = determineSideToUse(finalGraspPosInWorld);

      setTasks(valveTransformToWorld, finalGraspPosInWorld, approachDirectionInWorld);
   }

   public RobotSide getRobotSideOfGraspingHand()
   {
      if (!haveInputsBeenSet.getBooleanValue())
      {
         throw new RuntimeException("Must set behavior inputs first!");
      }
      return robotSideOfGraspingHand;
   }

   public void getFinalGraspPose(FramePose poseToPack)
   {
      if (!haveInputsBeenSet.getBooleanValue())
      {
         throw new RuntimeException("Must set behavior inputs first!");
      }
      poseToPack.setPose(finalGrabPose);
   }

   private final FramePose finalGrabPose = new FramePose();
   private final Quat4d desiredGraspOrientation = new Quat4d();

   private void setTasks(RigidBodyTransform valveTransformToWorld, Point3d pointOnValveToGraspInWorld, Vector3d approachDirectionInWorld)
   {
      computeDesiredGraspOrientation(valveTransformToWorld, approachDirectionInWorld, fullRobotModel.getHandControlFrame(robotSideOfGraspingHand),
            desiredGraspOrientation);

      FramePose midGrabPose = new FramePose(worldFrame, getOffsetPoint3dCopy(pointOnValveToGraspInWorld, approachDirectionInWorld,
            -midPoseOffsetFromFinalPose.getDoubleValue()), desiredGraspOrientation);
      RigidBodyTransform midGrabTransform = new RigidBodyTransform();
      midGrabPose.getPose(midGrabTransform);

      finalGrabPose.setPoseIncludingFrame(worldFrame, getOffsetPoint3dCopy(pointOnValveToGraspInWorld, approachDirectionInWorld, -WRIST_OFFSET_FROM_HAND),
            desiredGraspOrientation);
      RigidBodyTransform finalGraspTransform = new RigidBodyTransform();
      finalGrabPose.getPose(finalGraspTransform);

      taskExecutor.submit(new HandPoseTask(robotSideOfGraspingHand, yoTime, handPoseBehavior, Frame.WORLD, midGrabTransform, HAND_POSE_TRAJECTORY_TIME));
      taskExecutor.submit(new FingerStateTask(robotSideOfGraspingHand, FingerState.OPEN, fingerStateBehavior, yoTime));
      taskExecutor.submit(new HandPoseTask(robotSideOfGraspingHand, yoTime, handPoseBehavior, Frame.WORLD, finalGraspTransform, HAND_POSE_TRAJECTORY_TIME));
      taskExecutor.submit(new FingerStateTask(robotSideOfGraspingHand, FingerState.CLOSE, fingerStateBehavior, yoTime));

      haveInputsBeenSet.set(true);
   }

   private Vector3d tempVec = new Vector3d();

   private Point3d getOffsetPoint3dCopy(Point3d initialPosition, Vector3d offsetDirection, double offsetDistance)
   {
      Point3d ret = new Point3d(initialPosition);

      tempVec.set(offsetDirection);
      tempVec.normalize();
      tempVec.scale(offsetDistance);

      ret.add(tempVec);

      return ret;
   }

   private void computeDesiredGraspOrientation(RigidBodyTransform valveTransformToWorld, Vector3d graspVector, ReferenceFrame handFrameBeforeGrasping,
         Quat4d desiredGraspOrientationToPack)
   {
      PoseReferenceFrame handFrameBeforeRotation = new PoseReferenceFrame("handFrameBeforeRotation", worldFrame);
      handFrameBeforeRotation.setPoseAndUpdate(valveTransformToWorld);

      FramePose handPose = new FramePose(handFrameBeforeRotation);
      handPose.setOrientation(0.0, 0.0, Math.PI);
      handPose.changeFrame(handFrameBeforeGrasping);

      handPose.changeFrame(worldFrame);
      handPose.getOrientation(desiredGraspOrientationToPack);
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
      return robotSideOfGraspingHand;
   }

   @Override
   public void stop()
   {
      handPoseBehavior.stop();
      fingerStateBehavior.stop();
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
   }

   @Override
   public void resume()
   {
      handPoseBehavior.resume();
      fingerStateBehavior.resume();
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
