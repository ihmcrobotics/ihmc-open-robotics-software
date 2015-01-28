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
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.math.geometry.TransformTools;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.TaskExecutor;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspValveBehavior extends BehaviorInterface
{
   private final TaskExecutor taskExecutor = new TaskExecutor();

   private final HandPoseBehavior handPoseBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   //   private final BooleanYoVariable isDone;
   private final BooleanYoVariable haveInputsBeenSet;
   private final BooleanYoVariable reachedMidPoint;
   private final DoubleYoVariable midPoseOffsetFromFinalPose;

   private final ReferenceFrame pelvisFrame;

   private RobotSide robotSideOfGraspingHand = null;
   private double trajectoryTime = 1.0;//2.0;

   private FullRobotModel fullRobotModel;

   private static final double MIDPOSE_OFFSET_FROM_FINALPOSE = 0.3;
   private static final double WRIST_OFFSET_FROM_HAND = 0.05; // 0.14 //0.11 , wrist to hand 0.11

   private DoubleYoVariable yoTime;

   private ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double MAXIMUM_Z_BEFORE_KNEEL = 0.41;

   public GraspValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      this.fullRobotModel = fullRobotModel;

      this.yoTime = yoTime;
      handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
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

   public void setGraspPose(ValveType valveType, RigidBodyTransform valveTransformToWorld, Point3d finalGraspPosInWorld, Vector3d finalToMidGraspVec)
   {
      setGraspPose(valveType, valveTransformToWorld, finalGraspPosInWorld, finalToMidGraspVec, MIDPOSE_OFFSET_FROM_FINALPOSE);
   }

   public void setGraspPose(ValveType valveType, RigidBodyTransform valveTransformToWorld, Point3d finalGraspPosInWorld, Vector3d finalToMidGraspVec,
         double midPoseOffsetFromFinalPose)
   {
      this.midPoseOffsetFromFinalPose.set(midPoseOffsetFromFinalPose);

      if (finalToMidGraspVec.length() == 0.0)
      {
         throw new RuntimeException("finalToMidGraspVec has not been set!");
      }
      robotSideOfGraspingHand = determineSideToUse(finalGraspPosInWorld);

      setTasks(valveTransformToWorld, finalGraspPosInWorld, finalToMidGraspVec);
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

   private void setTasks(RigidBodyTransform valveTransformToWorld, Point3d finalGraspPosInWorld, Vector3d finalToMidGraspVec)
   {
      Quat4d graspOrientation = computeDesiredGraspOrientation(valveTransformToWorld, fullRobotModel.getHandControlFrame(robotSideOfGraspingHand),
            finalToMidGraspVec);

      FramePose midGrabPose = new FramePose();
      setHandPose(midGrabPose, graspOrientation, finalGraspPosInWorld, finalToMidGraspVec, -midPoseOffsetFromFinalPose.getDoubleValue());
      RigidBodyTransform midGrabTransform = new RigidBodyTransform();
      midGrabPose.getPose(midGrabTransform);
      taskExecutor.submit(new HandPoseTask(robotSideOfGraspingHand, yoTime, handPoseBehavior, Frame.WORLD, midGrabTransform, trajectoryTime));

      taskExecutor.submit(new FingerStateTask(robotSideOfGraspingHand, FingerState.OPEN, fingerStateBehavior));

      setHandPose(finalGrabPose, graspOrientation, finalGraspPosInWorld, finalToMidGraspVec, -WRIST_OFFSET_FROM_HAND);
      RigidBodyTransform finalGraspTransform = new RigidBodyTransform();
      finalGrabPose.getPose(finalGraspTransform);
      taskExecutor.submit(new HandPoseTask(robotSideOfGraspingHand, yoTime, handPoseBehavior, Frame.WORLD, finalGraspTransform, trajectoryTime));

      taskExecutor.submit(new FingerStateTask(robotSideOfGraspingHand, FingerState.CLOSE, fingerStateBehavior));
      
      haveInputsBeenSet.set(true);
   }

   private void setHandPose(FramePose poseToPack, Quat4d rotationToBePerformed, Point3d initialPosition, Vector3d positionOffset, double offsetDistance)
   {
      poseToPack.setToZero(fullRobotModel.getHandControlFrame(robotSideOfGraspingHand));
      poseToPack.changeFrame(ReferenceFrame.getWorldFrame());
      poseToPack.setOrientation(rotationToBePerformed);

      Vector3d scaledGraspVector = new Vector3d(positionOffset);
      scaledGraspVector.normalize();
      scaledGraspVector.scale(offsetDistance);

      Point3d handPose = new Point3d(initialPosition);
      handPose.add(scaledGraspVector);
      poseToPack.setPosition(handPose);
   }

   private Quat4d computeDesiredGraspOrientation(RigidBodyTransform debrisTransform, ReferenceFrame handFrame, Vector3d finalToMidGraspVec)
   {
      Vector3d xAxis = new Vector3d(1, 0, 0);
      Matrix3d piRoll = new Matrix3d();
      piRoll.rotX(Math.PI);

      Vector3d midToFinalGraspVec = new Vector3d(finalToMidGraspVec);
      //      midToFinalGraspVec.negate();

      AxisAngle4d rotationFromXAxis = new AxisAngle4d();
      GeometryTools.getRotationBasedOnNormal(rotationFromXAxis, midToFinalGraspVec, xAxis);

      FramePose handPoseBeforeRotation = new FramePose(worldFrame, new Point3d(0.0, 0.0, 0.0), rotationFromXAxis);
      PoseReferenceFrame handFrameBeforeRotation = new PoseReferenceFrame("handFrameBeforeRotation", handPoseBeforeRotation);

      FramePose debrisPoseInHandBeforeRotationFrame = new FramePose(worldFrame, debrisTransform);
      debrisPoseInHandBeforeRotationFrame.changeFrame(handFrameBeforeRotation);

      FramePose debrisPoseInHandBeforeRotationFrameWithPiRoll = new FramePose(worldFrame, debrisTransform);
      debrisPoseInHandBeforeRotationFrameWithPiRoll.changeFrame(handFrameBeforeRotation);

      Matrix3d piRollHandOrientation = new Matrix3d();
      debrisPoseInHandBeforeRotationFrameWithPiRoll.getOrientation(piRollHandOrientation);

      Matrix3d finalHandOrientation = new Matrix3d();
      finalHandOrientation.mul(piRoll, piRollHandOrientation);

      Quat4d finalHandQuat = new Quat4d();
      RotationFunctions.setQuaternionBasedOnMatrix3d(finalHandQuat, finalHandOrientation);
      debrisPoseInHandBeforeRotationFrameWithPiRoll.setOrientation(finalHandQuat);

      double debrisPoseInHandBeforeRotationRoll = debrisPoseInHandBeforeRotationFrame.getRoll();
      double debrisPoseInHandBeforeRotationWithPiRollRoll = debrisPoseInHandBeforeRotationFrameWithPiRoll.getRoll();

      Matrix3d rotationToBePerformedAroundX = new Matrix3d();

      if (Math.abs(debrisPoseInHandBeforeRotationRoll) <= Math.abs(debrisPoseInHandBeforeRotationWithPiRollRoll))
      {
         rotationToBePerformedAroundX.rotX(debrisPoseInHandBeforeRotationRoll);
      }
      else
      {
         rotationToBePerformedAroundX.rotX(debrisPoseInHandBeforeRotationWithPiRollRoll);
      }

      Matrix3d debrisOrientation = new Matrix3d();
      debrisPoseInHandBeforeRotationFrame.getOrientation(debrisOrientation);

      Matrix3d actualHandRotation = new Matrix3d();
      actualHandRotation.mul(rotationToBePerformedAroundX, debrisOrientation);

      Quat4d actualHandRotationQuat = new Quat4d();
      RotationFunctions.setQuaternionBasedOnMatrix3d(actualHandRotationQuat, actualHandRotation);

      FramePose graspPose = new FramePose(handFrameBeforeRotation);
      graspPose.setOrientation(actualHandRotationQuat);
      graspPose.changeFrame(worldFrame);

      Quat4d ret = new Quat4d();
      graspPose.getOrientation(ret);

      System.out.println(ret);

      return ret;
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
