package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveGraspLocation;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FingerStateBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.CoMHeightTask;
import us.ihmc.humanoidBehaviors.taskExecutor.FingerStateTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WholeBodyInverseKinematicTask;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.io.printing.SysoutTool;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.TransformReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspValveBehavior extends BehaviorInterface
{
   private final double TWELVE_O_CLOCK_HANDFRAME_ROLL = Math.PI;
   private final double VALVE_RIM_THICKNESS = 0.03;
   private final double WRIST_OFFSET_FROM_HAND = 0.11 * 0.75; // 0.11
   private final double MIDPOSE_OFFSET_FROM_FINALPOSE = 3.5 * WRIST_OFFSET_FROM_HAND; //0.3
   private final double HAND_POSE_TRAJECTORY_TIME = 2.0;//2.0;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame pelvisFrame;
   private final SDFFullRobotModel actualFullRobotModel;
   private final SDFFullRobotModel desiredFullRobotModel;
   private final WholeBodyIkSolver wholeBodyIKSolver;
   private final ArmJointName[] armJointNames;
   private final int numberOfArmJoints;
   private final LinkedHashMap<ArmJointName, Integer> armJointIndices = new LinkedHashMap<ArmJointName, Integer>();

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();
   private final ComHeightBehavior comHeightBehavior;
   private final HandPoseBehavior handPoseBehavior;
   private final WholeBodyInverseKinematicBehavior wholeBodyInverseKinematicBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   private final BooleanYoVariable haveInputsBeenSet;
   private final BooleanYoVariable reachedMidPoint;
   private final DoubleYoVariable yoTime;

   private RobotSide robotSideOfGraspingHand = null;

   private final boolean DEBUG = false;

   public GraspValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullRobotModel fullRobotModel,
         WholeBodyControllerParameters wholeBodyControllerParameters, DoubleYoVariable yoTime)
   {
      this(outgoingCommunicationBridge, new ComHeightBehavior(outgoingCommunicationBridge, yoTime), new HandPoseBehavior(outgoingCommunicationBridge, yoTime),
            fullRobotModel, wholeBodyControllerParameters, yoTime);
   }

   public GraspValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, ComHeightBehavior comHeightBehavior,
         HandPoseBehavior handPoseBehavior, SDFFullRobotModel fullRobotModel, WholeBodyControllerParameters wholeBodyControllerParameters,
         DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.actualFullRobotModel = fullRobotModel;
      this.desiredFullRobotModel = wholeBodyControllerParameters.createFullRobotModel();
      this.wholeBodyIKSolver = wholeBodyControllerParameters.createWholeBodyIkSolver();
      this.armJointNames = fullRobotModel.getRobotSpecificJointNames().getArmJointNames();
      this.numberOfArmJoints = armJointNames.length;
      for (int i = 0; i < numberOfArmJoints; i++)
      {
         armJointIndices.put(armJointNames[i], i);
      }

      this.yoTime = yoTime;
      this.comHeightBehavior = comHeightBehavior;
      this.handPoseBehavior = handPoseBehavior;
      this.wholeBodyInverseKinematicBehavior = new WholeBodyInverseKinematicBehavior(outgoingCommunicationBridge, wholeBodyControllerParameters,
            fullRobotModel, yoTime);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);

      pelvisFrame = fullRobotModel.getPelvis().getBodyFixedFrame();

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);
      reachedMidPoint = new BooleanYoVariable("reachedMidPoint", registry);
   }

   public void setGraspPoseWholeBodyIK(RobotSide robotSideOfGraspingHand, RigidBodyTransform valveTransformToWorld, double valveRadius,
         ValveGraspLocation valveGraspLocation, double graspApproachConeAngle, Axis valvePinJointAxisInValveFrame)
   {
      this.robotSideOfGraspingHand = robotSideOfGraspingHand;
      TransformReferenceFrame valveFrame = new TransformReferenceFrame("Valve", worldFrame, valveTransformToWorld);

      double graspPoseXOffset = 0.07;
      FramePose graspPose = getTwelveOClockGraspPalmFramePose(robotSideOfGraspingHand, valveRadius, 0.0, graspPoseXOffset, valveFrame, true);

      double graspClockwiseOffsetFromTwelveOClock = -valveGraspLocation.ordinal() * Math.toRadians(90.0);
      graspPose.rotatePoseAboutAxis(valveFrame, Axis.X, graspClockwiseOffsetFromTwelveOClock);

      if (DEBUG)
      {
         SysoutTool.println("graspClockwiseOffsetFromTwelveOClock : " + graspClockwiseOffsetFromTwelveOClock);
         SysoutTool.println("graspPose in valve frame : " + graspPose);
      }
      graspPose.changeFrame(worldFrame);

      graspPoses.add(graspPose);

      FingerStateTask openHandTask = new FingerStateTask(robotSideOfGraspingHand, FingerState.OPEN, fingerStateBehavior, yoTime);

      WholeBodyInverseKinematicTask moveHandToFavorableGraspApproachLocation = new WholeBodyInverseKinematicTask(robotSideOfGraspingHand, yoTime,
            wholeBodyInverseKinematicBehavior, graspPose, 0.5 * MIDPOSE_OFFSET_FROM_FINALPOSE, 1.0, 0);

      WholeBodyInverseKinematicTask movePalmToBeInContactWithValveRim = new WholeBodyInverseKinematicTask(robotSideOfGraspingHand, yoTime,
            wholeBodyInverseKinematicBehavior, graspPose, 1.0, 0, ControlledDoF.DOF_3P2R, true);

      FingerStateTask closeHandTask = new FingerStateTask(robotSideOfGraspingHand, FingerState.CLOSE, fingerStateBehavior, yoTime);

      pipeLine.clearAll();
      pipeLine.submitSingleTaskStage(openHandTask);
      pipeLine.submitSingleTaskStage(openHandTask);
      pipeLine.submitSingleTaskStage(moveHandToFavorableGraspApproachLocation);
      pipeLine.submitSingleTaskStage(movePalmToBeInContactWithValveRim);
      pipeLine.submitSingleTaskStage(closeHandTask);

      haveInputsBeenSet.set(true);
   }

   private ArrayList<FramePose> graspPoses = new ArrayList<FramePose>();

   public void setGraspPose(RobotSide robotSideOfGraspingHand, RigidBodyTransform valveTransformToWorld, double valveRadius,
         ValveGraspLocation valveGraspLocation, double graspApproachConeAngle, Axis valvePinJointAxisInValveFrame)
   {
      this.robotSideOfGraspingHand = robotSideOfGraspingHand;

      TransformReferenceFrame valveFrame = new TransformReferenceFrame("Valve", worldFrame, valveTransformToWorld);
      FramePose valvePose = new FramePose();
      valvePose.setToZero(valveFrame);

      FramePose finalGraspHandFramePose = new FramePose(valvePose);
      finalGraspHandFramePose.rotatePoseAboutAxis(finalGraspHandFramePose.getReferenceFrame(), Axis.X,
            robotSideOfGraspingHand.negateIfLeftSide(TWELVE_O_CLOCK_HANDFRAME_ROLL));

      double twelveOClockGraspPositionZinValveFrame;
      if (valveGraspLocation.equals(ValveGraspLocation.CENTER))
      {
         twelveOClockGraspPositionZinValveFrame = 0.0;
      }
      else
      {
         twelveOClockGraspPositionZinValveFrame = valveRadius;
      }
      finalGraspHandFramePose.translate(-WRIST_OFFSET_FROM_HAND - VALVE_RIM_THICKNESS, 0.0, twelveOClockGraspPositionZinValveFrame);

      FramePose preGraspHandFramePose = new FramePose(finalGraspHandFramePose);
      preGraspHandFramePose.translate(-MIDPOSE_OFFSET_FROM_FINALPOSE, 0.0, 0.0);


      double graspClockwiseOffsetFromTwelveOClock;
      if (valveGraspLocation.equals(ValveGraspLocation.CENTER))
      {
         graspClockwiseOffsetFromTwelveOClock = 0.0;
      }
      else
      {
         graspClockwiseOffsetFromTwelveOClock = -valveGraspLocation.ordinal() * Math.toRadians(90.0);
      }
      
      double angleToTwistHandDuringFinalGraspApproach = -0.5 * TurnValveBehavior.MAX_ANGLE_TO_ROTATE_PER_GRASP_CYCLE;

      preGraspHandFramePose.rotatePoseAboutAxis(valveFrame, valvePinJointAxisInValveFrame, graspClockwiseOffsetFromTwelveOClock
            - angleToTwistHandDuringFinalGraspApproach);

      finalGraspHandFramePose.rotatePoseAboutAxis(valveFrame, valvePinJointAxisInValveFrame, graspClockwiseOffsetFromTwelveOClock);

      if (DEBUG)
      {
         SysoutTool.println("graspClockwiseOffsetFromTwelveOClock : " + graspClockwiseOffsetFromTwelveOClock);
         SysoutTool.println("preGraspHandFramePose in valve frame: " + preGraspHandFramePose);
         SysoutTool.println("graspPose in valve frame : " + finalGraspHandFramePose);
      }

      preGraspHandFramePose.changeFrame(worldFrame);
      finalGraspHandFramePose.changeFrame(worldFrame);

      graspPoses.add(preGraspHandFramePose);
      graspPoses.add(finalGraspHandFramePose);

      double valveHeightOffsetFromNominal = valveTransformToWorld.cloneTranslation().getZ() - 1.3;
      double comHeightDesiredOffset = valveHeightOffsetFromNominal + valveRadius * Math.cos(graspClockwiseOffsetFromTwelveOClock);

      CoMHeightTask comHeightTask = new CoMHeightTask(comHeightDesiredOffset, yoTime, comHeightBehavior, 1.0);
      FingerStateTask openHandTask = new FingerStateTask(robotSideOfGraspingHand, FingerState.OPEN, fingerStateBehavior, yoTime);
      HandPoseTask moveHandToFavorableGraspApproachLocation = new HandPoseTask(robotSideOfGraspingHand, 1.0, preGraspHandFramePose, Frame.WORLD,
            handPoseBehavior, yoTime);
      HandPoseTask movePalmToBeInContactWithValveRim = new HandPoseTask(robotSideOfGraspingHand, 1.0, finalGraspHandFramePose, Frame.WORLD, handPoseBehavior,
            yoTime);
      FingerStateTask closeHandTask = new FingerStateTask(robotSideOfGraspingHand, FingerState.CLOSE, fingerStateBehavior, yoTime);

      pipeLine.clearAll();
      pipeLine.submitTaskForPallelPipesStage(comHeightBehavior, comHeightTask);
      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, moveHandToFavorableGraspApproachLocation);
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, openHandTask);
      pipeLine.submitSingleTaskStage(movePalmToBeInContactWithValveRim);
      pipeLine.submitSingleTaskStage(closeHandTask);

      haveInputsBeenSet.set(true);
   }

   private Vector3d getLongAxisOfValveRimAtTwelveOClock(boolean putThumbInsideRim)
   {
      Vector3d ret = new Vector3d(0.0, robotSideOfGraspingHand.negateIfRightSide(1.0), 0.0); // Thumb on outside of rim: (0.0, robotSideOfGraspingHand.negateIfRightSide(-1.0), 0.0)

      if (putThumbInsideRim)
         ret.negate();

      return ret;
   }

   private FramePose getTwelveOClockGraspPalmFramePose(RobotSide robotSideOfGraspingHand, double valveRadius, double twelveOClockGraspApproachPitch,
         double positionOffset, ReferenceFrame valveFrame, boolean putThumbInsideRim)
   {
      Point3d twelveOClockGraspPositionInValveFrame = new Point3d(0.0, 0.0, valveRadius);
      AxisAngle4d twelveOClockGraspOrientationInValveFrame = new AxisAngle4d(getLongAxisOfValveRimAtTwelveOClock(putThumbInsideRim),
            twelveOClockGraspApproachPitch);

      FramePose twelveOClockGraspPose = new FramePose(valveFrame, twelveOClockGraspPositionInValveFrame, twelveOClockGraspOrientationInValveFrame);

      twelveOClockGraspPose.changeFrame(new PoseReferenceFrame("graspPoseFrame", twelveOClockGraspPose));
      twelveOClockGraspPose.translate(positionOffset - VALVE_RIM_THICKNESS, 0.0, 0.0);
      twelveOClockGraspPose.changeFrame(valveFrame);

      return twelveOClockGraspPose;
   }

   public FramePose getGraspApproachPose()
   {
      return graspPoses.get(0);
   }

   public FramePose getDesiredFinalGraspPose()
   {
      if (!hasInputBeenSet())
         throw new RuntimeException("Must set input before calling this method!");
      return graspPoses.get(graspPoses.size() - 1);
   }

   //   private final Quat4d desiredGraspOrientation = new Quat4d();
   //   private final Vector3d valveOffsetFromWorld = new Vector3d();

   // public void setGraspPose(RigidBodyTransform valveTransformToWorld, Point3d pointToGraspInWorldFrame, Vector3d approachDirectionInWorld,
   // double midPoseOffsetFromFinalPose)
   //{
   //if (approachDirectionInWorld.length() == 0.0)
   //{
   // throw new RuntimeException("finalToMidGraspVec has not been set!");
   //}
   //robotSideOfGraspingHand = getRobotSideOfHandClosestToGraspPosition(pointToGraspInWorldFrame, pelvisFrame);
   //
   //double [] desiredArmPose = getCurrentArmPose(robotSideOfGraspingHand);
   //ArmJointName wristTwist = ArmJointName.WRIST_PITCH;
   //setSingleJoint(desiredArmPose, robotSideOfGraspingHand, wristTwist, fullRobotModel.getArmJoint(robotSideOfGraspingHand, wristTwist).getJointLimitLower(), true);
   //
   //computeDesiredGraspOrientation(valveTransformToWorld, approachDirectionInWorld, fullRobotModel.getHandControlFrame(robotSideOfGraspingHand),
   //    desiredGraspOrientation);
   //
   //FramePose midGrabPose = new FramePose(worldFrame, getOffsetPoint3dCopy(pointToGraspInWorldFrame, approachDirectionInWorld, -midPoseOffsetFromFinalPose),
   //    desiredGraspOrientation);
   //RigidBodyTransform midGrabTransform = new RigidBodyTransform();
   //midGrabPose.getPose(midGrabTransform);
   //
   //FramePose finalGrabPose = new FramePose(worldFrame, getOffsetPoint3dCopy(pointToGraspInWorldFrame, approachDirectionInWorld, -WRIST_OFFSET_FROM_HAND),
   //    desiredGraspOrientation);
   //RigidBodyTransform finalGraspTransform = new RigidBodyTransform();
   //finalGrabPose.getPose(finalGraspTransform);
   //
   //valveTransformToWorld.get(valveOffsetFromWorld);
   //double comHeightDesired = valveOffsetFromWorld.getZ() - 1.15;
   //
   //taskExecutor.clear();
   //taskExecutor.submit(new CoMHeightTask(comHeightDesired, yoTime, comHeightBehavior, 1.0));
   //taskExecutor.submit(new HandPoseTask(robotSideOfGraspingHand, desiredArmPose, yoTime, handPoseBehavior, 1.0));
   //taskExecutor.submit(new HandPoseTask(robotSideOfGraspingHand, yoTime, handPoseBehavior, Frame.WORLD, midGrabTransform, HAND_POSE_TRAJECTORY_TIME));
   //taskExecutor.submit(new FingerStateTask(robotSideOfGraspingHand, FingerState.OPEN, fingerStateBehavior, yoTime));
   //
   //boolean stopHandIfCollision = false;
   //taskExecutor.submit(new HandPoseTask(robotSideOfGraspingHand, yoTime, handPoseBehavior, Frame.WORLD, finalGraspTransform, HAND_POSE_TRAJECTORY_TIME,
   //    stopHandIfCollision));
   //taskExecutor.submit(new FingerStateTask(robotSideOfGraspingHand, FingerState.CLOSE, fingerStateBehavior, yoTime));
   //
   //haveInputsBeenSet.set(true);
   //}

   private Vector3d computeGraspOffsetFromValveOrigin(double valveRadius, double graspClockwiseOffsetFromTwelveOClock, boolean graspValveRim)
   {
      Vector3d graspOffsetFromValveCenter = new Vector3d(-WRIST_OFFSET_FROM_HAND, 0.0, 0.0);
      if (graspValveRim)
         graspOffsetFromValveCenter.setZ(valveRadius);

      if (graspClockwiseOffsetFromTwelveOClock != 0.0)
      {
         RigidBodyTransform afterToBeforeRotationTransform = new RigidBodyTransform();
         afterToBeforeRotationTransform.rotX(graspClockwiseOffsetFromTwelveOClock);
         afterToBeforeRotationTransform.transform(graspOffsetFromValveCenter);
      }
      return graspOffsetFromValveCenter;
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
      handPose.setOrientation(0.0, 0.0, -0.5 * Math.PI); // MATH.PI or -0.5 * MATH.PI
      handPose.changeFrame(handFrameBeforeGrasping);

      handPose.changeFrame(worldFrame);
      handPose.getOrientation(desiredGraspOrientationToPack);
   }

   public static RobotSide getRobotSideOfHandClosestToGraspPosition(RigidBodyTransform valveTransformToWorld, ReferenceFrame pelvisFrame)
   {
      Vector3d worldToValveVec = new Vector3d();
      valveTransformToWorld.getTranslation(worldToValveVec);

      Point3d finalGraspPosInWorld = new Point3d(worldToValveVec);

      return getRobotSideOfHandClosestToGraspPosition(finalGraspPosInWorld, pelvisFrame);
   }

   public static RobotSide getRobotSideOfHandClosestToGraspPosition(Point3d graspPositionInWorld, ReferenceFrame pelvisFrame)
   {
      FramePoint graspPositionInPelvisFrame = new FramePoint(ReferenceFrame.getWorldFrame(), graspPositionInWorld);
      graspPositionInPelvisFrame.changeFrame(pelvisFrame);

      if (graspPositionInPelvisFrame.getY() <= 0.0)
         return RobotSide.RIGHT;
      else
         return RobotSide.LEFT;
   }

   private void setSingleJoint(double[] armPoseToPack, RobotSide robotSide, ArmJointName armJointName, double desiredJointAngle,
         boolean clipDesiredQToJointLimits)
   {
      double q = desiredJointAngle;
      int armJointIndex = armJointIndices.get(armJointName);
      if (clipDesiredQToJointLimits)
      {
         q = clipDesiredToJointLimits(robotSide, armJointName, desiredJointAngle);
      }

      armPoseToPack[armJointIndex] = q;
   }

   private double clipDesiredToJointLimits(RobotSide robotSide, ArmJointName armJointName, double desiredJointAngle)
   {
      double q;
      double qMin = actualFullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitLower();
      double qMax = actualFullRobotModel.getArmJoint(robotSide, armJointName).getJointLimitUpper();

      if (qMin > qMax)
      {
         double temp = qMax;
         qMax = qMin;
         qMin = temp;
      }

      q = MathTools.clipToMinMax(desiredJointAngle, qMin, qMax);
      return q;
   }

   private double[] getCurrentArmPose(RobotSide robotSide)
   {
      double[] armPose = new double[numberOfArmJoints];

      for (int jointNum = 0; jointNum < numberOfArmJoints; jointNum++)
      {
         ArmJointName jointName = armJointNames[jointNum];
         double currentAngle = actualFullRobotModel.getArmJoint(robotSide, jointName).getQ();
         armPose[jointNum] = currentAngle;
      }

      return armPose;
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
      reachedMidPoint.set(false);

   }

   @Override
   public boolean hasInputBeenSet()
   {
      return haveInputsBeenSet.getBooleanValue();
   }
}
