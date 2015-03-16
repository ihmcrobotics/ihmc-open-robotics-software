package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveGraspLocation;
import us.ihmc.humanoidBehaviors.behaviors.TurnValveBehavior.ValveTurnDirection;
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
import us.ihmc.utilities.io.printing.PrintTools;
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
   private static final boolean STOP_GRASP_MOTION_IF_COLLISION_IS_DETECTED = true;
   private final double TWELVE_O_CLOCK_HANDFRAME_ROLL_ANGLE = Math.PI;
   private final double VALVE_RIM_THICKNESS = 0.08;
   private final double WRIST_OFFSET_FROM_HAND = 0.11 * 0.75; // 0.11
   private final double MIDPOSE_OFFSET_FROM_FINALPOSE = 3.5 * WRIST_OFFSET_FROM_HAND; //0.3
   private final double HAND_POSE_TRAJECTORY_TIME = 2.0; //2.0;

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
         ValveGraspLocation valveGraspLocation, ValveTurnDirection valveTurnDirection, double graspApproachConeAngle, Axis valvePinJointAxisInValveFrame)
   {
      this.robotSideOfGraspingHand = robotSideOfGraspingHand;
      TransformReferenceFrame valveFrame = new TransformReferenceFrame("Valve", worldFrame, valveTransformToWorld);

      computeGraspPoses(valveFrame, valveRadius, valveGraspLocation, valveTurnDirection, graspApproachConeAngle, valvePinJointAxisInValveFrame, graspPoses);
      
//      double graspPoseXOffset = 0.07;
//      double graspPitchAngle = Math.toRadians(15.0);
//      FramePose graspPose = getTwelveOClockGraspPalmFramePose(robotSideOfGraspingHand, valveRadius, graspPitchAngle, graspPoseXOffset, valveFrame, false);
//
//      double graspClockwiseOffsetFromTwelveOClock;
//      if (valveGraspLocation.equals(ValveGraspLocation.CENTER))
//      {
//         graspClockwiseOffsetFromTwelveOClock = 0.0;
//      }
//      else
//      {
//         graspClockwiseOffsetFromTwelveOClock = -valveGraspLocation.ordinal() * Math.toRadians(90.0);
//      }
//      graspPose.rotatePoseAboutAxis(valveFrame, valvePinJointAxisInValveFrame, graspClockwiseOffsetFromTwelveOClock);
//
//      if (DEBUG)
//      {
//         PrintTools.debug(this, "graspClockwiseOffsetFromTwelveOClock : " + graspClockwiseOffsetFromTwelveOClock);
//         PrintTools.debug(this, "graspPose in valve frame : " + graspPose);
//      }
//      graspPose.changeFrame(worldFrame);
//
//      graspPoses.add(graspPose);
      
      double comHeightDesiredOffset = graspPoses.get(1).getZ() - 1.27;
      CoMHeightTask comHeightTask = new CoMHeightTask(comHeightDesiredOffset, yoTime, comHeightBehavior, 1.0);
      

      FingerStateTask openHandTask = new FingerStateTask(robotSideOfGraspingHand, FingerState.OPEN, fingerStateBehavior, yoTime);

//      WholeBodyInverseKinematicTask moveHandToFavorableGraspApproachLocation = new WholeBodyInverseKinematicTask(robotSideOfGraspingHand, yoTime,
//            wholeBodyInverseKinematicBehavior, graspPose, 0.5 * MIDPOSE_OFFSET_FROM_FINALPOSE, 1.0, 0);

      WholeBodyInverseKinematicTask moveHandToFavorableGraspApproachLocation = new WholeBodyInverseKinematicTask(robotSideOfGraspingHand, yoTime,
            wholeBodyInverseKinematicBehavior, graspPoses.get(0), 1.0, 10, ControlledDoF.DOF_3P3R, false);
      
      WholeBodyInverseKinematicTask movePalmToBeInContactWithValveRim = new WholeBodyInverseKinematicTask(robotSideOfGraspingHand, yoTime,
            wholeBodyInverseKinematicBehavior, graspPoses.get(1), 2.0, 10, ControlledDoF.DOF_3P3R, false);
      
//      WholeBodyInverseKinematicTask rotatePalmAboutRimLongAxis = new WholeBodyInverseKinematicTask(robotSideOfGraspingHand, yoTime,
//            wholeBodyInverseKinematicBehavior, graspPoseLongAxisRotated, 1.0, 10, ControlledDoF.DOF_3P3R, true);

      FingerStateTask closeHandTask = new FingerStateTask(robotSideOfGraspingHand, FingerState.CLOSE, fingerStateBehavior, yoTime);

      pipeLine.clearAll();
      pipeLine.submitSingleTaskStage(openHandTask);
      pipeLine.submitSingleTaskStage(comHeightTask);
      pipeLine.submitSingleTaskStage(moveHandToFavorableGraspApproachLocation);
      pipeLine.submitSingleTaskStage(movePalmToBeInContactWithValveRim);
      pipeLine.submitSingleTaskStage(closeHandTask);

      haveInputsBeenSet.set(true);
   }

   private ArrayList<FramePose> graspPoses = new ArrayList<FramePose>();

   public void setGraspPose(RobotSide robotSideOfGraspingHand, RigidBodyTransform valveTransformToWorld, double valveRadius,
         ValveGraspLocation valveGraspLocation, ValveTurnDirection valveTurnDirection, double graspApproachConeAngle, Axis valvePinJointAxisInValveFrame)
   {
      this.robotSideOfGraspingHand = robotSideOfGraspingHand;
      TransformReferenceFrame valveFrame = new TransformReferenceFrame("Valve", worldFrame, valveTransformToWorld);

      computeGraspPoses(valveFrame, valveRadius, valveGraspLocation, valveTurnDirection, graspApproachConeAngle, valvePinJointAxisInValveFrame, graspPoses);
      
      FramePose preGraspHandFramePose = graspPoses.get(0);
      FramePose finalGraspHandFramePose = graspPoses.get(1);
      
      double comHeightDesiredOffset = finalGraspHandFramePose.getZ() - 1.27;
      CoMHeightTask comHeightTask = new CoMHeightTask(comHeightDesiredOffset, yoTime, comHeightBehavior, 1.0);
      FingerStateTask openHandTask = new FingerStateTask(robotSideOfGraspingHand, FingerState.OPEN, fingerStateBehavior, yoTime);
      HandPoseTask moveHandToFavorableGraspApproachLocation = new HandPoseTask(robotSideOfGraspingHand, 1.0, preGraspHandFramePose, Frame.WORLD,
            handPoseBehavior, yoTime);
      HandPoseTask movePalmToBeInContactWithValveRim = new HandPoseTask(robotSideOfGraspingHand, 1.0, finalGraspHandFramePose, Frame.WORLD, handPoseBehavior,
            yoTime, STOP_GRASP_MOTION_IF_COLLISION_IS_DETECTED);
      FingerStateTask closeHandTask = new FingerStateTask(robotSideOfGraspingHand, FingerState.CLOSE, fingerStateBehavior, yoTime);

      pipeLine.clearAll();
      pipeLine.submitTaskForPallelPipesStage(comHeightBehavior, comHeightTask);
      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, moveHandToFavorableGraspApproachLocation);
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, openHandTask);
      pipeLine.submitSingleTaskStage(movePalmToBeInContactWithValveRim);

      if (!valveGraspLocation.equals(ValveGraspLocation.CENTER))
         pipeLine.submitSingleTaskStage(closeHandTask);

      haveInputsBeenSet.set(true);
   }

   private void computeGraspPoses(ReferenceFrame valveFrame, double valveRadius, ValveGraspLocation valveGraspLocation, ValveTurnDirection valveTurnDirection,
         double graspApproachConeAngle, Axis valvePinJointAxisInValveFrame, ArrayList<FramePose> graspPosesToPack)
   {
      FramePose valvePose = new FramePose();
      valvePose.setToZero(valveFrame);

      FramePose finalGraspHandFramePose = getTwelveOClockGraspHandFramePose(valvePose, valveGraspLocation, graspApproachConeAngle, valveRadius,
            robotSideOfGraspingHand);
      
      double graspClockwiseOffsetFromTwelveOClock;
      if (valveGraspLocation.equals(ValveGraspLocation.CENTER))
      {
         graspClockwiseOffsetFromTwelveOClock = 0.0;
      }
      else
      {
         graspClockwiseOffsetFromTwelveOClock = -valveGraspLocation.ordinal() * Math.toRadians(90.0);
      }

      double angleToTwistHandDuringFinalGraspApproach;
      if (valveTurnDirection == ValveTurnDirection.CLOCKWISE)
      {
         angleToTwistHandDuringFinalGraspApproach = -0.5 * TurnValveBehavior.MAX_ANGLE_TO_ROTATE_PER_GRASP_CYCLE;
      }
      else
      {
         angleToTwistHandDuringFinalGraspApproach = 0.5 * TurnValveBehavior.MAX_ANGLE_TO_ROTATE_PER_GRASP_CYCLE;
      }

      FramePose preGraspHandFramePose = new FramePose(finalGraspHandFramePose);
      preGraspHandFramePose.translate(-MIDPOSE_OFFSET_FROM_FINALPOSE, 0.0, 0.0);
      preGraspHandFramePose.rotatePoseAboutAxis(valveFrame, valvePinJointAxisInValveFrame, graspClockwiseOffsetFromTwelveOClock
            - angleToTwistHandDuringFinalGraspApproach);

      finalGraspHandFramePose.rotatePoseAboutAxis(valveFrame, valvePinJointAxisInValveFrame, graspClockwiseOffsetFromTwelveOClock);

      if (DEBUG)
      {
         PrintTools.debug(this, "graspClockwiseOffsetFromTwelveOClock : " + graspClockwiseOffsetFromTwelveOClock);
         PrintTools.debug(this, "preGraspHandFramePose in valve frame: " + preGraspHandFramePose);
         PrintTools.debug(this, "graspPose in valve frame : " + finalGraspHandFramePose);
      }

      preGraspHandFramePose.changeFrame(worldFrame);
      finalGraspHandFramePose.changeFrame(worldFrame);

      graspPosesToPack.add(preGraspHandFramePose);
      graspPosesToPack.add(finalGraspHandFramePose);
   }
   
   private FramePose getTwelveOClockGraspHandFramePose(FramePose valvePose, ValveGraspLocation valveGraspLocation, double graspApproachConeAngle,
         double valveRadius, RobotSide robotSideOfGraspingHand)
   {
      FramePose ret = new FramePose(valvePose);
      ret.rotatePoseAboutAxis(ret.getReferenceFrame(), Axis.X, robotSideOfGraspingHand.negateIfLeftSide(TWELVE_O_CLOCK_HANDFRAME_ROLL_ANGLE));

      double twelveOClockGraspPositionZinValveFrame;
      if (valveGraspLocation.equals(ValveGraspLocation.CENTER))
      {
         twelveOClockGraspPositionZinValveFrame = 0.0;
      }
      else
      {
         twelveOClockGraspPositionZinValveFrame = valveRadius;
      }

      ret.translate(-VALVE_RIM_THICKNESS, 0.0, twelveOClockGraspPositionZinValveFrame);
      PoseReferenceFrame longAxisOfValveRimAtTwelveOClockFrame = new PoseReferenceFrame("valveRimFrame", ret);
      ret.translate(-WRIST_OFFSET_FROM_HAND, 0.0, 0.0);
      ret.rotatePoseAboutAxis(longAxisOfValveRimAtTwelveOClockFrame, Axis.Y, graspApproachConeAngle);

      return ret;
   }

   private Vector3d getLongAxisOfValveRimAtTwelveOClock(boolean putThumbInsideRim)
   {
      Vector3d ret = new Vector3d(0.0, robotSideOfGraspingHand.negateIfRightSide(1.0), 0.0);

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
