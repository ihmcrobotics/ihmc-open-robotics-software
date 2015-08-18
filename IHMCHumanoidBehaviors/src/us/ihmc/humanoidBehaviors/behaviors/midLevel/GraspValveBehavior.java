package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.communication.packets.dataobjects.FingerState;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
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
import us.ihmc.robotics.Axis;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.WholeBodyIkSolver.ControlledDoF;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspValveBehavior extends BehaviorInterface
{
   protected double HANDFRAME_ROLL_ANGLE_IN_VALVE_FRAME_AT_TWELVE_O_CLOCK_GRASP_POSITION = Math.PI;

   private final double VALVE_RIM_THICKNESS = 0.08;
   private final double WRIST_OFFSET_FROM_HAND = 0.11 * 0.6; // 0.11
   private final double MIDPOSE_OFFSET_FROM_FINALPOSE = 3.5 * WRIST_OFFSET_FROM_HAND; //0.3

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();
   private final ComHeightBehavior comHeightBehavior;
   private final HandPoseBehavior handPoseBehavior;
   private final WholeBodyInverseKinematicBehavior wholeBodyInverseKinematicBehavior;
   private final FingerStateBehavior fingerStateBehavior;

   private final BooleanYoVariable haveInputsBeenSet;
   private final BooleanYoVariable reachedMidPoint;
   private final DoubleYoVariable yoTime;

   private RobotSide robotSideOfGraspingHand = null;
   
   public enum ValveGraspMethod
   {
      RIM, SPINNER_HANDLE, SPOKE
   }

   private final boolean DEBUG = false;

   public GraspValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullHumanoidRobotModel fullRobotModel,
         WholeBodyControllerParameters wholeBodyControllerParameters, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);

      this.yoTime = yoTime;
      
      this.comHeightBehavior = new ComHeightBehavior(outgoingCommunicationBridge, yoTime);
      this.handPoseBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      this.wholeBodyInverseKinematicBehavior = new WholeBodyInverseKinematicBehavior(outgoingCommunicationBridge, wholeBodyControllerParameters,
            fullRobotModel, yoTime);
      fingerStateBehavior = new FingerStateBehavior(outgoingCommunicationBridge, yoTime);

      haveInputsBeenSet = new BooleanYoVariable("haveInputsBeenSet", registry);
      reachedMidPoint = new BooleanYoVariable("reachedMidPoint", registry);
   }

   public void setGraspPose(RobotSide robotSideOfGraspingHand, RigidBodyTransform valveTransformToWorld, double valveRadius, ValveGraspMethod graspMethod,
	         ValveTurnDirection turnDirection, double graspApproachConeAngle, Axis valvePinJointAxisInValveFrame, boolean stopHandIfCollision)
	   {
	      ValveGraspLocation graspLocation;
	      if (turnDirection == ValveTurnDirection.CLOCKWISE)
	      {
	         graspLocation = ValveGraspLocation.TWELVE_O_CLOCK;
	      }
	      else
	      {
	         graspLocation = ValveGraspLocation.SIX_O_CLOCK;
	      }
	      
	      setGraspPose(robotSideOfGraspingHand, valveTransformToWorld, valveRadius, graspLocation, graspMethod, turnDirection, graspApproachConeAngle, valvePinJointAxisInValveFrame, stopHandIfCollision);
	   }

   private ArrayList<FramePose> graspPoses = new ArrayList<FramePose>();

   public void setGraspPose(RobotSide robotSideOfGraspingHand, RigidBodyTransform valveTransformToWorld, double valveRadius,
	         ValveGraspLocation graspLocation, ValveGraspMethod graspMethod, ValveTurnDirection turnDirection, double graspApproachConeAngle, Axis valvePinJointAxisInValveFrame, boolean stopHandIfCollision)
	  {
      this.robotSideOfGraspingHand = robotSideOfGraspingHand;
      TransformReferenceFrame valveFrame = new TransformReferenceFrame("Valve", worldFrame, valveTransformToWorld);

      computeGraspPoses(valveFrame, valvePinJointAxisInValveFrame, graspLocation, graspMethod, turnDirection, graspApproachConeAngle, valveRadius, graspPoses);
      
      FramePose preGraspHandFramePose = graspPoses.get(0);
      FramePose finalGraspHandFramePose = graspPoses.get(1);
      
      double comHeightDesiredOffset = finalGraspHandFramePose.getZ() - 1.27;
      CoMHeightTask comHeightTask = new CoMHeightTask(comHeightDesiredOffset, yoTime, comHeightBehavior, 1.0);
      FingerStateTask openHandTask = new FingerStateTask(robotSideOfGraspingHand, FingerState.OPEN, fingerStateBehavior, yoTime);
      HandPoseTask moveHandToFavorableGraspApproachLocation = new HandPoseTask(robotSideOfGraspingHand, 1.0, preGraspHandFramePose, Frame.WORLD,
            handPoseBehavior, yoTime);
      HandPoseTask movePalmToBeInContactWithValveRim = new HandPoseTask(robotSideOfGraspingHand, 1.0, finalGraspHandFramePose, Frame.WORLD, handPoseBehavior,
            yoTime, stopHandIfCollision);
      FingerStateTask closeHandTask = new FingerStateTask(robotSideOfGraspingHand, FingerState.CLOSE, fingerStateBehavior, yoTime);

      pipeLine.clearAll();
      pipeLine.submitSingleTaskStage(comHeightTask);
      pipeLine.submitTaskForPallelPipesStage(handPoseBehavior, moveHandToFavorableGraspApproachLocation);
      pipeLine.submitTaskForPallelPipesStage(fingerStateBehavior, openHandTask);
      pipeLine.submitSingleTaskStage(movePalmToBeInContactWithValveRim);

      if (!graspLocation.equals(ValveGraspLocation.CENTER))
         pipeLine.submitSingleTaskStage(closeHandTask);

      haveInputsBeenSet.set(true);
   }

   private void computeGraspPoses(ReferenceFrame valveFrame,  Axis valvePinJointAxisInValveFrame, ValveGraspLocation graspLocation, ValveGraspMethod graspMethod, ValveTurnDirection turnDirection,
	         double graspApproachConeAngle, double valveRadius, ArrayList<FramePose> graspPosesToPack)
   {
      FramePose finalGraspHandFramePose = getTwelveOClockGraspHandFramePose(valveFrame, graspLocation, graspMethod, graspApproachConeAngle, valveRadius,
            robotSideOfGraspingHand);
            
      double graspClockwiseOffsetFromTwelveOClock;
      if (graspLocation.equals(ValveGraspLocation.CENTER))
      {
         graspClockwiseOffsetFromTwelveOClock = 0.0;
      }
      else
      {
         graspClockwiseOffsetFromTwelveOClock = -graspLocation.ordinal() * Math.toRadians(90.0);
      }


      double angleToRotateHandPoseAboutValvePinjointDuringFinalGraspApproach = 0.0;

//         if (turnDirection == ValveTurnDirection.CLOCKWISE)
//         {
//            angleToRotateHandPoseAboutValvePinjointDuringFinalGraspApproach = -0.5 * TurnValveBehavior.MAX_ANGLE_TO_ROTATE_PER_GRASP_CYCLE;
//         }
//         else
//         {
//            angleToRotateHandPoseAboutValvePinjointDuringFinalGraspApproach = 0.5 * TurnValveBehavior.MAX_ANGLE_TO_ROTATE_PER_GRASP_CYCLE;
//         }

      FramePose preGraspHandFramePose = new FramePose(finalGraspHandFramePose);
      
      preGraspHandFramePose.changeFrame(new PoseReferenceFrame("finalGraspPoseFrame", preGraspHandFramePose));
      preGraspHandFramePose.translate(-MIDPOSE_OFFSET_FROM_FINALPOSE, 0.0, 0.0);
      preGraspHandFramePose.changeFrame(finalGraspHandFramePose.getReferenceFrame());

      boolean lockOrientation = graspMethod.equals(ValveGraspMethod.SPINNER_HANDLE);
      
      preGraspHandFramePose.rotatePoseAboutAxis(valveFrame, valvePinJointAxisInValveFrame, graspClockwiseOffsetFromTwelveOClock
            - angleToRotateHandPoseAboutValvePinjointDuringFinalGraspApproach, false, lockOrientation);
      
      finalGraspHandFramePose.rotatePoseAboutAxis(valveFrame, valvePinJointAxisInValveFrame, graspClockwiseOffsetFromTwelveOClock, false, lockOrientation);

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
   
   private FramePose getTwelveOClockGraspHandFramePose(ReferenceFrame valveFrame, ValveGraspLocation graspLocation, ValveGraspMethod graspMethod, double graspApproachConeAngle,
         double valveRadius, RobotSide robotSideOfGraspingHand)
   {
      FramePose ret = new FramePose();
      ret.setToZero(valveFrame);
      
      Axis pitchAxis = Axis.Y;
      Axis rollAxis = Axis.X;
      Axis yawAxis = Axis.Z;
      
      ret.rotatePoseAboutAxis(valveFrame, rollAxis, robotSideOfGraspingHand.negateIfLeftSide(HANDFRAME_ROLL_ANGLE_IN_VALVE_FRAME_AT_TWELVE_O_CLOCK_GRASP_POSITION));

      Vector3d graspPositionInValveFrame = new Vector3d(-VALVE_RIM_THICKNESS -WRIST_OFFSET_FROM_HAND, 0.0, valveRadius);

      if (graspLocation == ValveGraspLocation.CENTER)
         graspPositionInValveFrame.setZ(0.0);
      
      ret.translate(graspPositionInValveFrame);
           
      if (graspMethod == ValveGraspMethod.SPINNER_HANDLE)
      {
         ret.rotatePoseAboutAxis(valveFrame, yawAxis, robotSideOfGraspingHand.negateIfLeftSide(0.5 * Math.PI));
         ret.translate(-1.5 * VALVE_RIM_THICKNESS, 0.0, 0.0);
      }
      
      ret.rotatePoseAboutAxis(new PoseReferenceFrame("graspLocationFrame", ret), pitchAxis, graspApproachConeAngle);
      
      return ret;
   }

   public void setGraspPoseWholeBodyIK(RobotSide robotSideOfGraspingHand, RigidBodyTransform valveTransformToWorld, double valveRadius,
	         ValveGraspLocation graspLocation, ValveTurnDirection turnDirection, double graspApproachConeAngle, Axis valvePinJointAxisInValveFrame)
	   {
	      this.robotSideOfGraspingHand = robotSideOfGraspingHand;
	      TransformReferenceFrame valveFrame = new TransformReferenceFrame("Valve", worldFrame, valveTransformToWorld);

	      computeGraspPoses(valveFrame, valvePinJointAxisInValveFrame, graspLocation, ValveGraspMethod.RIM, turnDirection, graspApproachConeAngle, valveRadius, graspPoses);

	      
	      double comHeightDesiredOffset = graspPoses.get(1).getZ() - 1.27;
	      CoMHeightTask comHeightTask = new CoMHeightTask(comHeightDesiredOffset, yoTime, comHeightBehavior, 1.0);
	      

	      FingerStateTask openHandTask = new FingerStateTask(robotSideOfGraspingHand, FingerState.OPEN, fingerStateBehavior, yoTime);


	      WholeBodyInverseKinematicTask moveHandToFavorableGraspApproachLocation = new WholeBodyInverseKinematicTask(robotSideOfGraspingHand, yoTime,
	            wholeBodyInverseKinematicBehavior, graspPoses.get(0), 1.0, 10, ControlledDoF.DOF_3P3R, false);
	      
	      WholeBodyInverseKinematicTask movePalmToBeInContactWithValveRim = new WholeBodyInverseKinematicTask(robotSideOfGraspingHand, yoTime,
	            wholeBodyInverseKinematicBehavior, graspPoses.get(1), 2.0, 10, ControlledDoF.DOF_3P3R, false);


	      FingerStateTask closeHandTask = new FingerStateTask(robotSideOfGraspingHand, FingerState.CLOSE, fingerStateBehavior, yoTime);

	      pipeLine.clearAll();
	      pipeLine.submitSingleTaskStage(openHandTask);
	      pipeLine.submitSingleTaskStage(comHeightTask);
	      pipeLine.submitSingleTaskStage(moveHandToFavorableGraspApproachLocation);
	      pipeLine.submitSingleTaskStage(movePalmToBeInContactWithValveRim);
	      pipeLine.submitSingleTaskStage(closeHandTask);

	      haveInputsBeenSet.set(true);
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
   public void doPostBehaviorCleanup()
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
