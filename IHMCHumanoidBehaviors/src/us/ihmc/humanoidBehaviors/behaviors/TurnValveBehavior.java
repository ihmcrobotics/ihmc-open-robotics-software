package us.ihmc.humanoidBehaviors.behaviors;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.behaviors.TurnValvePacket;
import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.WalkToLocationBehavior.WalkingOrientation;
import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior.ValveGraspMethod;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.taskExecutor.CoMHeightTask;
import us.ihmc.humanoidBehaviors.taskExecutor.GraspTurnAndUnGraspValveTask;
import us.ihmc.humanoidBehaviors.taskExecutor.HandPoseTask;
import us.ihmc.humanoidBehaviors.taskExecutor.ScriptTask;
import us.ihmc.humanoidBehaviors.taskExecutor.WalkToLocationTask;
import us.ihmc.robotics.Axis;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.referenceFrames.Pose2dReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.PipeLine;
import us.ihmc.utilities.taskExecutor.Task;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class TurnValveBehavior extends BehaviorInterface
{
   public enum ValveGraspLocation
   {
      TWELVE_O_CLOCK, THREE_O_CLOCK, SIX_O_CLOCK, NINE_O_CLOCK, CENTER
   }
   
   public enum ValveTurnDirection
   {
      CLOCKWISE, COUNTERCLOCKWISE
   }

   private static final boolean DEBUG = false;
   private final boolean USE_WHOLE_BODY_INVERSE_KINEMATICS = false;

   public static final RobotSide robotSideOfHandToUse = RobotSide.RIGHT;
   public static final ValveGraspLocation DEFAULT_GRASP_LOCATION = ValveGraspLocation.TWELVE_O_CLOCK;
   public static final double DEFAULT_GRASP_APPROACH_CONE_ANGLE = Math.toRadians(20.0);
   public static final double MAX_ANGLE_TO_ROTATE_PER_GRASP_CYCLE = Math.toRadians(160.0);
   public static final double DEFAULT_ROTATION_RATE_RAD_PER_SEC = 2.0 * Math.PI / 7.0;
   public static final boolean STOP_HAND_IF_GRASP_COLLISION = true;
   public static final boolean STOP_HAND_IF_TURN_COLLISION = true;
   public static final double howFarToStandToTheRightOfValve = 2.0 * robotSideOfHandToUse.negateIfRightSide(0.13); //0.13
   public static final double howFarToStandBackFromValve = 1.25 * 0.64; //0.64

   private final SDFFullRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;

   private final PipeLine<BehaviorInterface> pipeLine = new PipeLine<>();

   private final WalkingControllerParameters walkingControllerParameters;

   private final ArrayList<BehaviorInterface> childBehaviors;
   private final HandPoseBehavior moveHandToHomeBehavior;
   private final WalkToLocationBehavior walkToLocationBehavior;
   private WalkToLocationTask walkToValveTask;
   private final GraspTurnAndUnGraspValveBehavior graspValveTurnAndUnGraspBehavior;
   private final ComHeightBehavior comHeightBehavior;
   private final ScriptBehavior scriptBehavior;

   private final ConcurrentListeningQueue<ScriptBehaviorInputPacket> scriptBehaviorInputPacketListener;
   private final ConcurrentListeningQueue<TurnValvePacket> turnValvePacketListener;

   private final DoubleYoVariable yoTime;
   private final BooleanYoVariable tippingDetected;
   private final BooleanYoVariable hasInputBeenSet;

   private final double maxObservedWristForce = 0.0;
   private final double maxObservedCapturePointError = 0.0;

   private final double minDistanceForWalkingBetweenValveTurns = 0.1;
   private final double minYawDeltaForWalkingBetweenValveTurns = Math.toRadians(5.0);

   public TurnValveBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, SDFFullRobotModel fullRobotModel,
         HumanoidReferenceFrames referenceFrames, DoubleYoVariable yoTime, BooleanYoVariable yoDoubleSupport, BooleanYoVariable tippingDetectedBoolean,
         WholeBodyControllerParameters wholeBodyControllerParameters)
   {
      super(outgoingCommunicationBridge);
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;
      this.walkingControllerParameters = wholeBodyControllerParameters.getWalkingControllerParameters();

      childBehaviors = new ArrayList<BehaviorInterface>();
      moveHandToHomeBehavior = new HandPoseBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(moveHandToHomeBehavior);
      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, walkingControllerParameters);
      childBehaviors.add(walkToLocationBehavior);
      graspValveTurnAndUnGraspBehavior = new GraspTurnAndUnGraspValveBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames, yoTime,
            wholeBodyControllerParameters, tippingDetectedBoolean, USE_WHOLE_BODY_INVERSE_KINEMATICS);
      childBehaviors.add(graspValveTurnAndUnGraspBehavior);
      comHeightBehavior = new ComHeightBehavior(outgoingCommunicationBridge, yoTime);
      childBehaviors.add(comHeightBehavior);
      scriptBehavior = new ScriptBehavior(outgoingCommunicationBridge, fullRobotModel, yoTime, yoDoubleSupport, walkingControllerParameters);
      childBehaviors.add(scriptBehavior);

      scriptBehaviorInputPacketListener = new ConcurrentListeningQueue<ScriptBehaviorInputPacket>();
      turnValvePacketListener = new ConcurrentListeningQueue<TurnValvePacket>();

      this.tippingDetected = tippingDetectedBoolean;
      this.yoTime = yoTime;
      this.hasInputBeenSet = new BooleanYoVariable("hasInputBeenSet", registry);

      walkToLocationBehavior.setDistanceThreshold(minDistanceForWalkingBetweenValveTurns);
      walkToLocationBehavior.setYawAngleThreshold(minYawDeltaForWalkingBetweenValveTurns);

      super.attachNetworkProcessorListeningQueue(scriptBehaviorInputPacketListener, ScriptBehaviorInputPacket.class);
      super.attachNetworkProcessorListeningQueue(turnValvePacketListener, TurnValvePacket.class);
   }

   @Override
   public void doControl()
   {
      if (scriptBehaviorInputPacketListener.isNewPacketAvailable() && !hasInputBeenSet.getBooleanValue())
      {
         setInput(scriptBehaviorInputPacketListener.getNewestPacket());
      }

      if (turnValvePacketListener.isNewPacketAvailable() && !hasInputBeenSet.getBooleanValue())
      {
         setInput(turnValvePacketListener.getNewestPacket());
      }

      pipeLine.doControl();

      if (walkToValveTask != null && walkToValveTask.isDone())
      {
         PrintTools.debug(this, "PAUSING BECAUSE CAPTURE POINT ERROR EXCEEDS THRESHOLD");
         pauseIfCapturePointErrorIsTooLarge();
      }
   }

   private void pauseIfCapturePointErrorIsTooLarge()
   {
      if (tippingDetected.getBooleanValue() && !isPaused.getBooleanValue() && !isStopped.getBooleanValue())
      {
         this.pause();
         if (DEBUG)
            PrintTools.debug(this, "TurnValveBehavior: Tipping detected! Pausing behavior.");
      }
   }

   public void setInput(TurnValvePacket turnValvePacket)
   {
      double turnValveAngle = turnValvePacket.getTurnValveAngle();
      
      ValveGraspLocation valveGraspLocation;
      if (turnValveAngle > 0.0)
      {
         valveGraspLocation = ValveGraspLocation.TWELVE_O_CLOCK;
      }
      else
      {
         valveGraspLocation = ValveGraspLocation.SIX_O_CLOCK;
      }
      
      setInput(turnValvePacket.getValveTransformToWorld(), valveGraspLocation, ValveGraspMethod.RIM, turnValvePacket.getGraspApproachConeAngle(), Axis.X,
              turnValvePacket.getValveRadius(), turnValvePacket.getTurnValveAngle(), turnValvePacket.getValveRotationRate());
   }

   public void setInput(RigidBodyTransform valveTransformToWorld, ValveGraspLocation graspLocation, ValveGraspMethod graspMethod, double graspApproachConeAngle,
	         Axis valvePinJointAxisInValveFrame, double valveRadius, double totalAngleToRotateValve, double valveRotationRateRadPerSec)
   {
      PrintTools.debug(this, "Not using script behavior.");

      HandPoseTask moveHandToHomeTask = new HandPoseTask(robotSideOfHandToUse, PacketControllerTools.createGoToHomeHandPosePacket(robotSideOfHandToUse, 1.0),
            moveHandToHomeBehavior, yoTime);

      walkToValveTask = createWalkToValveTaskIfNecessary(valveTransformToWorld, 0.7 * walkingControllerParameters.getMaxStepLength());
      
      ArrayList<Task> graspTurnAndUngraspValveTasks = new ArrayList<>();
      int numberOfGraspUngraspCycles = (int) Math.ceil(Math.abs(totalAngleToRotateValve / MAX_ANGLE_TO_ROTATE_PER_GRASP_CYCLE));
      double totalAngleToRotateValveRemaining = totalAngleToRotateValve;
      
      for (int i = 0; i < numberOfGraspUngraspCycles; i++)
      {
         double angleToRotateOnThisGraspUngraspCycle = MathTools.clipToMinMax(totalAngleToRotateValveRemaining, MAX_ANGLE_TO_ROTATE_PER_GRASP_CYCLE);

         graspTurnAndUngraspValveTasks.add(new GraspTurnAndUnGraspValveTask(graspValveTurnAndUnGraspBehavior, valveTransformToWorld, graspLocation, graspMethod,
                 graspApproachConeAngle, valvePinJointAxisInValveFrame, valveRadius, angleToRotateOnThisGraspUngraspCycle, valveRotationRateRadPerSec,
                 STOP_HAND_IF_GRASP_COLLISION, STOP_HAND_IF_TURN_COLLISION, yoTime));
         
         totalAngleToRotateValveRemaining -= angleToRotateOnThisGraspUngraspCycle;
      }

      CoMHeightTask goToDefaultCoMHeightTask = new CoMHeightTask(0.0, yoTime, comHeightBehavior, 1.0);
      
//      pipeLine.submitSingleTaskStage(moveHandToHomeTask);  //FIXME: Go To Home HandPosePacket doesn't work at the moment...
      if (walkToValveTask != null)
         pipeLine.submitSingleTaskStage(walkToValveTask);
      pipeLine.submitAll(graspTurnAndUngraspValveTasks);
//      pipeLine.submitTaskForPallelPipesStage(moveHandToHomeBehavior, moveHandToHomeTask); //FIXME: Go To Home HandPosePacket doesn't work at the moment...
//      pipeLine.submitTaskForPallelPipesStage(comHeightBehavior, goToDefaultCoMHeightTask); //FIXME: Go To Home HandPosePacket doesn't work at the moment...
      pipeLine.submitSingleTaskStage(goToDefaultCoMHeightTask);

      hasInputBeenSet.set(true);
   }

   public void setInput(ScriptBehaviorInputPacket scriptBehaviorInputPacket)
   {
      RigidBodyTransform valveTransformToWorld = scriptBehaviorInputPacket.getReferenceTransform();

      PrintTools.debug(this, "New Script Behavior Input Packet Received.  Script File : " + scriptBehaviorInputPacket.getScriptName());

      HandPoseTask moveHandToHomeTask = new HandPoseTask(robotSideOfHandToUse, PacketControllerTools.createGoToHomeHandPosePacket(robotSideOfHandToUse, 1.0),
            moveHandToHomeBehavior, yoTime);

      walkToValveTask = createWalkToValveTaskIfNecessary(valveTransformToWorld, 0.7 * walkingControllerParameters.getMaxStepLength());

      ScriptTask turnValveTask = new ScriptTask(scriptBehavior, scriptBehaviorInputPacket, yoTime);

      pipeLine.submitSingleTaskStage(moveHandToHomeTask);
      if (walkToValveTask != null)
         pipeLine.submitSingleTaskStage(walkToValveTask);
      pipeLine.submitSingleTaskStage(turnValveTask);

      hasInputBeenSet.set(true);
   }

   private WalkToLocationTask createWalkToValveTaskIfNecessary(RigidBodyTransform valveTransformToWorld, double stepLength)
   {
      FramePose2d initialMidFeetZUpFramePose = getInitialRobotMidFeetZupPose(initialMidFeetPose);
      FramePose2d targetMidFeetZUpFramePose = computeDesiredWalkToLocation(valveTransformToWorld);

      double positionDistanceToValve = initialMidFeetZUpFramePose.getPositionDistance(targetMidFeetZUpFramePose);
      double orientationDistanceToValve = initialMidFeetZUpFramePose.getOrientationDistance(targetMidFeetZUpFramePose);

      boolean isItNecessaryToWalkToValve = Math.abs(positionDistanceToValve) > Math.abs(minDistanceForWalkingBetweenValveTurns)
            || Math.abs(orientationDistanceToValve) > Math.abs(minYawDeltaForWalkingBetweenValveTurns);

      if (!isItNecessaryToWalkToValve)
         return null;

      boolean reduceNumberOfStepsByWalkingNotAlignedWithWalkingPath = positionDistanceToValve < 4.0 * walkingControllerParameters.getMaxStepLength();
      WalkingOrientation walkingOrientation;
      if (reduceNumberOfStepsByWalkingNotAlignedWithWalkingPath)
      {
         walkingOrientation = WalkingOrientation.START_TARGET_ORIENTATION_MEAN;
      }
      else
      {
         walkingOrientation = WalkingOrientation.ALIGNED_WITH_PATH;
      }

      double sleepTimeBeforeNextTask = 1.0; // Add sleep time to prevent ICP fall detection from intervening during final transition into double support at the end of the walking task
      WalkToLocationTask ret = new WalkToLocationTask(targetMidFeetZUpFramePose, walkToLocationBehavior, walkingOrientation, stepLength, yoTime,
            sleepTimeBeforeNextTask);

      if (DEBUG)
      {
         PrintTools.debug(this, "initialMidFeetZUpFramePose: " + initialMidFeetZUpFramePose);
         PrintTools.debug(this, "targetMidFeetZUpFramePose: " + targetMidFeetZUpFramePose);
         PrintTools.debug(this, "positionDistanceToValve: " + positionDistanceToValve);
         PrintTools.debug(this, "orientationDistanceToValve: " + orientationDistanceToValve);
         PrintTools.debug(this, "walkingOrientation: " + walkingOrientation);
      }

      return ret;
   }

   private FramePose2d computeDesiredWalkToLocation(RigidBodyTransform valveTransformToWorld)
   {
      fullRobotModel.updateFrames();

      FramePose2d valvePose2d = new FramePose2d();
      valvePose2d.setPose(valveTransformToWorld);

      Pose2dReferenceFrame valveZUpFrame = new Pose2dReferenceFrame("valveZUp", valvePose2d);

      FramePose2d targetMidFeetZUpFramePose = new FramePose2d(valveZUpFrame);
      targetMidFeetZUpFramePose.setX(-howFarToStandBackFromValve);
      targetMidFeetZUpFramePose.setY(-howFarToStandToTheRightOfValve);
      targetMidFeetZUpFramePose.changeFrame(ReferenceFrame.getWorldFrame());

      return targetMidFeetZUpFramePose;
   }

   private FramePose2d getInitialRobotMidFeetZupPose(FramePose2d framePoseToPack)
   {
      fullRobotModel.updateFrames();
      framePoseToPack.setPose(referenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame());

      return framePoseToPack;
   }

   private final FramePose2d initialMidFeetPose = new FramePose2d();

   private double getWalkingDistanceToValve(FramePose2d targetMidFeetFramePose)
   {
      return getInitialRobotMidFeetZupPose(initialMidFeetPose).getPositionDistance(targetMidFeetFramePose);
   }

   private boolean isWalkingTargetBehindRobot(FramePose2d targetMidFeetFramePose)
   {
      targetMidFeetFramePose.changeFrame(referenceFrames.getMidFeetZUpFrame());
      double targetMidFeetXPositionInCurrentMidFeetFrame = targetMidFeetFramePose.getX();
      boolean ret = targetMidFeetXPositionInCurrentMidFeetFrame < 0.0;

      return ret;
   }

   private boolean isRobotAlreadyFacingValve(FramePose2d targetMidFeetFramePose)
   {
      double orientationDistance = targetMidFeetFramePose.getOrientationDistance(getInitialRobotMidFeetZupPose(initialMidFeetPose));
      return orientationDistance < Math.toRadians(5.0);
   }

   @Override
   protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {
      for (BehaviorInterface childBehavior : childBehaviors)
      {
         childBehavior.consumeObjectFromNetworkProcessor(object);
      }
   }

   @Override
   protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {
      for (BehaviorInterface childBehavior : childBehaviors)
      {
         childBehavior.consumeObjectFromController(object);
      }
   }

   @Override
   public void stop()
   {
      pipeLine.getCurrentStage().stop();
   }

   @Override
   public void enableActions()
   {
      //      PrintTools.debug(this, "Current Child Behavior: " + currentBehavior.getName());

      PrintTools.debug(this, "max wrist force : " + maxObservedWristForce);
      PrintTools.debug(this, "max capture point error : " + maxObservedCapturePointError);
   }

   @Override
   public void pause()
   {
      pipeLine.getCurrentStage().pause();
      isPaused.set(true);
   }

   @Override
   public void resume()
   {
      pipeLine.getCurrentStage().resume();
      isPaused.set(false);
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      for (BehaviorInterface childBehavior : childBehaviors)
      {
         childBehavior.doPostBehaviorCleanup();
      }

      hasInputBeenSet.set(false);
   }

   @Override
   public void initialize()
   {
      for (BehaviorInterface childBehavior : childBehaviors)
      {
         childBehavior.initialize();
      }
      hasInputBeenSet.set(false);
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return hasInputBeenSet.getBooleanValue();
   }
}
