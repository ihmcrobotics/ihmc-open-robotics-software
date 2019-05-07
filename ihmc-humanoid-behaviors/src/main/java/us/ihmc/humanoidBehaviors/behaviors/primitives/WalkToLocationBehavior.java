package us.ihmc.humanoidBehaviors.behaviors.primitives;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.SimplePathParameters;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.TurnStraightTurnFootstepGenerator;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.referenceFrames.Pose2dReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class WalkToLocationBehavior extends AbstractBehavior
{
   public enum WalkingOrientation
   {
      ALIGNED_WITH_PATH, START_ORIENTATION, TARGET_ORIENTATION, START_TARGET_ORIENTATION_MEAN, CUSTOM
   }

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final boolean DEBUG = false;
   private final FullRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;

   private double swingTime;
   private double transferTime;

   private final FramePose3D robotPose = new FramePose3D();
   private final Point3D robotLocation = new Point3D();
   private final Quaternion robotOrientation = new Quaternion();

   private final YoFramePoseUsingYawPitchRoll robotYoPose = new YoFramePoseUsingYawPitchRoll("robotYoPose", worldFrame, registry);

   private final YoBoolean hasTargetBeenProvided = new YoBoolean("hasTargetBeenProvided", registry);
   private final YoBoolean hasInputBeenSet = new YoBoolean("hasInputBeenSet", registry);
   private final YoBoolean haveFootstepsBeenGenerated = new YoBoolean("haveFootstepsBeenGenerated", registry);

   private final YoFramePoint3D targetLocation = new YoFramePoint3D(getName() + "TargetLocation", worldFrame, registry);
   private final YoFrameYawPitchRoll targetOrientation = new YoFrameYawPitchRoll(getName() + "TargetOrientation", worldFrame, registry);
   private final YoFrameVector3D walkPathVector = new YoFrameVector3D(getName(), worldFrame, registry);
   private final YoDouble walkDistance = new YoDouble(getName() + "WalkDistance", registry);

   private SimplePathParameters pathType;// = new SimplePathParameters(0.4, 0.30, 0.0, Math.toRadians(10.0), Math.toRadians(5.0), 0.4);

   private ArrayList<Footstep> footsteps = new ArrayList<Footstep>();
   private FootstepListBehavior footstepListBehavior;

   private final SideDependentList<RigidBodyBasics> feet = new SideDependentList<RigidBodyBasics>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<ReferenceFrame>();

   private double minDistanceThresholdForWalking, minYawThresholdForWalking;

   public WalkToLocationBehavior(String robotName, Ros2Node ros2Node, FullHumanoidRobotModel fullRobotModel,
                                 HumanoidReferenceFrames referenceFrames, WalkingControllerParameters walkingControllerParameters)
   {
      super(robotName, ros2Node);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;

      this.swingTime = walkingControllerParameters.getDefaultSwingTime();
      this.transferTime = walkingControllerParameters.getDefaultTransferTime();

      this.pathType = new SimplePathParameters(walkingControllerParameters.getSteppingParameters().getMaxStepLength() / 2,
                                               walkingControllerParameters.getSteppingParameters().getInPlaceWidth(), 0.0, Math.toRadians(20.0),
                                               Math.toRadians(10.0), 0.4); // 10 5 0.4
      footstepListBehavior = new FootstepListBehavior(robotName, ros2Node, walkingControllerParameters);

      for (RobotSide robotSide : RobotSide.values)
      {
         feet.put(robotSide, fullRobotModel.getFoot(robotSide));
         soleFrames.put(robotSide, fullRobotModel.getSoleFrame(robotSide));
      }

   }

   
   public void setTarget(FramePose2D targetPose2dInWorld)
   {
      setTarget(targetPose2dInWorld, WalkingOrientation.CUSTOM);
   }

   public void setTarget(FramePose2D targetPose2dInWorld, WalkingOrientation walkingOrientation)
   {
      targetPose2dInWorld.checkReferenceFrameMatch(worldFrame);
      this.targetLocation.set(targetPose2dInWorld.getX(), targetPose2dInWorld.getY(), 0.0);
      this.targetOrientation.setYawPitchRoll(targetPose2dInWorld.getYaw(), 0.0, 0.0);

      ReferenceFrame initialRobotFrame = referenceFrames.getPelvisZUpFrame();
      ReferenceFrame targetRobotFrame = new Pose2dReferenceFrame("targetFrame", targetPose2dInWorld);

      double initialRobotOrientationRelativeToWalkingPath = computeFrameOrientationRelativeToWalkingPath(initialRobotFrame);
      double targetRobotOrientationRelativeToWalkingPath = computeFrameOrientationRelativeToWalkingPath(targetRobotFrame);

      switch (walkingOrientation)
      {
      case ALIGNED_WITH_PATH:
         setWalkingOrientationRelativeToPathDirection(0.0);
         break;

      case START_ORIENTATION:
         setWalkingOrientationRelativeToPathDirection(initialRobotOrientationRelativeToWalkingPath);
         break;

      case TARGET_ORIENTATION:
         setWalkingOrientationRelativeToPathDirection(targetRobotOrientationRelativeToWalkingPath);
         break;

      case START_TARGET_ORIENTATION_MEAN:
         setWalkingOrientationRelativeToPathDirection(0.5 * (initialRobotOrientationRelativeToWalkingPath + targetRobotOrientationRelativeToWalkingPath));
         break;

      default:
         break;
      }

      hasTargetBeenProvided.set(true);
      haveFootstepsBeenGenerated.set(false);
      hasInputBeenSet.set(true);
   }

   private double computeFrameOrientationRelativeToWalkingPath(ReferenceFrame referenceFrame)
   {
      this.walkPathVector.sub(this.targetLocation, robotYoPose.getPosition());
      fullRobotModel.updateFrames();

      FrameVector2D frameHeadingVector = new FrameVector2D(referenceFrame, 1.0, 0.0);
      frameHeadingVector.changeFrame(worldFrame);
      double ret = -Math.abs(frameHeadingVector.angle(new FrameVector2D(walkPathVector)));

      if (DEBUG)
      {
         PrintTools.debug(this, "FrameHeadingVector : " + frameHeadingVector);
         PrintTools.debug(this, "WalkPathVector : " + walkPathVector);
         PrintTools.debug(this, "OrientationToWalkPath : " + ret);
      }

      return ret;
   }

   public void setSwingTime(double swingTime)
   {
      this.swingTime = swingTime;
   }

   public void setTransferTime(double transferTime)
   {
      this.transferTime = transferTime;
   }

   public void setWalkingOrientationRelativeToPathDirection(double orientationRelativeToPathDirection)
   {
      pathType.setAngle(orientationRelativeToPathDirection);
      if (hasTargetBeenProvided.getBooleanValue())
         generateFootsteps();
   }

  
   public int getNumberOfFootSteps()
   {
      return footsteps.size();
   }

   public ArrayList<Footstep> getFootSteps()
   {
      return footsteps;
   }

   private void generateFootsteps()
   {
      FramePoint3D midFeetPosition = getCurrentMidFeetPosition();

      footsteps.clear();
      FramePose2D endPose = new FramePose2D(worldFrame);
      endPose.setPosition(new FramePoint2D(worldFrame, targetLocation.getX(), targetLocation.getY()));
      endPose.setOrientation(new FrameOrientation2D(worldFrame, targetOrientation.getYaw()));

      boolean computeFootstepsWithFlippedInitialTurnDirection = pathType.getAngle() != 0.0;

      TurnStraightTurnFootstepGenerator footstepGenerator = new TurnStraightTurnFootstepGenerator(feet, soleFrames, endPose, pathType);
      footstepGenerator.initialize();

      walkDistance.set(footstepGenerator.getDistance());

      if (footstepGenerator.getDistance() > minDistanceThresholdForWalking
            || Math.abs(footstepGenerator.getSignedInitialTurnDirection()) > minYawThresholdForWalking)
      {
         List<Footstep> footstepsNominalOrientation = footstepGenerator.generateDesiredFootstepList();

         if (computeFootstepsWithFlippedInitialTurnDirection)
         {
            pathType.setAngle(-pathType.getAngle());
            TurnStraightTurnFootstepGenerator footstepGeneratorFlippedInitialTurnDirection = new TurnStraightTurnFootstepGenerator(feet, soleFrames, endPose,
                                                                                                                                   pathType); //FIXME: should be able to re-use other footStepGenerator, but doesn't work so far..
            footstepGeneratorFlippedInitialTurnDirection.initialize();
            List<Footstep> footstepsFlippedOrientation = footstepGeneratorFlippedInitialTurnDirection.generateDesiredFootstepList();
            pathType.setAngle(-pathType.getAngle());

            if (footstepsFlippedOrientation.size() < footstepsNominalOrientation.size())
            {
               footsteps.addAll(footstepsFlippedOrientation);
            }
            else
            {
               footsteps.addAll(footstepsNominalOrientation);
            }
         }
         else
         {
            footsteps.addAll(footstepsNominalOrientation);
         }
      }

      footstepListBehavior.set(footsteps, swingTime, transferTime);
      haveFootstepsBeenGenerated.set(true);

      if (DEBUG)
         PrintTools.debug(this, "Walk Distance: " + walkDistance.getDoubleValue());
   }

   @Override
   public void doControl()
   {
      if (!hasTargetBeenProvided.getBooleanValue())
         return;
      if (!haveFootstepsBeenGenerated.getBooleanValue())
         generateFootsteps();
      footstepListBehavior.doControl();
   }

   private FramePoint3D getCurrentMidFeetPosition()
   {
      FramePoint3D ret = new FramePoint3D();
      ret.setToZero(referenceFrames.getMidFeetZUpFrame());
      ret.changeFrame(worldFrame);

      return ret;
   }

   @Override
   public void onBehaviorAborted()
   {
      footstepListBehavior.onBehaviorAborted();
      isAborted.set(true);
   }

   @Override
   public void onBehaviorPaused()
   {
      footstepListBehavior.onBehaviorPaused();
      isPaused.set(true);
   }

   @Override
   public void onBehaviorResumed()
   {
      footstepListBehavior.onBehaviorResumed();
      isPaused.set(false);

   }

   @Override
   public boolean isDone()
   {
      if (!haveFootstepsBeenGenerated.getBooleanValue() || !hasTargetBeenProvided.getBooleanValue())
      {
         return false;
      }
      if (haveFootstepsBeenGenerated.getBooleanValue() && footsteps.size() == 0)
      {
         return true;
      }
      return footstepListBehavior.isDone();
   }

   @Override
   public void onBehaviorExited()
   {
      isPaused.set(false);
      isAborted.set(false);
      footstepListBehavior.onBehaviorExited();
   }
   
   @Override
   public void onBehaviorEntered()
   {
      hasTargetBeenProvided.set(false);
      haveFootstepsBeenGenerated.set(false);
      hasInputBeenSet.set(false);
      footstepListBehavior.initialize();

      robotPose.setToZero(fullRobotModel.getRootJoint().getFrameAfterJoint());
      robotPose.changeFrame(worldFrame);

      robotYoPose.set(robotPose);

      robotLocation.set(robotPose.getPosition());
      robotOrientation.set(robotPose.getOrientation());

      this.targetLocation.set(robotLocation);
      this.targetOrientation.set(robotOrientation);
   }


   public boolean hasInputBeenSet()
   {
      return (hasInputBeenSet.getBooleanValue());
   }

   public void setFootstepLength(double footstepLength)
   {
      pathType.setStepLength(footstepLength);
   }

   public void setDistanceThreshold(double minDistanceThresholdForWalking)
   {
      this.minDistanceThresholdForWalking = minDistanceThresholdForWalking;
   }

   public void setYawAngleThreshold(double minYawThresholdForWalking)
   {
      this.minYawThresholdForWalking = minYawThresholdForWalking;
   }

}
