package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.BlindWalkingDirection;
import us.ihmc.humanoidRobotics.footstep.Footstep;


public class BlindWalkingToDestinationDesiredFootstepCalculator extends AbstractAdjustableDesiredFootstepCalculator
{
   private static final double DISTANCE_TO_DESTINATION_FOR_STEP_IN_PLACE = 0.2;

   private final YoFramePoint2d desiredDestination = new YoFramePoint2d("desiredDestination", "", ReferenceFrame.getWorldFrame(), registry);

   private final EnumYoVariable<BlindWalkingDirection> blindWalkingDirection = new EnumYoVariable<BlindWalkingDirection>("blindWalkingDirection", "", registry, BlindWalkingDirection.class, false);

   private final DoubleYoVariable distanceToDestination = new DoubleYoVariable("distanceToDestination", registry);
   private final DoubleYoVariable angleToDestination = new DoubleYoVariable("angleToDestination", registry);

   private final DoubleYoVariable desiredStepWidth = new DoubleYoVariable("desiredStepWidth", registry);
   private final DoubleYoVariable desiredStepForward = new DoubleYoVariable("desiredStepForward", registry);
   private final DoubleYoVariable desiredStepSideward = new DoubleYoVariable("desiredStepSideward", registry);
   private final DoubleYoVariable maxStepLength = new DoubleYoVariable("maxStepLength", registry);

   private final DoubleYoVariable minStepWidth = new DoubleYoVariable("minStepWidth", registry);
   private final DoubleYoVariable maxStepWidth = new DoubleYoVariable("maxStepWidth", registry);

   private final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);
   private final IntegerYoVariable numberBlindStepsInPlace = new IntegerYoVariable("numberBlindStepsInPlace", registry);

   private SideDependentList<? extends ReferenceFrame> ankleZUpFrames;
   private SideDependentList<? extends ReferenceFrame> ankleFrames;

   public BlindWalkingToDestinationDesiredFootstepCalculator(SideDependentList<? extends ReferenceFrame> ankleZUpFrames,
           SideDependentList<? extends ReferenceFrame> ankleFrames, SideDependentList<? extends ContactablePlaneBody> bipedFeet,
           YoVariableRegistry parentRegistry)
   {
      super(bipedFeet, getFramesToStoreFootstepsIn(), parentRegistry);

      this.ankleZUpFrames = ankleZUpFrames;
      this.ankleFrames = ankleFrames;
   }

   public Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
      return super.updateAndGetDesiredFootstep(supportLegSide);   
   }
   
   public void setBlindWalkingDirection(BlindWalkingDirection blindWalkingDirection)
   {
      this.blindWalkingDirection.set(blindWalkingDirection);
   }
   
   public void setDesiredDestination(FramePoint2d desiredDestinationInWorld)
   {
      numberBlindStepsInPlace.set(0);
      this.desiredDestination.set(desiredDestinationInWorld);
   }
   
   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {      
      RobotSide swingLegSide = supportLegSide.getOppositeSide();
      ReferenceFrame supportAnkleZUpFrame = ankleZUpFrames.get(supportLegSide);
      ReferenceFrame supportAnkleFrame = ankleFrames.get(supportLegSide);

      computeDistanceAndAngleToDestination(supportAnkleZUpFrame, swingLegSide, desiredDestination.getFramePoint2dCopy());

      if (distanceToDestination.getDoubleValue() < DISTANCE_TO_DESTINATION_FOR_STEP_IN_PLACE) 
      {
         numberBlindStepsInPlace.increment();
      }
      
      Matrix3d footToWorldRotation = computeDesiredFootRotation(angleToDestination.getDoubleValue(), swingLegSide, supportAnkleFrame);
      FramePoint footstepPosition = getDesiredFootstepPositionCopy(supportAnkleZUpFrame, supportAnkleFrame, swingLegSide, desiredDestination.getFramePoint2dCopy(), footToWorldRotation);

      setYoVariables(swingLegSide, footToWorldRotation, footstepPosition);
   }

   @Override
   public Footstep predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, Footstep desiredFootstep)
   {
      RobotSide futureSwingLegSide = supportLegSide;
      ReferenceFrame futureSupportAnkleFrame = desiredFootstep.getPoseReferenceFrame();

      ReferenceFrame futureSupportAnkleZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), futureSupportAnkleFrame, "ankleZUp");
      futureSupportAnkleZUpFrame.update();
      
      computeDistanceAndAngleToDestination(futureSupportAnkleZUpFrame, futureSwingLegSide, desiredDestination.getFramePoint2dCopy());
      Matrix3d footToWorldRotation = computeDesiredFootRotation(angleToDestination.getDoubleValue(), futureSwingLegSide, futureSupportAnkleFrame);

      FramePoint footstepPosition = getDesiredFootstepPositionCopy(futureSupportAnkleZUpFrame, futureSupportAnkleFrame, futureSwingLegSide, desiredDestination.getFramePoint2dCopy(), footToWorldRotation);
      FrameOrientation footstepOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
      double[] yawPitchRoll = new double[3];
      RotationFunctions.getYawPitchRoll(yawPitchRoll, footToWorldRotation);
      footstepOrientation.setYawPitchRoll(yawPitchRoll);

      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("poseReferenceFrame", footstepPose);

      ContactablePlaneBody foot = contactableBodies.get(futureSwingLegSide);
      boolean trustHeight = true;
      return new Footstep(foot.getRigidBody(), futureSwingLegSide, foot.getSoleFrame(), poseReferenceFrame, trustHeight);
   }

   private FramePoint getDesiredFootstepPositionCopy(ReferenceFrame supportAnkleZUpFrame, ReferenceFrame supportAnkleFrame, RobotSide swingLegSide, FramePoint2d desiredDestination,
           Matrix3d footToWorldRotation)
   {
      FrameVector2d desiredOffsetFromAnkle = computeDesiredOffsetFromSupportAnkle(supportAnkleZUpFrame, swingLegSide, angleToDestination.getDoubleValue(), distanceToDestination.getDoubleValue());       
      FramePoint footstepPosition = computeDesiredFootPosition(swingLegSide, supportAnkleZUpFrame, supportAnkleFrame, desiredOffsetFromAnkle, footToWorldRotation);
      footstepPosition.changeFrame(ReferenceFrame.getWorldFrame());

      return footstepPosition;
   }

   
   private void computeDistanceAndAngleToDestination(ReferenceFrame supportAnkleZUpFrame, RobotSide swingLegSide, FramePoint2d desiredDestination)
   {
      FramePoint2d destinationInAnkleFrame = new FramePoint2d(desiredDestination);
      destinationInAnkleFrame.changeFrame(supportAnkleZUpFrame);
       
      FramePoint2d squaredUpMidpointInAnkleFrame = new FramePoint2d(supportAnkleZUpFrame, 0.0, swingLegSide.negateIfRightSide(desiredStepWidth.getDoubleValue()/2.0));

      FrameVector2d midpointToDestination = new FrameVector2d(destinationInAnkleFrame);
      midpointToDestination.sub(squaredUpMidpointInAnkleFrame);
      
      distanceToDestination.set(midpointToDestination.length());
      angleToDestination.set(Math.atan2(midpointToDestination.getY(), midpointToDestination.getX()));
         
   }
   
   private final Vector2d desiredOffsetFromSquaredUp = new Vector2d();
   private FrameVector2d computeDesiredOffsetFromSupportAnkle(ReferenceFrame supportAnkleZUpFrame, RobotSide swingLegSide, double angleToDestination, double distanceToDestination)
   {   		
      if (distanceToDestination < DISTANCE_TO_DESTINATION_FOR_STEP_IN_PLACE) 
      {
         return new FrameVector2d(supportAnkleZUpFrame, 0.0, swingLegSide.negateIfRightSide(desiredStepWidth.getDoubleValue()));
      }
      
      double absoluteAngleToDestination;
      switch(blindWalkingDirection.getEnumValue())
      {
      case BACKWARD:
      {
         absoluteAngleToDestination = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(angleToDestination, Math.PI));
         break;
      }
      case LEFT:
      {
         absoluteAngleToDestination = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(angleToDestination, Math.PI/2.0));
         break;
      }
      case RIGHT:
      {
         absoluteAngleToDestination = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(angleToDestination, -Math.PI/2.0));
         break;
      }
      case FORWARD:
      {
         absoluteAngleToDestination = Math.abs(angleToDestination);
         break;
      }
      default:
      {
         throw new RuntimeException("Shouldn't get here!");
      }
      }
     
      double minAngleBeforeShorterSteps = 0.1;
      double maxAngleBeforeTurnInPlace = 0.5;
      
      double percentToStepInXY = 1.0 - (absoluteAngleToDestination - minAngleBeforeShorterSteps)/(maxAngleBeforeTurnInPlace - minAngleBeforeShorterSteps);
      
      if (percentToStepInXY > 1.0) percentToStepInXY = 1.0;
      if (percentToStepInXY < 0.0) percentToStepInXY = 0.0;
           
      
      switch(blindWalkingDirection.getEnumValue())
      {
      case BACKWARD:
      {
         double backwardsDistanceReduction = 0.75;
         desiredOffsetFromSquaredUp.set(-backwardsDistanceReduction * percentToStepInXY * desiredStepForward.getDoubleValue(), 0.0);
         break;
      }
      case LEFT:
      {
         desiredOffsetFromSquaredUp.set(0.0, percentToStepInXY * desiredStepSideward.getDoubleValue());
         break;
      }
      case RIGHT:
      {
         desiredOffsetFromSquaredUp.set(0.0, -percentToStepInXY * desiredStepSideward.getDoubleValue());
         break;
      }
      case FORWARD:
      {
         desiredOffsetFromSquaredUp.set(percentToStepInXY * desiredStepForward.getDoubleValue(), 0.0);
         break;
      }
      default:
      {
         throw new RuntimeException("Shouldn't get here!");
      }
      }
     
      double stepLength = desiredOffsetFromSquaredUp.length();
      
      double maxDistanceToAllow = distanceToDestination - 0.5 * DISTANCE_TO_DESTINATION_FOR_STEP_IN_PLACE;
      if (maxDistanceToAllow < 0.0) maxDistanceToAllow = 0.0;
      
      if (stepLength > maxDistanceToAllow)
      {
         desiredOffsetFromSquaredUp.scale(maxDistanceToAllow / stepLength);
         stepLength = desiredOffsetFromSquaredUp.length();
      }
      
      if (stepLength > maxStepLength.getDoubleValue())
      {
         desiredOffsetFromSquaredUp.scale(maxStepLength.getDoubleValue() / stepLength);
      }
      
      FrameVector2d desiredOffsetFromAnkle = new FrameVector2d(supportAnkleZUpFrame, desiredOffsetFromSquaredUp.getX(), desiredOffsetFromSquaredUp.getY() + swingLegSide.negateIfRightSide(desiredStepWidth.getDoubleValue()));

      if (swingLegSide == RobotSide.LEFT)
      {
         desiredOffsetFromAnkle.setY(MathTools.clipToMinMax(desiredOffsetFromAnkle.getY(), minStepWidth.getDoubleValue(), maxStepWidth.getDoubleValue()));
      }
      else
      {
         desiredOffsetFromAnkle.setY(MathTools.clipToMinMax(desiredOffsetFromAnkle.getY(), -maxStepWidth.getDoubleValue(), -minStepWidth.getDoubleValue()));
      }

      

      return desiredOffsetFromAnkle;
   }

   private Matrix3d computeDesiredFootRotation(double angleToDestination, RobotSide swingLegSide, ReferenceFrame supportFootFrame)
   {  
      RigidBodyTransform supportFootToWorldTransform = supportFootFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
      Matrix3d supportFootToWorldRotation = new Matrix3d();
      supportFootToWorldTransform.get(supportFootToWorldRotation);
      
      double maxTurnInAngle = 0.25;
      double maxTurnOutAngle = 0.4;
      
      double amountToYaw;
      switch(blindWalkingDirection.getEnumValue())
      {
      case BACKWARD:
      {
         amountToYaw = AngleTools.computeAngleDifferenceMinusPiToPi(angleToDestination, Math.PI);
         break;
      }
      case LEFT:
      {
         amountToYaw = AngleTools.computeAngleDifferenceMinusPiToPi(angleToDestination, Math.PI/2.0);
         break;
      }
      case RIGHT:
      {
         amountToYaw = AngleTools.computeAngleDifferenceMinusPiToPi(angleToDestination, -Math.PI/2.0);
         break;
      }
      case FORWARD:
      {
         amountToYaw = angleToDestination;
         break;
      }
      default:
      {
         throw new RuntimeException("Shouldn't get here!");
      }
      }
      
      if (swingLegSide == RobotSide.LEFT)
      {
         amountToYaw = MathTools.clipToMinMax(amountToYaw, -maxTurnInAngle, maxTurnOutAngle);

      }
      else
      {
         amountToYaw = MathTools.clipToMinMax(amountToYaw, -maxTurnOutAngle, maxTurnInAngle);
      }
      
      Matrix3d yawRotation = new Matrix3d();
      yawRotation.rotZ(amountToYaw);
      
      Matrix3d ret = new Matrix3d();
      ret.mul(yawRotation, supportFootToWorldRotation);

      return ret;
   }


   private FramePoint computeDesiredFootPosition(RobotSide upcomingSwingLegSide, ReferenceFrame upcomingSupportAnkleZUpFrame, ReferenceFrame upcomingSupportAnkleFrame,
           FrameVector2d desiredOffsetFromAnkle, Matrix3d swingFootToWorldRotation)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      desiredOffsetFromAnkle.changeFrame(upcomingSupportAnkleFrame);
      FramePoint footstepPosition = new FramePoint(upcomingSupportAnkleFrame, desiredOffsetFromAnkle.getX(), desiredOffsetFromAnkle.getY(), 0.0);
      footstepPosition.changeFrame(worldFrame);

      return footstepPosition;
   }

   private void setYoVariables(RobotSide swingLegSide, Matrix3d rotation, FramePoint footstepPosition)
   {
      footstepOrientations.get(swingLegSide).set(rotation);
      footstepPositions.get(swingLegSide).set(footstepPosition);
   }

   public void setDesiredStepWidth(double desiredStepWidth)
   {
      this.desiredStepWidth.set(desiredStepWidth);
   }

   public void setMaxStepLength(double maxStepLength)
   {
      this.maxStepLength.set(maxStepLength);
   }
   
   public void setDesiredStepForward(double desiredStepForward)
   {
      this.desiredStepForward.set(desiredStepForward);
   }
   
   public void setDesiredStepSideward(double desiredStepSideward)
   {
      this.desiredStepSideward.set(desiredStepSideward);
   }

   public void setMinStepWidth(double minStepWidth)
   {
      this.minStepWidth.set(minStepWidth);
   }

   public void setMaxStepWidth(double maxStepWidth)
   {
      this.maxStepWidth.set(maxStepWidth);
   }

   public void setStepPitch(double stepPitch)
   {
      this.stepPitch.set(stepPitch);
   }

   private static SideDependentList<ReferenceFrame> getFramesToStoreFootstepsIn()
   {
      return new SideDependentList<ReferenceFrame>(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   protected List<FramePoint> getContactPoints(RobotSide swingSide)
   {
      double stepPitch = this.stepPitch.getDoubleValue();
      List<FramePoint> allContactPoints = contactableBodies.get(swingSide).getContactPointsCopy();
      if (stepPitch == 0.0)
      {
         return allContactPoints;
      }
      else
      {
         FrameVector forwardInFootFrame = new FrameVector(contactableBodies.get(swingSide).getFrameAfterParentJoint());
         ReferenceFrame frame = allContactPoints.get(0).getReferenceFrame();
         forwardInFootFrame.changeFrame(frame);
         forwardInFootFrame.scale(Math.signum(stepPitch));
         int nPoints = 2;

         return DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(allContactPoints, forwardInFootFrame, nPoints);
      }
   }

   public boolean isDone()
   {
      return (numberBlindStepsInPlace.getIntegerValue() >= 2);
   }
}
