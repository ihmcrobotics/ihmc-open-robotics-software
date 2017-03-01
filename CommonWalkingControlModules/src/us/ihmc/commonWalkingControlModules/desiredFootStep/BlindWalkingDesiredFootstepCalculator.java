package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.BlindWalkingDirection;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class BlindWalkingDesiredFootstepCalculator extends AbstractDesiredFootstepCalculator
{
   private static final double DISTANCE_TO_DESTINATION_FOR_STEP_IN_PLACE = 0.2;

   private final YoFramePoint2d desiredDestination = new YoFramePoint2d("desiredDestination", "", worldFrame, registry);

   private final EnumYoVariable<BlindWalkingDirection> blindWalkingDirection = new EnumYoVariable<BlindWalkingDirection>("blindWalkingDirection", "", registry,
         BlindWalkingDirection.class, false);

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

   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ZUpFrame> soleZUpFrames = new SideDependentList<>();

   public BlindWalkingDesiredFootstepCalculator(SideDependentList<? extends ContactablePlaneBody> contactableBodies, YoVariableRegistry parentRegistry)
   {
      super(parentRegistry);

      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = contactableBodies.get(robotSide).getSoleFrame();
         soleFrames.put(robotSide, soleFrame);
         soleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, soleFrame, "soleZUpFrame"));
      }

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

   @Override
   public void initializeDesiredFootstep(RobotSide supportLegSide, double stepDuration)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();
      ZUpFrame supportZUpFrame = soleZUpFrames.get(supportLegSide);
      supportZUpFrame.update();
      ReferenceFrame supportFrame = soleFrames.get(supportLegSide);

      computeDistanceAndAngleToDestination(supportZUpFrame, swingLegSide, desiredDestination.getFramePoint2dCopy());

      if (distanceToDestination.getDoubleValue() < DISTANCE_TO_DESTINATION_FOR_STEP_IN_PLACE)
      {
         numberBlindStepsInPlace.increment();
      }

      FrameOrientation footOrientation = computeDesiredFootRotation(angleToDestination.getDoubleValue(), swingLegSide, supportFrame);
      FramePoint footstepPosition = getDesiredFootstepPositionCopy(supportZUpFrame, supportFrame, swingLegSide);

      setYoVariables(swingLegSide, footOrientation, footstepPosition);
   }

   @Override
   public FootstepDataMessage predictFootstepAfterDesiredFootstep(RobotSide supportLegSide, FootstepDataMessage desiredFootstep, double timeFromNow,
         double stepDuration)
   {
      RobotSide futureSwingLegSide = supportLegSide;
      PoseReferenceFrame futureSupportFrame = new PoseReferenceFrame("futureSupportFrame", worldFrame);
      futureSupportFrame.setPoseAndUpdate(desiredFootstep.getLocation(), desiredFootstep.getOrientation());
      ReferenceFrame futureSupportZUpFrame = new ZUpFrame(worldFrame, futureSupportFrame, "supportZUp");
      futureSupportZUpFrame.update();

      computeDistanceAndAngleToDestination(futureSupportZUpFrame, futureSwingLegSide, desiredDestination.getFramePoint2dCopy());
      FrameOrientation footOrientation = computeDesiredFootRotation(angleToDestination.getDoubleValue(), futureSwingLegSide, futureSupportZUpFrame);
      FramePoint footstepPosition = getDesiredFootstepPositionCopy(futureSupportZUpFrame, futureSupportFrame, futureSwingLegSide);

      FootstepDataMessage predictedFootstep = new FootstepDataMessage();
      predictedFootstep.setRobotSide(futureSwingLegSide);
      predictedFootstep.setLocation(footstepPosition.getPoint());
      predictedFootstep.setOrientation(footOrientation.getQuaternion());
      return predictedFootstep;
   }

   private FramePoint getDesiredFootstepPositionCopy(ReferenceFrame supportAnkleZUpFrame, ReferenceFrame supportAnkleFrame, RobotSide swingLegSide)
   {
      FrameVector2d desiredOffsetFromAnkle = computeDesiredOffsetFromSupportAnkle(supportAnkleZUpFrame, swingLegSide, angleToDestination.getDoubleValue(),
            distanceToDestination.getDoubleValue());
      FramePoint footstepPosition = computeDesiredFootPosition(supportAnkleFrame, desiredOffsetFromAnkle);
      footstepPosition.changeFrame(worldFrame);

      return footstepPosition;
   }

   private void computeDistanceAndAngleToDestination(ReferenceFrame supportAnkleZUpFrame, RobotSide swingLegSide, FramePoint2d desiredDestination)
   {
      FramePoint2d destinationInAnkleFrame = new FramePoint2d(desiredDestination);
      destinationInAnkleFrame.changeFrame(supportAnkleZUpFrame);

      FramePoint2d squaredUpMidpointInAnkleFrame = new FramePoint2d(supportAnkleZUpFrame, 0.0,
            swingLegSide.negateIfRightSide(desiredStepWidth.getDoubleValue() / 2.0));

      FrameVector2d midpointToDestination = new FrameVector2d(destinationInAnkleFrame);
      midpointToDestination.sub(squaredUpMidpointInAnkleFrame);

      distanceToDestination.set(midpointToDestination.length());
      angleToDestination.set(Math.atan2(midpointToDestination.getY(), midpointToDestination.getX()));

   }

   private final Vector2D desiredOffsetFromSquaredUp = new Vector2D();

   private FrameVector2d computeDesiredOffsetFromSupportAnkle(ReferenceFrame supportAnkleZUpFrame, RobotSide swingLegSide, double angleToDestination,
         double distanceToDestination)
   {
      if (distanceToDestination < DISTANCE_TO_DESTINATION_FOR_STEP_IN_PLACE)
      {
         return new FrameVector2d(supportAnkleZUpFrame, 0.0, swingLegSide.negateIfRightSide(desiredStepWidth.getDoubleValue()));
      }

      double absoluteAngleToDestination;
      switch (blindWalkingDirection.getEnumValue())
      {
      case BACKWARD:
      {
         absoluteAngleToDestination = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(angleToDestination, Math.PI));
         break;
      }
      case LEFT:
      {
         absoluteAngleToDestination = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(angleToDestination, Math.PI / 2.0));
         break;
      }
      case RIGHT:
      {
         absoluteAngleToDestination = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(angleToDestination, -Math.PI / 2.0));
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

      double percentToStepInXY = 1.0 - (absoluteAngleToDestination - minAngleBeforeShorterSteps) / (maxAngleBeforeTurnInPlace - minAngleBeforeShorterSteps);

      if (percentToStepInXY > 1.0)
         percentToStepInXY = 1.0;
      if (percentToStepInXY < 0.0)
         percentToStepInXY = 0.0;

      switch (blindWalkingDirection.getEnumValue())
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
      if (maxDistanceToAllow < 0.0)
         maxDistanceToAllow = 0.0;

      if (stepLength > maxDistanceToAllow)
      {
         desiredOffsetFromSquaredUp.scale(maxDistanceToAllow / stepLength);
         stepLength = desiredOffsetFromSquaredUp.length();
      }

      if (stepLength > maxStepLength.getDoubleValue())
      {
         desiredOffsetFromSquaredUp.scale(maxStepLength.getDoubleValue() / stepLength);
      }

      FrameVector2d desiredOffsetFromAnkle = new FrameVector2d(supportAnkleZUpFrame, desiredOffsetFromSquaredUp.getX(),
            desiredOffsetFromSquaredUp.getY() + swingLegSide.negateIfRightSide(desiredStepWidth.getDoubleValue()));

      if (swingLegSide == RobotSide.LEFT)
      {
         desiredOffsetFromAnkle.setY(MathTools.clamp(desiredOffsetFromAnkle.getY(), minStepWidth.getDoubleValue(), maxStepWidth.getDoubleValue()));
      }
      else
      {
         desiredOffsetFromAnkle.setY(MathTools.clamp(desiredOffsetFromAnkle.getY(), -maxStepWidth.getDoubleValue(), -minStepWidth.getDoubleValue()));
      }

      return desiredOffsetFromAnkle;
   }

   private FrameOrientation computeDesiredFootRotation(double angleToDestination, RobotSide swingLegSide, ReferenceFrame supportFootFrame)
   {
      RigidBodyTransform supportFootToWorldTransform = supportFootFrame.getTransformToDesiredFrame(worldFrame);
      RotationMatrix supportFootToWorldRotation = new RotationMatrix();
      supportFootToWorldTransform.getRotation(supportFootToWorldRotation);

      double maxTurnInAngle = 0.25;
      double maxTurnOutAngle = 0.4;

      double amountToYaw;
      switch (blindWalkingDirection.getEnumValue())
      {
      case BACKWARD:
      {
         amountToYaw = AngleTools.computeAngleDifferenceMinusPiToPi(angleToDestination, Math.PI);
         break;
      }
      case LEFT:
      {
         amountToYaw = AngleTools.computeAngleDifferenceMinusPiToPi(angleToDestination, Math.PI / 2.0);
         break;
      }
      case RIGHT:
      {
         amountToYaw = AngleTools.computeAngleDifferenceMinusPiToPi(angleToDestination, -Math.PI / 2.0);
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
         amountToYaw = MathTools.clamp(amountToYaw, -maxTurnInAngle, maxTurnOutAngle);

      }
      else
      {
         amountToYaw = MathTools.clamp(amountToYaw, -maxTurnOutAngle, maxTurnInAngle);
      }

      RotationMatrix yawRotation = new RotationMatrix();
      yawRotation.setToYawMatrix(amountToYaw);

      RotationMatrix ret = new RotationMatrix();
      ret.set(yawRotation);
      ret.multiply(supportFootToWorldRotation);

      return new FrameOrientation(worldFrame, ret);
   }

   private FramePoint computeDesiredFootPosition(ReferenceFrame upcomingSupportFrame, FrameVector2d desiredOffsetFromSupport)
   {
      desiredOffsetFromSupport.changeFrame(upcomingSupportFrame);
      FramePoint footstepPosition = new FramePoint(upcomingSupportFrame, desiredOffsetFromSupport.getX(), desiredOffsetFromSupport.getY(), 0.0);
      footstepPosition.changeFrame(worldFrame);

      return footstepPosition;
   }

   private void setYoVariables(RobotSide swingLegSide, FrameOrientation footstepOrientation, FramePoint footstepPosition)
   {
      footstepOrientations.get(swingLegSide).set(footstepOrientation);
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

   @Override
   public boolean isDone()
   {
      return numberBlindStepsInPlace.getIntegerValue() >= 2;
   }
}
