package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.utilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;


public class WalkOnTheEdgesManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // TODO it would be nice to use toe touchdown for side steps, but it requires too often an unreachable orientation of the foot resulting in unstable behaviors
   private static final boolean DO_TOE_TOUCHDOWN_ONLY_WHEN_STEPPING_DOWN = true;
   private static final boolean DO_TOEOFF_FOR_SIDE_STEPS = false;
   private static final boolean ENABLE_TOE_OFF_FOR_STEP_DOWN = true;

   private final BooleanYoVariable doToeOffIfPossible = new BooleanYoVariable("doToeOffIfPossible", registry);
   private final BooleanYoVariable doToeOffWhenHittingAnkleLimit = new BooleanYoVariable("doToeOffWhenHittingAnkleLimit", registry);
   private final BooleanYoVariable doToeOff = new BooleanYoVariable("doToeOff", registry);
   private final DoubleYoVariable jacobianDeterminantThresholdForToeOff = new DoubleYoVariable("jacobianDeterminantThresholdForToeOff", registry);

   private final BooleanYoVariable doToeTouchdownIfPossible = new BooleanYoVariable("doToeTouchdownIfPossible", registry);
   private final BooleanYoVariable doToeTouchdown = new BooleanYoVariable("doToeTouchdown", registry);

   private final BooleanYoVariable doHeelTouchdownIfPossible = new BooleanYoVariable("doHeelTouchdownIfPossible", registry);
   private final BooleanYoVariable doHeelTouchdown = new BooleanYoVariable("doHeelTouchdown", registry);

   private final BooleanYoVariable isDesiredICPOKForToeOff = new BooleanYoVariable("isDesiredICPOKForToeOff", registry);
   private final BooleanYoVariable isCurrentICPOKForToeOff = new BooleanYoVariable("isCurrentICPOKForToeOff", registry);
   private final BooleanYoVariable isDesiredECMPOKForToeOff = new BooleanYoVariable("isDesiredECMPOKForToeOff", registry);

   private final DoubleYoVariable minStepLengthForToeOff = new DoubleYoVariable("minStepLengthForToeOff", registry);
   private final DoubleYoVariable minStepHeightForToeOff = new DoubleYoVariable("minStepHeightForToeOff", registry);
   private final DoubleYoVariable minStepLengthForToeTouchdown = new DoubleYoVariable("minStepLengthForToeTouchdown", registry);

   private final SideDependentList<? extends ContactablePlaneBody> feet;
   private final SideDependentList<FootControlModule> footEndEffectorControlModules;

   private final DoubleYoVariable extraCoMMaxHeightWithToes = new DoubleYoVariable("extraCoMMaxHeightWithToes", registry);

   private final FramePoint tempLeadingFootPosition = new FramePoint();
   private final FramePoint tempTrailingFootPosition = new FramePoint();
   private final FramePoint tempLeadingFootPositionInWorld = new FramePoint();
   private final FramePoint tempTrailingFootPositionInWorld = new FramePoint();

   private final WalkingControllerParameters walkingControllerParameters;

   private final SideDependentList<DoubleYoVariable> angleFootsWithDesired = new SideDependentList<DoubleYoVariable>(
         new DoubleYoVariable("angleFootWithDesiredLeft", registry),
         new DoubleYoVariable("angleFootWithDesiredRight", registry));
   private BooleanYoVariable enabledDoubleState = new BooleanYoVariable("enabledDoubleState", registry);

   private final SideDependentList<YoFrameOrientation> footOrientationInWorld = new SideDependentList<YoFrameOrientation>(
         new YoFrameOrientation("orientationLeftFoot", worldFrame, registry),
         new YoFrameOrientation("orientationRightFoot", worldFrame, registry));

   private final SideDependentList<BooleanYoVariable> desiredAngleReached = new SideDependentList<BooleanYoVariable>(
         new BooleanYoVariable("l_Desired_Pitch", registry),
         new BooleanYoVariable("r_Desired_Pitch", registry));
   private Footstep desiredFootstep;

   private final BooleanYoVariable isRearAnklePitchHittingLimit;
   private final GlitchFilteredBooleanYoVariable isRearAnklePitchHittingLimitFilt;

   private final FullRobotModel fullRobotModel;

   private final double inPlaceWidth;
   private final double footLength;

   public WalkOnTheEdgesManager(FullRobotModel fullRobotModel, WalkingControllerParameters walkingControllerParameters, SideDependentList<? extends ContactablePlaneBody> feet,
         SideDependentList<FootControlModule> footEndEffectorControlModules, YoVariableRegistry parentRegistry)
   {
      this.doToeOffIfPossible.set(walkingControllerParameters.doToeOffIfPossible());
      this.doToeOffWhenHittingAnkleLimit.set(walkingControllerParameters.doToeOffWhenHittingAnkleLimit());
      this.doToeTouchdownIfPossible.set(walkingControllerParameters.doToeTouchdownIfPossible());
      this.doHeelTouchdownIfPossible.set(walkingControllerParameters.doHeelTouchdownIfPossible());

      this.walkingControllerParameters = walkingControllerParameters;

      this.fullRobotModel = fullRobotModel;
      this.feet = feet;
      this.footEndEffectorControlModules = footEndEffectorControlModules;
      desiredFootstep = null;

      this.inPlaceWidth = walkingControllerParameters.getInPlaceWidth();
      this.footLength = walkingControllerParameters.getFootBackwardOffset() + walkingControllerParameters.getFootForwardOffset();

      extraCoMMaxHeightWithToes.set(0.08);

      minStepLengthForToeOff.set(walkingControllerParameters.getMinStepLengthForToeOff());
      minStepHeightForToeOff.set(0.10);
      jacobianDeterminantThresholdForToeOff.set(0.10);

      minStepLengthForToeTouchdown.set(0.40);

      isRearAnklePitchHittingLimit = new BooleanYoVariable("isRearAnklePitchHittingLimit", registry);
      isRearAnklePitchHittingLimitFilt = new GlitchFilteredBooleanYoVariable("isRearAnklePitchHittingLimitFilt", registry, isRearAnklePitchHittingLimit, 10);

      parentRegistry.addChild(registry);
   }

   public void updateToeOffStatus(RobotSide trailingLeg, FramePoint2d desiredECMP, FramePoint2d desiredICP, FramePoint2d currentICP)
   {
      if (!doToeOffIfPossible.getBooleanValue())
      {
         doToeOff.set(false);
         isDesiredECMPOKForToeOff.set(false);
         return;
      }

      ContactablePlaneBody trailingFoot = feet.get(trailingLeg);
      ContactablePlaneBody leadingFoot = feet.get(trailingLeg.getOppositeSide());
      if (walkingControllerParameters.checkECMPLocationToTriggerToeOff())
      {
         FrameConvexPolygon2d onToesSupportPolygon = getOnToesSupportPolygonCopy(trailingFoot, leadingFoot);
         isDesiredECMPOKForToeOff.set(onToesSupportPolygon.isPointInside(desiredECMP));
      }
      else
      {
         isDesiredECMPOKForToeOff.set(true);
      }

      FrameConvexPolygon2d leadingFootSupportPolygon = getFootSupportPolygonCopy(leadingFoot);
      isDesiredICPOKForToeOff.set(leadingFootSupportPolygon.isPointInside(desiredICP));
      isCurrentICPOKForToeOff.set(leadingFootSupportPolygon.isPointInside(currentICP));

      boolean needToSwitchToToeOffForAnkleLimit = checkAnkleLimitForToeOff(trailingLeg);
      if (needToSwitchToToeOffForAnkleLimit)
      {
         doToeOff.set(true);
         return;
      }

      if (!isDesiredECMPOKForToeOff.getBooleanValue())
      {
         doToeOff.set(false);
         return;
      }

      if (!isDesiredICPOKForToeOff.getBooleanValue() || !isCurrentICPOKForToeOff.getBooleanValue())
      {
         doToeOff.set(false);
         return;
      }

      isReadyToSwitchToToeOff(trailingLeg);
   }

   private boolean checkAnkleLimitForToeOff(RobotSide trailingLeg)
   {
      OneDoFJoint anklePitch = fullRobotModel.getLegJoint(trailingLeg, LegJointName.ANKLE_PITCH);
      isRearAnklePitchHittingLimit.set(Math.abs(anklePitch.getJointLimitLower() - anklePitch.getQ()) < 0.02);
      isRearAnklePitchHittingLimitFilt.update();

      if (!doToeOffWhenHittingAnkleLimit.getBooleanValue())
         return false;

      if (!isDesiredICPOKForToeOff.getBooleanValue() || !isCurrentICPOKForToeOff.getBooleanValue())
         return false;

      return isRearAnklePitchHittingLimitFilt.getBooleanValue();
   }

   private void isReadyToSwitchToToeOff(RobotSide trailingLeg)
   {
      RobotSide leadingLeg = trailingLeg.getOppositeSide();
      ReferenceFrame frontFootFrame = feet.get(leadingLeg).getFrameAfterParentJoint();

      if (!isFrontFootWellPositionedForToeOff(trailingLeg, frontFootFrame))
      {
         doToeOff.set(false);
         return;
      }

      FootControlModule rearFootControlModule = footEndEffectorControlModules.get(trailingLeg);
      doToeOff.set(Math.abs(rearFootControlModule.getJacobianDeterminant()) < jacobianDeterminantThresholdForToeOff.getDoubleValue());
   }

   private boolean isFrontFootWellPositionedForToeOff(RobotSide trailingLeg, ReferenceFrame frontFootFrame)
   {
      ReferenceFrame trailingFootFrame = feet.get(trailingLeg).getFrameAfterParentJoint();
      tempLeadingFootPosition.setToZero(frontFootFrame);
      tempTrailingFootPosition.setToZero(trailingFootFrame);
      tempLeadingFootPosition.changeFrame(trailingFootFrame);

      if (Math.abs(tempLeadingFootPosition.getY()) > inPlaceWidth)
         tempLeadingFootPosition.setY(tempLeadingFootPosition.getY() + trailingLeg.negateIfRightSide(inPlaceWidth));
      else
         tempLeadingFootPosition.setY(0.0);

      tempLeadingFootPositionInWorld.setToZero(frontFootFrame);
      tempTrailingFootPositionInWorld.setToZero(trailingFootFrame);
      tempLeadingFootPositionInWorld.changeFrame(worldFrame);
      tempTrailingFootPositionInWorld.changeFrame(worldFrame);

      double stepHeight = tempLeadingFootPositionInWorld.getZ() - tempTrailingFootPositionInWorld.getZ();

      boolean isNextStepHighEnough = stepHeight > minStepHeightForToeOff.getDoubleValue();
      if (isNextStepHighEnough)
         return true;

      if (!ENABLE_TOE_OFF_FOR_STEP_DOWN)
      {
         boolean isNextStepTooLow = stepHeight < -0.10;
         if (isNextStepTooLow)
            return false;
      }

      boolean isForwardOrSideStepping = tempLeadingFootPosition.getX() > -0.05;
      if (!isForwardOrSideStepping)
         return false;

      boolean isSideStepping = Math.abs(Math.atan2(tempLeadingFootPosition.getY(), tempLeadingFootPosition.getX())) > Math.toRadians(45.0);
      if (isSideStepping && !DO_TOEOFF_FOR_SIDE_STEPS)
         return false;

      boolean isStepLongEnough = tempLeadingFootPosition.distance(tempTrailingFootPosition) > minStepLengthForToeOff.getDoubleValue();
      boolean isStepLongEnoughAlongX = tempLeadingFootPosition.getX() > footLength;
      return isStepLongEnough && isStepLongEnoughAlongX;
   }

   private boolean isFrontFootWellPositionedForToeTouchdown(RobotSide trailingLeg, ReferenceFrame frontFootFrame)
   {
      ReferenceFrame trailingFootFrame = feet.get(trailingLeg).getFrameAfterParentJoint();
      tempLeadingFootPosition.setToZero(frontFootFrame);
      tempTrailingFootPosition.setToZero(trailingFootFrame);
      tempLeadingFootPosition.changeFrame(trailingFootFrame);

      tempLeadingFootPositionInWorld.setToZero(frontFootFrame);
      tempTrailingFootPositionInWorld.setToZero(trailingFootFrame);
      tempLeadingFootPositionInWorld.changeFrame(worldFrame);
      tempTrailingFootPositionInWorld.changeFrame(worldFrame);

      double stepHeight = tempLeadingFootPositionInWorld.getZ() - tempTrailingFootPositionInWorld.getZ();

      boolean isNextStepTooHigh = stepHeight > 0.05;
      if (isNextStepTooHigh)
         return false;

      boolean isNextStepLowEnough = stepHeight < -minStepHeightForToeOff.getDoubleValue();
      if (isNextStepLowEnough)
         return true;

      if (DO_TOE_TOUCHDOWN_ONLY_WHEN_STEPPING_DOWN)
         return false;

      boolean isBackardOrSideStepping = tempLeadingFootPosition.getX() < 0.05;
      if (!isBackardOrSideStepping)
         return false;

      boolean isStepLongEnough = tempLeadingFootPosition.distance(tempTrailingFootPosition) > minStepLengthForToeTouchdown.getDoubleValue();
      return isStepLongEnough;
   }

   private boolean isFrontFootWellPositionedForHeelTouchdown(RobotSide trailingLeg, ReferenceFrame frontFootFrame)
   {
      ReferenceFrame trailingFootFrame = feet.get(trailingLeg).getFrameAfterParentJoint();
      tempLeadingFootPosition.setToZero(frontFootFrame);
      tempTrailingFootPosition.setToZero(trailingFootFrame);
      tempLeadingFootPosition.changeFrame(trailingFootFrame);

      tempLeadingFootPositionInWorld.setToZero(frontFootFrame);
      tempTrailingFootPositionInWorld.setToZero(trailingFootFrame);
      tempLeadingFootPositionInWorld.changeFrame(worldFrame);
      tempTrailingFootPositionInWorld.changeFrame(worldFrame);

      boolean isBackardOrSideStepping = tempLeadingFootPosition.getX() < 0.15;
      if (isBackardOrSideStepping)
         return false;

      boolean isStepLongEnough = tempLeadingFootPosition.distance(tempTrailingFootPosition) > minStepLengthForToeTouchdown.getDoubleValue();
      return isStepLongEnough;
   }

   public void updateEdgeTouchdownStatus(RobotSide supportLeg, Footstep nextFootstep)
   {
      RobotSide nextTrailingLeg = supportLeg;
      ReferenceFrame nextFrontFootFrame;

      if (nextFootstep != null)
         nextFrontFootFrame = nextFootstep.getPoseReferenceFrame();
      else
         nextFrontFootFrame = feet.get(nextTrailingLeg.getOppositeSide()).getFrameAfterParentJoint();

      if (!doToeTouchdownIfPossible.getBooleanValue())
      {
         doToeTouchdown.set(false);
      }
      else
      {
         boolean frontFootWellPositionedForToeTouchdown = isFrontFootWellPositionedForToeTouchdown(nextTrailingLeg, nextFrontFootFrame);
         doToeTouchdown.set(frontFootWellPositionedForToeTouchdown);
      }

      if (!doHeelTouchdownIfPossible.getBooleanValue() || doToeTouchdown.getBooleanValue())
      {
         doHeelTouchdown.set(false);
         return;
      }

      boolean frontFootWellPositionedForHeelTouchdown = isFrontFootWellPositionedForHeelTouchdown(nextTrailingLeg, nextFrontFootFrame);
      doHeelTouchdown.set(frontFootWellPositionedForHeelTouchdown);
   }

   public void modifyFootstepForEdgeTouchdown(Footstep footstepToModify, double touchdownInitialPitch)
   {
      desiredFootstep = new Footstep(footstepToModify);
      if (!doToeTouchdown.getBooleanValue() && !doHeelTouchdown.getBooleanValue())
         return;

      FrameOrientation oldOrientation = new FrameOrientation();
      FrameOrientation newOrientation = new FrameOrientation();
      FramePoint newPosition = new FramePoint();

      footstepToModify.getPose(newPosition, oldOrientation);
      newOrientation.setIncludingFrame(oldOrientation);

      double[] yawPitchRoll = newOrientation.getYawPitchRoll();
      yawPitchRoll[1] += touchdownInitialPitch;
      newOrientation.setYawPitchRoll(yawPitchRoll);

      Vector3d ankleToEdge, edgeToAnkle;

      if (doToeTouchdown.getBooleanValue())
         ankleToEdge = new Vector3d(walkingControllerParameters.getFootForwardOffset(), 0.0, -walkingControllerParameters.getAnkleHeight());
      else
         ankleToEdge = new Vector3d(-walkingControllerParameters.getFootBackwardOffset(), 0.0, -walkingControllerParameters.getAnkleHeight());

      edgeToAnkle = new Vector3d(ankleToEdge);
      edgeToAnkle.negate();

      RigidBodyTransform tempTransform = new RigidBodyTransform();
      oldOrientation.getTransform3D(tempTransform);
      tempTransform.transform(ankleToEdge);
      newOrientation.getTransform3D(tempTransform);
      tempTransform.transform(edgeToAnkle);

      double newX = newPosition.getX() + ankleToEdge.x + edgeToAnkle.x;
      double newHeight = newPosition.getZ() + ankleToEdge.z + edgeToAnkle.z;

      newPosition.setX(newX);
      newPosition.setZ(newHeight);
      footstepToModify.setPose(newPosition, newOrientation);
   }

   public boolean willLandOnEdge()
   {
      if (!doToeTouchdownIfPossible.getBooleanValue() && !doHeelTouchdownIfPossible.getBooleanValue())
         return false;

      return doToeTouchdown.getBooleanValue() || doHeelTouchdown.getBooleanValue();
   }

   public boolean willLandOnToes()
   {
      if (!doToeTouchdownIfPossible.getBooleanValue())
         return false;

      return doToeTouchdown.getBooleanValue();
   }

   public boolean willLandOnHeel()
   {
      if (!doHeelTouchdownIfPossible.getBooleanValue())
         return false;

      return doHeelTouchdown.getBooleanValue();
   }

   public boolean willDoToeOff(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      if (!doToeOffIfPossible.getBooleanValue())
         return false;

      RobotSide nextTrailingLeg = transferToAndNextFootstepsData.getTransferToSide().getOppositeSide();
      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();
      ReferenceFrame nextFrontFootFrame;
      if (nextFootstep != null)
         nextFrontFootFrame = nextFootstep.getPoseReferenceFrame();
      else
         nextFrontFootFrame = feet.get(nextTrailingLeg.getOppositeSide()).getFrameAfterParentJoint();

      boolean frontFootWellPositionedForToeOff = isFrontFootWellPositionedForToeOff(nextTrailingLeg, nextFrontFootFrame);

      return frontFootWellPositionedForToeOff;
   }

   public boolean doToeOff()
   {
      return doToeOff.getBooleanValue();
   }

   public boolean doToeOffIfPossible()
   {
      return doToeOffIfPossible.getBooleanValue();
   }
   
   public void setDoToeOffIfPossible(boolean value)
   {
      doToeOffIfPossible.set(value);
   }

   public double getExtraCoMMaxHeightWithToes()
   {
      return extraCoMMaxHeightWithToes.getDoubleValue();
   }

   public void reset()
   {
      isDesiredECMPOKForToeOff.set(false);
      isDesiredICPOKForToeOff.set(false);

      doToeOff.set(false);
   }

   private List<FramePoint> getToePointsCopy(ContactablePlaneBody supportFoot)
   {
      FrameVector forward = new FrameVector(supportFoot.getSoleFrame(), 1.0, 0.0, 0.0);
      int nToePoints = 2;
      List<FramePoint> toePoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(supportFoot.getContactPointsCopy(), forward, nToePoints);

      return toePoints;
   }

   private FrameConvexPolygon2d getOnToesSupportPolygonCopy(ContactablePlaneBody trailingFoot, ContactablePlaneBody leadingFoot)
   {
      List<FramePoint> toePoints = getToePointsCopy(trailingFoot);
      FramePoint singleToePoint = FramePoint.average(toePoints);
      singleToePoint.changeFrame(worldFrame);
      List<FramePoint> leadingFootPoints = leadingFoot.getContactPointsCopy();

      List<FramePoint2d> allPoints = new ArrayList<FramePoint2d>();
      allPoints.add(singleToePoint.toFramePoint2d());

      for (int i = 0; i < leadingFootPoints.size(); i++)
      {
         FramePoint framePoint = leadingFootPoints.get(i);
         framePoint.changeFrame(worldFrame);
         allPoints.add(framePoint.toFramePoint2d());
      }
      return new FrameConvexPolygon2d(allPoints);
   }

   private FrameConvexPolygon2d getFootSupportPolygonCopy(ContactablePlaneBody foot)
   {
      List<FramePoint> footPoints = foot.getContactPointsCopy();

      List<FramePoint2d> allPoints = new ArrayList<FramePoint2d>();
      for (int i = 0; i < footPoints.size(); i++)
      {
         FramePoint framePoint = footPoints.get(i);
         framePoint.changeFrame(worldFrame);
         allPoints.add(framePoint.toFramePoint2d());
      }
      return new FrameConvexPolygon2d(allPoints);
   }

   public boolean isEdgeTouchDownDone(RobotSide robotSide)
   {
      if (!doToeTouchdownIfPossible.getBooleanValue() && !doHeelTouchdownIfPossible.getBooleanValue())
         return true;

      if (!doToeTouchdown.getBooleanValue() && !doHeelTouchdown.getBooleanValue())
         return true;

      if (desiredFootstep != null)
      {
         desiredAngleReached.get(robotSide).set(false);
         FrameOrientation footFrameOrientation = new FrameOrientation(feet.get(robotSide).getFrameAfterParentJoint());
         footFrameOrientation.changeFrame(worldFrame);
         footOrientationInWorld.get(robotSide).set(footFrameOrientation);
         FrameOrientation desiredOrientation = new FrameOrientation(desiredFootstep.getPoseReferenceFrame());
         desiredOrientation.changeFrame(worldFrame);

         double pitchDifference = footFrameOrientation.getYawPitchRoll()[1] - desiredOrientation.getYawPitchRoll()[1];
         double rollDifference = footFrameOrientation.getYawPitchRoll()[2] - desiredOrientation.getYawPitchRoll()[2];

         angleFootsWithDesired.get(robotSide).set(pitchDifference);

         if (Math.abs(pitchDifference) < 0.1 && Math.abs(rollDifference) < 0.1)
         {
            desiredAngleReached.get(robotSide).set(true);
            enabledDoubleState.set(true);
         }
         else
         {
            enabledDoubleState.set(false);
         }
      }
      return enabledDoubleState.getBooleanValue();
   }
}