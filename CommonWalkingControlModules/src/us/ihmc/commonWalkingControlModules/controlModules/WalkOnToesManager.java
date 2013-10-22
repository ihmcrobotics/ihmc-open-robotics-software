package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.endEffector.EndEffectorControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class WalkOnToesManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public enum SwitchToToeOffMethods
   {
      USE_ECMP, USE_ICP
   };
   
   public static final SwitchToToeOffMethods TOEOFF_TRIGGER_METHOD = SwitchToToeOffMethods.USE_ECMP;
   
   public enum ToeOffMotionType
   {
      QUINTIC_SPLINE, CUBIC_SPLINE, FREE
   };
   
   public static final ToeOffMotionType TOEOFF_MOTION_TYPE_USED = ToeOffMotionType.FREE;

   private final BooleanYoVariable stayOnToes = new BooleanYoVariable("stayOnToes", registry);
   
   private final BooleanYoVariable doToeOffIfPossible = new BooleanYoVariable("doToeOffIfPossible", registry);
   private final BooleanYoVariable doToeOff = new BooleanYoVariable("doToeOff", registry);
   
   private final BooleanYoVariable doToeTouchdownIfPossible = new BooleanYoVariable("doToeTouchdownIfPossible", registry);
   private final BooleanYoVariable doToeTouchdown = new BooleanYoVariable("doToeTouchdown", registry);

   private final DoubleYoVariable onToesTriangleArea = new DoubleYoVariable("onToesTriangleArea", registry);
   private final DoubleYoVariable onToesTriangleAreaLimit = new DoubleYoVariable("onToesTriangleAreaLimit", registry);
   private final BooleanYoVariable isOnToesTriangleLargeEnough = new BooleanYoVariable("isOnToesTriangleLargeEnough", registry);
   private FrameConvexPolygon2d onToesTriangle;

   private final BooleanYoVariable isDesiredICPOKForToeOff = new BooleanYoVariable("isDesiredICPOKForToeOff", registry);
   private final BooleanYoVariable isDesiredECMPOKForToeOff = new BooleanYoVariable("isDesiredECMPOKForToeOff", registry);

   private final DoubleYoVariable minStepLengthForToeOff = new DoubleYoVariable("minStepLengthForToeOff", registry);
   private final DoubleYoVariable minStepHeightForToeOff = new DoubleYoVariable("minStepHeightForToeOff", registry);

   private final SideDependentList<? extends ContactablePlaneBody> feet;
   private final Map<ContactablePlaneBody, EndEffectorControlModule> footEndEffectorControlModules;
   
   private final DoubleYoVariable extraCoMMaxHeightWithToes = new DoubleYoVariable("extraCoMMaxHeightWithToes", registry);
   
   public WalkOnToesManager(WalkingControllerParameters walkingControllerParameters, SideDependentList<? extends ContactablePlaneBody> feet,
         Map<ContactablePlaneBody, EndEffectorControlModule> footEndEffectorControlModules, YoVariableRegistry parentRegistry)
   {
      this.stayOnToes.set(walkingControllerParameters.stayOnToes());
      this.doToeOffIfPossible.set(walkingControllerParameters.doToeOffIfPossible());
      this.doToeTouchdownIfPossible.set(walkingControllerParameters.doToeTouchdownIfPossible());
      this.feet = feet;
      this.footEndEffectorControlModules = footEndEffectorControlModules;

      onToesTriangleAreaLimit.set(0.01);
      
      extraCoMMaxHeightWithToes.set(0.07);
      
      minStepLengthForToeOff.set(0.40);
      minStepHeightForToeOff.set(0.10);
      
      parentRegistry.addChild(registry);
   }

   public void updateToeOffStatusBasedOnECMP(RobotSide trailingLeg, FramePoint2d desiredECMP)
   {
      if (!doToeOffIfPossible.getBooleanValue() || stayOnToes.getBooleanValue() || TOEOFF_TRIGGER_METHOD != SwitchToToeOffMethods.USE_ECMP)
      {
         doToeOff.set(false);
         isDesiredECMPOKForToeOff.set(false);
         return;
      }
      
      ContactablePlaneBody trailingFoot = feet.get(trailingLeg);
      ContactablePlaneBody leadingFoot = feet.get(trailingLeg.getOppositeSide());
      FrameConvexPolygon2d OnToesSupportPolygon = getOnToesSupportPolygon(trailingFoot, leadingFoot);
      isDesiredECMPOKForToeOff.set(Math.abs(OnToesSupportPolygon.distance(desiredECMP)) < 0.06);
//    isDesiredCMPOKForToeOff.set(OnToesSupportPolygon.isPointInside(desiredECMP));

      if (!isDesiredECMPOKForToeOff.getBooleanValue())
      {
         doToeOff.set(false);
         return;
      }

      isReadyToSwitchToToeOff(trailingLeg);
   }
   
   public void updateToeOffStatusBasedOnICP(RobotSide trailingLeg, FramePoint2d desiredICP, FramePoint2d finalDesiredICP)
   {
      if (!doToeOffIfPossible.getBooleanValue() || stayOnToes.getBooleanValue() || TOEOFF_TRIGGER_METHOD != SwitchToToeOffMethods.USE_ICP)
      {
         doToeOff.set(false);
         isDesiredICPOKForToeOff.set(false);
         return;
      }
      
      updateOnToesTriangle(finalDesiredICP, trailingLeg);

      isDesiredICPOKForToeOff.set(onToesTriangle.isPointInside(desiredICP) && isOnToesTriangleLargeEnough.getBooleanValue());

      if (!isDesiredICPOKForToeOff.getBooleanValue())
      {
         doToeOff.set(false);
         return;
      }

      isReadyToSwitchToToeOff(trailingLeg);
   }

   public void updateOnToesTriangle(FramePoint2d finalDesiredICP, RobotSide supportSide)
   {
      onToesTriangle = getOnToesTriangle(finalDesiredICP, feet.get(supportSide));
      onToesTriangleArea.set(onToesTriangle.getArea());
      isOnToesTriangleLargeEnough.set(onToesTriangleArea.getDoubleValue() > onToesTriangleAreaLimit.getDoubleValue());
   }
   
   public boolean isOnToesTriangleLargeEnough()
   {
      return isOnToesTriangleLargeEnough.getBooleanValue();
   }

   private void isReadyToSwitchToToeOff(RobotSide trailingLeg)
   {
      RobotSide leadingLeg = trailingLeg.getOppositeSide();
      ReferenceFrame frontFootFrame = feet.get(leadingLeg).getBodyFrame();
      
      if (!isFrontFootWellPositionedForToeOff(trailingLeg, frontFootFrame))
      {
         doToeOff.set(false);
         return;
      }
      
      EndEffectorControlModule trailingEndEffectorControlModule = footEndEffectorControlModules.get(feet.get(trailingLeg));
      doToeOff.set(Math.abs(trailingEndEffectorControlModule.getJacobianDeterminant()) < 0.06);
   }
   
   private boolean isFrontFootWellPositionedForToeOff(RobotSide trailingLeg, ReferenceFrame frontFootFrame)
   {
      FramePoint leadingFootPosition = new FramePoint(frontFootFrame);
      ReferenceFrame trailingFootFrame = feet.get(trailingLeg).getBodyFrame();
      FramePoint trailingFootPosition = new FramePoint(trailingFootFrame);
      leadingFootPosition.changeFrame(trailingFootFrame);

      boolean isNextStepHighEnough = leadingFootPosition.getZ() > minStepHeightForToeOff.getDoubleValue();
      if (isNextStepHighEnough)
         return true;
      
      boolean isNextStepTooLow = leadingFootPosition.getZ() < -0.05;
      if (isNextStepTooLow)
         return false;
      
      boolean isForwardOrSideStepping = leadingFootPosition.getX() > -0.05;
      if (!isForwardOrSideStepping)
         return false;
      
      boolean isStepLongEnough = leadingFootPosition.distance(trailingFootPosition) > minStepLengthForToeOff.getDoubleValue();
      return isStepLongEnough;
   }
   
   public void checkAndRememberIfLandOnToes()
   {
      throw new RuntimeException("Not yet implemented");
   }
   
   public boolean willLandOnToes()
   {
      throw new RuntimeException("Not yet implemented");
   }
   
   public boolean willDoToeOff(TransferToAndNextFootstepsData transferToAndNextFootstepsData)
   {
      if (stayOnToes.getBooleanValue())
         return true;
      
      if (!doToeOffIfPossible.getBooleanValue())
         return false;
      
      RobotSide nextTrailingLeg = transferToAndNextFootstepsData.getTransferToSide().getOppositeSide();
      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();
      ReferenceFrame nextFrontFootFrame;
      if (nextFootstep != null)
         nextFrontFootFrame = nextFootstep.getPoseReferenceFrame();
      else
         nextFrontFootFrame = feet.get(nextTrailingLeg.getOppositeSide()).getBodyFrame();
      
      boolean frontFootWellPositionedForToeOff = isFrontFootWellPositionedForToeOff(nextTrailingLeg, nextFrontFootFrame);

      return frontFootWellPositionedForToeOff;
   }
   
   public boolean stayOnToes()
   {
      return stayOnToes.getBooleanValue();
   }

   public boolean doToeOff()
   {
      return doToeOff.getBooleanValue();
   }

   public boolean doToeOffIfPossible()
   {
      return doToeOffIfPossible.getBooleanValue();
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
   
   private FrameConvexPolygon2d getOnToesTriangle(FramePoint2d finalDesiredICP, ContactablePlaneBody supportFoot)
   {
      List<FramePoint> toePoints = getToePoints(supportFoot);
      Collection<FramePoint2d> points = new ArrayList<FramePoint2d>();
      for (FramePoint toePoint : toePoints)
      {
         toePoint.changeFrame(worldFrame);
         points.add(toePoint.toFramePoint2d());
      }

      points.add(finalDesiredICP);

      return new FrameConvexPolygon2d(points);
   }

   private List<FramePoint> getToePoints(ContactablePlaneBody supportFoot)
   {
      FrameVector forward = new FrameVector(supportFoot.getPlaneFrame(), 1.0, 0.0, 0.0);
      int nToePoints = 2;
      List<FramePoint> toePoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(supportFoot.getContactPoints(), forward, nToePoints);

      return toePoints;
   }

   private FrameConvexPolygon2d getOnToesSupportPolygon(ContactablePlaneBody trailingFoot, ContactablePlaneBody leadingFoot)
   {
      List<FramePoint> toePoints = getToePoints(trailingFoot);
      List<FramePoint> leadingFootPoints = leadingFoot.getContactPoints();

      List<FramePoint2d> allPoints = new ArrayList<FramePoint2d>();
      for (FramePoint framePoint : toePoints)
      {
         framePoint.changeFrame(worldFrame);
         allPoints.add(framePoint.toFramePoint2d());
      }

      for (FramePoint framePoint : leadingFootPoints)
      {
         framePoint.changeFrame(worldFrame);
         allPoints.add(framePoint.toFramePoint2d());
      }

      return new FrameConvexPolygon2d(allPoints);
   }

}
