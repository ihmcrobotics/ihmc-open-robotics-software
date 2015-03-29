package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.lists.FrameTupleArrayList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;

public class CapturePointPlannerAdapter
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable capturePointInFromFootCenter = new DoubleYoVariable("icpInFromCenter", registry);
   private final DoubleYoVariable capturePointForwardFromFootCenter = new DoubleYoVariable("icpForwardFromCenter", registry);

   // FIXME That's a hack which makes the planner slower than the swing foot. Need to get rid of it.
   private final DoubleYoVariable additionalSwingTimeForICP = new DoubleYoVariable("additionalSwingTimeForICP", registry);

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FramePoint transferToFootLocation = new FramePoint(worldFrame);
   private final EnumYoVariable<RobotSide> currentTransferToSide;
   private final FramePoint tmpFramePoint = new FramePoint(worldFrame);
   private final FramePoint tmpFramePoint2 = new FramePoint(worldFrame);
   private final FramePoint2d tmpFramePoint2d = new FramePoint2d(worldFrame);
   private final FrameVector tmpFrameVector = new FrameVector(worldFrame);
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final EnumYoVariable<RobotSide> supportLeg = new EnumYoVariable<>("icpPlannerAdapterSupportLeg", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> sideOfFirstFootstep = new EnumYoVariable<>("icpPlannerAdapterSideOfFirstFootstep", registry, RobotSide.class, true);

   private final FrameTupleArrayList<FramePoint> footstepLocations = FrameTupleArrayList.createFramePointArrayList(10);
   private final FramePoint currentDesiredICP = new FramePoint();
   private final FrameVector currentDesiredICPVelocity = new FrameVector();

   private final NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation newCapturePointPlanner;
//   private final NewInstantaneousCapturePointPlannerWithSmoother newCapturePointPlanner;

   public CapturePointPlannerAdapter(CapturePointPlannerParameters capturePointPlannerParameters, YoVariableRegistry registry,
         YoGraphicsListRegistry yoGraphicsListRegistry, double controlDT, CommonHumanoidReferenceFrames referenceFrames,
         BipedSupportPolygons bipedSupportPolygons)
   {
      this.referenceFrames = referenceFrames;
      this.currentTransferToSide = new EnumYoVariable<RobotSide>("icpPlannerAdapterCurrentTransferToSide", registry, RobotSide.class);

      newCapturePointPlanner = new NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation(capturePointPlannerParameters, registry, yoGraphicsListRegistry);
      registry.addChild(this.registry);
      capturePointInFromFootCenter.set(capturePointPlannerParameters.getCapturePointInFromFootCenterDistance());
      capturePointForwardFromFootCenter.set(capturePointPlannerParameters.getCapturePointForwardFromFootCenterDistance());
      additionalSwingTimeForICP.set(capturePointPlannerParameters.getAdditionalTimeForSingleSupport());
      this.bipedSupportPolygons = bipedSupportPolygons;
   }

   public void setSingleSupportTime(double singleSupportTime)
   {
      newCapturePointPlanner.setSingleSupportTime(singleSupportTime + additionalSwingTimeForICP.getDoubleValue());
   }

   public void setDoubleSupportTime(double doubleSupportTime)
   {
      newCapturePointPlanner.setDoubleSupportTime(doubleSupportTime);
   }

   public void setInitialDoubleSupportTime(double doubleSupportTime)
   {
      newCapturePointPlanner.setInitialDoubleSupportTime(doubleSupportTime);
   }

   public void setOmega0(double omega0)
   {
      newCapturePointPlanner.setOmega0(omega0);
   }

   public void clear(RobotSide supportLeg)
   {
      this.supportLeg.set(supportLeg);
      sideOfFirstFootstep.set(null);
      footstepLocations.clear();
   }

   public void addFootstep(Footstep footstep)
   {
      if (footstep == null)
         return;

      if (footstepLocations.isEmpty())
         sideOfFirstFootstep.set(footstep.getRobotSide());

      List<Point2d> predictedContactPoints = footstep.getPredictedContactPoints();
      FramePoint footstepLocation = footstepLocations.add();
      footstepLocation.setToZero(footstep.getSoleReferenceFrame());

      if (predictedContactPoints != null)
      {
         int numberOfContactPoints = predictedContactPoints.size();
         for (int i = 0; i < numberOfContactPoints; i++)
         {
            Point2d contactPoint = predictedContactPoints.get(i);
            footstepLocation.setX(footstepLocation.getX() + contactPoint.getX() / (double) numberOfContactPoints);
            footstepLocation.setY(footstepLocation.getY() + contactPoint.getY() / (double) numberOfContactPoints);
         }
      }

      footstepLocation.setX(footstepLocation.getX() + capturePointForwardFromFootCenter.getDoubleValue());
      footstepLocation.setY(footstepLocation.getY() + footstep.getRobotSide().negateIfLeftSide(capturePointInFromFootCenter.getDoubleValue()));
      footstepLocation.changeFrame(worldFrame);
   }

   public void setCurrentDesiredICPState(YoFramePoint2d desiredICP, YoFrameVector2d desiredICPVelocity)
   {
      desiredICP.getFrameTupleIncludingFrame(currentDesiredICP);
      desiredICPVelocity.getFrameTupleIncludingFrame(currentDesiredICPVelocity);
   }

   public void setTransferToSide(RobotSide transferToSide)
   {
      sideOfFirstFootstep.set(transferToSide.getOppositeSide());
   }

   public void initializeSingleSupport(double initialTime)
   {
      FramePoint supportFootLocation = footstepLocations.insertAtIndex(0);
      RobotSide supportSide = supportLeg.getEnumValue();
      setSupportFootLocation(supportSide, supportFootLocation);

      newCapturePointPlanner.initializeSingleSupport(initialTime, footstepLocations);
   }

   public void initializeDoubleSupport(YoFramePoint2d desiredICP, YoFrameVector2d desiredICPVelocity, double initialTime)
   {
      setCurrentDesiredICPState(desiredICP, desiredICPVelocity);
      
      RobotSide transferFromSide = sideOfFirstFootstep.getEnumValue();
      if (transferFromSide == null) transferFromSide = RobotSide.LEFT;

      RobotSide transferToSide = transferFromSide.getOppositeSide();
      currentTransferToSide.set(transferToSide);

      FramePoint transferFromFootLocation;
      FramePoint transferToFootLocation;

      if (footstepLocations.isEmpty())
      {
         transferFromFootLocation = footstepLocations.add();
         transferToFootLocation = footstepLocations.add();
      }
      else
      {
         transferFromFootLocation = footstepLocations.insertAtIndex(0);
         transferToFootLocation = footstepLocations.insertAtIndex(1);
      }

      setSupportFootLocation(transferFromSide, transferFromFootLocation);
      setSupportFootLocation(transferToSide, transferToFootLocation);

      transferToFootLocation = new FramePoint(referenceFrames.getSoleFrame(transferToSide));
      transferToFootLocation.changeFrame(worldFrame);

      newCapturePointPlanner.initializeDoubleSupport(currentDesiredICP, currentDesiredICPVelocity, initialTime, footstepLocations, transferToSide, transferToFootLocation);
   }

   public void updatePlanForSingleSupportPush(FramePoint actualCapturePointPosition, double time)
   {
      FramePoint supportFootLocation = footstepLocations.insertAtIndex(0);
      RobotSide supportSide = supportLeg.getEnumValue();
      setSupportFootLocation(supportSide, supportFootLocation);

      newCapturePointPlanner.initializeSingleSupport(time, footstepLocations);
      newCapturePointPlanner.updatePlanForSingleSupportPush(footstepLocations, actualCapturePointPosition, time);
   }

   public void updatePlanForDoubleSupportPush(FramePoint actualCapturePointPosition, double time)
   {
      FramePoint supportFootLocation = footstepLocations.insertAtIndex(0);
      RobotSide supportSide = supportLeg.getEnumValue();
      setSupportFootLocation(supportSide, supportFootLocation);

      newCapturePointPlanner.initializeSingleSupport(time, footstepLocations);
      newCapturePointPlanner.updatePlanForDoubleSupportPush(footstepLocations, actualCapturePointPosition, time);
   }

   private void setSupportFootLocation(RobotSide supportSide, FramePoint footLocationToPack)
   {
      FrameConvexPolygon2d supportFootPolygon = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportSide);
      if (supportFootPolygon == null)
         footLocationToPack.setToZero(referenceFrames.getSoleFrame(supportSide));
      else
      {
         footLocationToPack.setXYIncludingFrame(supportFootPolygon.getCentroid());
         footLocationToPack.changeFrame(referenceFrames.getSoleFrame(supportSide));
      }
      footLocationToPack.setX(footLocationToPack.getX() + capturePointForwardFromFootCenter.getDoubleValue());
      footLocationToPack.setY(footLocationToPack.getY() + supportSide.negateIfLeftSide(capturePointInFromFootCenter.getDoubleValue()));
      footLocationToPack.setZ(0.0); // Otherwise we end up with the ankle height
      footLocationToPack.changeFrame(worldFrame);
   }

   public void getICPPositionAndVelocity(FramePoint2d icpPositionToPack, FrameVector2d icpVelocityToPack, FramePoint2d ecmpToPack, FramePoint2d actualICP,
         double time)
   {
      transferToFootLocation.setToZero(referenceFrames.getSoleFrame(currentTransferToSide.getEnumValue()));
      transferToFootLocation.changeFrame(worldFrame);

      tmpFramePoint2.set(actualICP.getX(), actualICP.getY(), 0.0);
      newCapturePointPlanner.packDesiredCapturePointPositionAndVelocity(tmpFramePoint, tmpFrameVector, time, tmpFramePoint2, transferToFootLocation);
//      newCapturePointPlanner.packDesiredCapturePointPositionAndVelocity(tmpFramePoint, tmpFrameVector, time);

      icpPositionToPack.set(tmpFramePoint.getX(), tmpFramePoint.getY());

      icpVelocityToPack.set(tmpFrameVector.getX(), tmpFrameVector.getY());

      newCapturePointPlanner.packDesiredCentroidalMomentumPivotPosition(tmpFramePoint);
      ecmpToPack.set(tmpFramePoint.getX(), tmpFramePoint.getY());
   }

   public void reset(double time)
   {
      newCapturePointPlanner.reset(time);
   }

   public boolean isDone(double time)
   {
      // This is a hack, and is needed for now because with the current icp planner, the single and double support 
      // durations are 0 on construction and then isDone() is called which returns true even though the 
      // planner has done nothing.
      if (newCapturePointPlanner.getHasBeenWokenUp())
      {
         return newCapturePointPlanner.isDone(time);
      }
      else
      {
         newCapturePointPlanner.wakeUp();
         return true;
      }
   }

   public double getEstimatedTimeRemainingForState(double time)
   {
      return newCapturePointPlanner.computeAndReturnTimeRemaining(time);
   }

   public boolean isPerformingICPDoubleSupport()
   {
      return newCapturePointPlanner.isDoubleSupport.getBooleanValue();
   }

   public FramePoint2d getFinalDesiredICP()
   {
      newCapturePointPlanner.getFinalDesiredCapturePointPosition(tmpFramePoint);
      tmpFramePoint2d.set(tmpFramePoint.getX(), tmpFramePoint.getY());
      return tmpFramePoint2d;
   }
}
