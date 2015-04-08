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
   private static final boolean USE_NEW_PLANNER = false;

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

   private final FrameTupleArrayList<FramePoint> footstepLocations = FrameTupleArrayList.createFramePointArrayList(10);
   private final FramePoint currentDesiredICP = new FramePoint();
   private final FrameVector currentDesiredICPVelocity = new FrameVector();

   private final NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation capturePointPlanner;
   private final ICPPlannerWithTimeFreezer icpPlanner;

   public CapturePointPlannerAdapter(CapturePointPlannerParameters capturePointPlannerParameters, YoVariableRegistry registry,
         YoGraphicsListRegistry yoGraphicsListRegistry, double controlDT, CommonHumanoidReferenceFrames referenceFrames,
         BipedSupportPolygons bipedSupportPolygons)
   {
      this.referenceFrames = referenceFrames;
      this.currentTransferToSide = new EnumYoVariable<RobotSide>("icpPlannerAdapterCurrentTransferToSide", registry, RobotSide.class);

      if (USE_NEW_PLANNER)
      {
         capturePointPlanner = null;
         icpPlanner = new ICPPlannerWithTimeFreezer(bipedSupportPolygons, referenceFrames, capturePointPlannerParameters, registry, yoGraphicsListRegistry);
      }
      else
      {
         capturePointPlanner = new NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation(capturePointPlannerParameters, registry, yoGraphicsListRegistry);
         icpPlanner = null;
      }

      registry.addChild(this.registry);
      capturePointInFromFootCenter.set(capturePointPlannerParameters.getReferenceCMPInsideOffset());
      capturePointForwardFromFootCenter.set(capturePointPlannerParameters.getReferenceCMPForwardOffset());
      additionalSwingTimeForICP.set(capturePointPlannerParameters.getAdditionalTimeForSingleSupport());
      this.bipedSupportPolygons = bipedSupportPolygons;
   }

   public void setSingleSupportTime(double singleSupportTime)
   {
      if (USE_NEW_PLANNER)
         icpPlanner.setSingleSupportTime(singleSupportTime + additionalSwingTimeForICP.getDoubleValue());
      else
         capturePointPlanner.setSingleSupportTime(singleSupportTime + additionalSwingTimeForICP.getDoubleValue());
   }

   public void setDoubleSupportTime(double doubleSupportTime)
   {
      if (USE_NEW_PLANNER)
         icpPlanner.setDoubleSupportTime(doubleSupportTime);
      else
         capturePointPlanner.setDoubleSupportTime(doubleSupportTime);
   }

   public void setInitialDoubleSupportTime(double doubleSupportTime)
   {
      if (USE_NEW_PLANNER)
         icpPlanner.setInitialDoubleSupportTime(doubleSupportTime);
      else      
         capturePointPlanner.setInitialDoubleSupportTime(doubleSupportTime);
   }

   public void setOmega0(double omega0)
   {
      if (USE_NEW_PLANNER)
         icpPlanner.setOmega0(omega0);
      else
         capturePointPlanner.setOmega0(omega0);
   }

   public void clear()
   {
      if (USE_NEW_PLANNER)
         icpPlanner.clearPlan();

      footstepLocations.clear();
   }

   public void addFootstep(Footstep footstep)
   {
      if (USE_NEW_PLANNER)
      {
         icpPlanner.addFootstepToPlan(footstep);
      }
      else
      {
         if (footstep == null)
            return;

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
   }

   public void setCurrentDesiredICPState(YoFramePoint2d desiredICP, YoFrameVector2d desiredICPVelocity)
   {
      desiredICP.getFrameTupleIncludingFrame(currentDesiredICP);
      desiredICPVelocity.getFrameTupleIncludingFrame(currentDesiredICPVelocity);
   }

   public void initializeSingleSupport(double initialTime, RobotSide supportSide)
   {
      this.supportLeg.set(supportSide);

      if (USE_NEW_PLANNER)
         icpPlanner.initializeSingleSupport(initialTime, supportSide);
      else
      {
         FramePoint supportFootLocation = footstepLocations.insertAtIndex(0);
         setSupportFootLocation(supportSide, supportFootLocation);

         capturePointPlanner.initializeSingleSupport(initialTime, footstepLocations);
      }
   }

   public void initializeDoubleSupport(YoFramePoint2d desiredICP, YoFrameVector2d desiredICPVelocity, double initialTime, RobotSide transferToSide)
   {
      setCurrentDesiredICPState(desiredICP, desiredICPVelocity);
      
      if (USE_NEW_PLANNER)
      {
         icpPlanner.setDesiredCapturePointState(desiredICP, desiredICPVelocity);
         icpPlanner.initializeDoubleSupport(initialTime, transferToSide);
      }
      else
      {
         if (transferToSide == null) transferToSide = RobotSide.LEFT;
         RobotSide transferFromSide = transferToSide.getOppositeSide();
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

         capturePointPlanner.initializeDoubleSupport(currentDesiredICP, currentDesiredICPVelocity, initialTime, footstepLocations, transferToSide, transferToFootLocation);
      }
   }

   public void updatePlanForSingleSupportDisturbances(FramePoint actualCapturePointPosition, double time, RobotSide supportSide)
   {
      if (USE_NEW_PLANNER)
      {
         icpPlanner.updatePlanForSingleSupportDisturbances(time, actualCapturePointPosition);
      }
      else
      {
         FramePoint supportFootLocation = footstepLocations.insertAtIndex(0);
         supportLeg.set(supportSide);
         setSupportFootLocation(supportSide, supportFootLocation);

         capturePointPlanner.updatePlanForSingleSupportDisturbances(time, footstepLocations, actualCapturePointPosition);
      }
   }

   private void setSupportFootLocation(RobotSide supportSide, FramePoint footLocationToPack)
   {
      FrameConvexPolygon2d supportFootPolygon = bipedSupportPolygons.getFootPolygonInSoleFrame(supportSide);
      footLocationToPack.setXYIncludingFrame(supportFootPolygon.getCentroid());
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
      if (USE_NEW_PLANNER)
         icpPlanner.packDesiredCapturePointPositionAndVelocity(tmpFramePoint, tmpFrameVector, tmpFramePoint2, time);
      else
         capturePointPlanner.packDesiredCapturePointPositionAndVelocity(tmpFramePoint, tmpFrameVector, time, tmpFramePoint2, transferToFootLocation);

      icpPositionToPack.set(tmpFramePoint.getX(), tmpFramePoint.getY());

      icpVelocityToPack.set(tmpFrameVector.getX(), tmpFrameVector.getY());

      if (USE_NEW_PLANNER)
         icpPlanner.packDesiredCentroidalMomentumPivotPosition(tmpFramePoint);
      else
         capturePointPlanner.packDesiredCentroidalMomentumPivotPosition(tmpFramePoint);
      ecmpToPack.set(tmpFramePoint.getX(), tmpFramePoint.getY());
   }

   public void reset(double time)
   {
      if (USE_NEW_PLANNER)
         icpPlanner.reset(time);
      else
      {
         transferToFootLocation.setToZero(referenceFrames.getSoleFrame(currentTransferToSide.getEnumValue()));
         transferToFootLocation.changeFrame(worldFrame);
         capturePointPlanner.reset(time, currentTransferToSide.getEnumValue(), transferToFootLocation);
      }
   }

   public boolean isDone(double time)
   {
      if (USE_NEW_PLANNER)
      {
         if (icpPlanner.getHasBeenWokenUp())
         {
            return icpPlanner.isDone(time);
         }
         else
         {
            icpPlanner.wakeUp();
            return true;
         }
      }
      else
      {
         // This is a hack, and is needed for now because with the current icp planner, the single and double support 
         // durations are 0 on construction and then isDone() is called which returns true even though the 
         // planner has done nothing.
         if (capturePointPlanner.getHasBeenWokenUp())
         {
            return capturePointPlanner.isDone(time);
         }
         else
         {
            capturePointPlanner.wakeUp();
            return true;
         }
      }
   }

   public double getEstimatedTimeRemainingForState(double time)
   {
      if (USE_NEW_PLANNER)
         return icpPlanner.computeAndReturnTimeRemaining(time);
      else
         return capturePointPlanner.computeAndReturnTimeRemaining(time);
   }

   public boolean isPerformingICPDoubleSupport()
   {
      if (USE_NEW_PLANNER)
         return icpPlanner.isInDoubleSupport();
      else
         return capturePointPlanner.isDoubleSupport.getBooleanValue();
   }

   public FramePoint2d getFinalDesiredICP()
   {
      if (USE_NEW_PLANNER)
         icpPlanner.getFinalDesiredCapturePointPosition(tmpFramePoint);
      else         
         capturePointPlanner.getFinalDesiredCapturePointPosition(tmpFramePoint);
      tmpFramePoint2d.set(tmpFramePoint.getX(), tmpFramePoint.getY());
      return tmpFramePoint2d;
   }
}
