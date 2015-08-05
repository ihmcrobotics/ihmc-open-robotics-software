package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
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
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final BipedSupportPolygons bipedSupportPolygons;
   private final EnumYoVariable<RobotSide> supportLeg = new EnumYoVariable<>("icpPlannerAdapterSupportLeg", registry, RobotSide.class, true);

   private final FrameTupleArrayList<FramePoint> footstepLocations = FrameTupleArrayList.createFramePointArrayList(10);
   private final FramePoint currentDesiredICP = new FramePoint();
   private final FrameVector currentDesiredICPVelocity = new FrameVector();

   private final boolean useNewICPPlanner;
   private final NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation capturePointPlanner;
   private final ICPPlannerWithTimeFreezer icpPlanner;

   private double omega0;

   public CapturePointPlannerAdapter(CapturePointPlannerParameters capturePointPlannerParameters, WalkingControllerParameters walkingControllerParameters, YoVariableRegistry registry,
         YoGraphicsListRegistry yoGraphicsListRegistry, double controlDT, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
         BipedSupportPolygons bipedSupportPolygons)
   {
      useNewICPPlanner = capturePointPlannerParameters.useNewICPPlanner();
      this.currentTransferToSide = new EnumYoVariable<RobotSide>("icpPlannerAdapterCurrentTransferToSide", registry, RobotSide.class);

      for (RobotSide robotSide : RobotSide.values)
         soleFrames.put(robotSide, contactableFeet.get(robotSide).getSoleFrame());

      if (useNewICPPlanner)
      {
         capturePointPlanner = null;
         icpPlanner = new ICPPlannerWithTimeFreezer(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters, registry, yoGraphicsListRegistry);
         icpPlanner.setMinimumSingleSupportTimeForDisturbanceRecovery(walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery());
      }
      else
      {
         capturePointPlanner = new NewInstantaneousCapturePointPlannerWithTimeFreezerAndFootSlipCompensation(capturePointPlannerParameters, registry, yoGraphicsListRegistry);
         icpPlanner = null;
      }

      registry.addChild(this.registry);
      capturePointInFromFootCenter.set(capturePointPlannerParameters.getEntryCMPInsideOffset());
      capturePointForwardFromFootCenter.set(capturePointPlannerParameters.getEntryCMPForwardOffset());
      additionalSwingTimeForICP.set(capturePointPlannerParameters.getAdditionalTimeForSingleSupport());
      this.bipedSupportPolygons = bipedSupportPolygons;
   }

   public void setSingleSupportTime(double singleSupportTime)
   {
      if (useNewICPPlanner)
         icpPlanner.setSingleSupportTime(singleSupportTime + additionalSwingTimeForICP.getDoubleValue());
      else
         capturePointPlanner.setSingleSupportTime(singleSupportTime + additionalSwingTimeForICP.getDoubleValue());
   }

   public void setDoubleSupportTime(double doubleSupportTime)
   {
      if (useNewICPPlanner)
         icpPlanner.setDoubleSupportTime(doubleSupportTime);
      else
         capturePointPlanner.setDoubleSupportTime(doubleSupportTime);
   }

   public void setInitialDoubleSupportTime(double doubleSupportTime)
   {
      if (useNewICPPlanner)
         icpPlanner.setInitialDoubleSupportTime(doubleSupportTime);
      else      
         capturePointPlanner.setInitialDoubleSupportTime(doubleSupportTime);
   }

   public void setOmega0(double omega0)
   {
      this.omega0 = omega0;
      if (useNewICPPlanner)
         icpPlanner.setOmega0(omega0);
      else
         capturePointPlanner.setOmega0(omega0);
   }

   public void clear()
   {
      if (useNewICPPlanner)
         icpPlanner.clearPlan();

      footstepLocations.clear();
   }

   public void addFootstep(Footstep footstep)
   {
      if (useNewICPPlanner)
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

   public void setDesiredCapturePointState(YoFramePoint desiredICP, YoFrameVector desiredICPVelocity)
   {
      desiredICP.getFrameTupleIncludingFrame(currentDesiredICP);
      desiredICPVelocity.getFrameTupleIncludingFrame(currentDesiredICPVelocity);
   }
   
   public void setDesiredCapturePointState(FramePoint2d desiredICP, FrameVector2d desiredICPVelocity)
   {
      currentDesiredICP.setXY(desiredICP);
      currentDesiredICPVelocity.setXY(desiredICPVelocity);
   }

   public void setCurrentDesiredICPState(YoFramePoint2d desiredICP, YoFrameVector2d desiredICPVelocity)
   {
      desiredICP.getFrameTupleIncludingFrame(currentDesiredICP);
      desiredICPVelocity.getFrameTupleIncludingFrame(currentDesiredICPVelocity);
   }

   public void initializeSingleSupport(double initialTime, RobotSide supportSide)
   {
      this.supportLeg.set(supportSide);

      if (useNewICPPlanner)
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
      if (desiredICP != null)
         setCurrentDesiredICPState(desiredICP, desiredICPVelocity);
      
      if (useNewICPPlanner)
      {
         icpPlanner.setDesiredCapturePointState(currentDesiredICP, currentDesiredICPVelocity);
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

      transferToFootLocation = new FramePoint(soleFrames.get(transferToSide));
      transferToFootLocation.changeFrame(worldFrame);

         capturePointPlanner.initializeDoubleSupport(currentDesiredICP, currentDesiredICPVelocity, initialTime, footstepLocations, transferToSide, transferToFootLocation);
      }
   }

   public void updatePlanForSingleSupportDisturbances(FramePoint actualCapturePointPosition, double time, RobotSide supportSide)
   {
      if (useNewICPPlanner)
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
      transferToFootLocation.setToZero(soleFrames.get(currentTransferToSide.getEnumValue()));
      transferToFootLocation.changeFrame(worldFrame);

      tmpFramePoint2.setXYIncludingFrame(actualICP);
      if (useNewICPPlanner)
         icpPlanner.packDesiredCapturePointPositionAndVelocity(tmpFramePoint, tmpFrameVector, tmpFramePoint2, time);
      else
         capturePointPlanner.packDesiredCapturePointPositionAndVelocity(tmpFramePoint, tmpFrameVector, time, tmpFramePoint2, transferToFootLocation);

      icpPositionToPack.setByProjectionOntoXYPlaneIncludingFrame(tmpFramePoint);
      icpVelocityToPack.setByProjectionOntoXYPlaneIncludingFrame(tmpFrameVector);

      CapturePointTools.computeDesiredCentroidalMomentumPivot(icpPositionToPack, icpVelocityToPack, omega0, ecmpToPack);
   }

   public void reset(double time)
   {
      if (useNewICPPlanner)
         icpPlanner.reset(time);
      else
      {
         transferToFootLocation.setToZero(soleFrames.get(currentTransferToSide.getEnumValue()));
         transferToFootLocation.changeFrame(worldFrame);
         capturePointPlanner.reset(time, currentTransferToSide.getEnumValue(), transferToFootLocation);
      }
   }

   public boolean isDone(double time)
   {
      if (useNewICPPlanner)
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

   public double getInitialTransferDuration()
   {
      if (useNewICPPlanner)
         return icpPlanner.getInitialTransferDuration();
      else
         return capturePointPlanner.getInitialTransferDuration();
   }

   public double getEstimatedTimeRemainingForState(double time)
   {
      if (useNewICPPlanner)
         return icpPlanner.computeAndReturnTimeRemaining(time);
      else
         return capturePointPlanner.computeAndReturnTimeRemaining(time);
   }

   public double estimateTimeRemainingForStateUnderDisturbance(double time, FramePoint actualICP)
   {
      if (useNewICPPlanner)
         return icpPlanner.estimateTimeRemainingForStateUnderDisturbance(time, actualICP);
      else
         return capturePointPlanner.computeAndReturnTimeRemaining(time);
   }

   public boolean isPerformingICPDoubleSupport()
   {
      if (useNewICPPlanner)
         return icpPlanner.isInDoubleSupport();
      else
         return capturePointPlanner.isDoubleSupport.getBooleanValue();
   }

   public FramePoint2d getFinalDesiredICP()
   {
      if (useNewICPPlanner)
         icpPlanner.getFinalDesiredCapturePointPosition(tmpFramePoint);
      else         
         capturePointPlanner.getFinalDesiredCapturePointPosition(tmpFramePoint);
      tmpFramePoint.changeFrame(worldFrame);
      tmpFramePoint2d.setByProjectionOntoXYPlaneIncludingFrame(tmpFramePoint);
      return tmpFramePoint2d;
   }

   public void holdCurrentICP(double initialTime, FramePoint actualICPToHold)
   {
      if (useNewICPPlanner)
         icpPlanner.holdCurrentICP(initialTime, actualICPToHold);
      else
         PrintTools.error(this.getClass(), "Old planner cannot hold current ICP");
   }
}
