package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.PolynomialSpline;

public class FlatThenPolynomialCoMHeightTrajectoryGenerator implements CenterOfMassHeightTrajectoryGenerator
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final ProcessedSensorsInterface processedSensors;
   private final DesiredFootstepCalculator desiredFootstepCalculator;
   private final ReferenceFrame referenceFrame;
   private final SideDependentList<BipedFootInterface> bipedFeet;
   private final CommonWalkingReferenceFrames referenceFrames;
   
   private final PolynomialSpline singleSupportSpline = new PolynomialSpline("comHeightSpline", 4, registry);
   private final DoubleYoVariable minXForSpline = new DoubleYoVariable("minXForSpline", registry);
   private final DoubleYoVariable maxXForSpline = new DoubleYoVariable("maxXForSpline", registry);
   private final DoubleYoVariable nominalHeightAboveGround = new DoubleYoVariable("nominalHeightAboveGround", registry);
   private final BooleanYoVariable hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);

   private final DoubleYoVariable desiredComHeight = new DoubleYoVariable("desiredComHeight", registry);
   private final DoubleYoVariable desiredComHeightSlope = new DoubleYoVariable("desiredComHeightSlope", registry);
   private final DoubleYoVariable desiredComHeightSecondDerivative = new DoubleYoVariable("desiredComHeightSecondDerivative", registry);

   public FlatThenPolynomialCoMHeightTrajectoryGenerator(ProcessedSensorsInterface processedSensors, DesiredFootstepCalculator desiredFootstepCalculator,
         ReferenceFrame desiredHeadingFrame, SideDependentList<BipedFootInterface> bipedFeet, CommonWalkingReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      this.desiredFootstepCalculator = desiredFootstepCalculator;
      this.referenceFrame = desiredHeadingFrame;
      this.bipedFeet = bipedFeet;
      this.referenceFrames = referenceFrames;

      nominalHeightAboveGround.set(1.36);
      parentRegistry.addChild(registry);
   }

   public void initialize(RobotSide supportLeg, RobotSide upcomingSupportLeg)
   {
      // initialize both the single support and double support trajectory on transition into single support:
      if (supportLeg == null)
      {
         if (!hasBeenInitialized.getBooleanValue())
         {
            double minZ = Math.min(findMinZOfGroundContactPoints(RobotSide.LEFT), findMinZOfGroundContactPoints(RobotSide.RIGHT));
            double z = minZ + nominalHeightAboveGround.getDoubleValue();
            singleSupportSpline.setConstant(z);    // will be overwritten later, but this fixes issues when initialize is called twice in double support
            hasBeenInitialized.set(true);
         }
      }
      else
      {
         compute();
         FramePoint com = processedSensors.getCenterOfMassPositionInFrame(referenceFrame);
         double x0 = com.getX();
         double z0 = getDesiredCenterOfMassHeight();
         double dzdx0 = getDesiredCenterOfMassHeightSlope();
//         double d2zdx20 = getDesiredCenterOfMassHeightSecondDerivative();

         Footstep footstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(supportLeg);
         double xf = computeXf(supportLeg, footstep);
         double zf = findMinZOfGroundContactPoints(supportLeg) + nominalHeightAboveGround.getDoubleValue();
         double dzdxf = 0.0;
//         double d2zdx2f = 0.0;

//         singleSupportSpline.setQuintic(x0, xf, z0, dzdx0, d2zdx20, zf, dzdxf, d2zdx2f);
         singleSupportSpline.setCubic(x0, xf, z0, dzdx0, zf, dzdxf);
         minXForSpline.set(x0);
         maxXForSpline.set(xf);
      }
   }

   public void compute()
   {
      FramePoint com = processedSensors.getCenterOfMassPositionInFrame(referenceFrame);
      double x = MathTools.clipToMinMax(com.getX(), minXForSpline.getDoubleValue(), maxXForSpline.getDoubleValue());
      singleSupportSpline.compute(x);

      desiredComHeight.set(getDesiredCenterOfMassHeight());
      desiredComHeightSlope.set(getDesiredCenterOfMassHeightSlope());
      desiredComHeightSecondDerivative.set(getDesiredCenterOfMassHeightSecondDerivative());
   }

   public double getDesiredCenterOfMassHeight()
   {
      return singleSupportSpline.getPosition();
   }

   public double getDesiredCenterOfMassHeightSlope()
   {
      return singleSupportSpline.getVelocity();
   }

   public double getDesiredCenterOfMassHeightSecondDerivative()
   {
      return singleSupportSpline.getAcceleration();
   }

   private double computeXf(RobotSide supportLeg, Footstep footstep)
   {
      double supportFootMaxX = findMaxXOfGroundContactPoints(supportLeg);
      double swingFootMaxX = findMaxXOfGroundContactPoints(footstep);
      return (supportFootMaxX + swingFootMaxX) / 2.0;
   }
   
   private double findMinZOfGroundContactPoints(Footstep footstep)
   {
      RobotSide robotSide = footstep.getFootstepSide();
      FramePose footstepPose = footstep.getFootstepPose();
      Transform3D desiredFootToDesiredHeading = new Transform3D();
      footstepPose.getTransform3D(desiredFootToDesiredHeading);

      ArrayList<FramePoint2d> footPoints = bipedFeet.get(robotSide).getFootPolygonInSoleFrame().getClockwiseOrderedListOfFramePoints();
      double minZ = Double.POSITIVE_INFINITY;
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      for (FramePoint2d footPoint : footPoints)
      {
         tempFramePoint.setToZero(footPoint.getReferenceFrame());
         tempFramePoint.setXY(footPoint);
         tempFramePoint.changeFrame(referenceFrames.getFootFrame(robotSide));

         tempFramePoint.changeFrameUsingTransform(referenceFrame, desiredFootToDesiredHeading);
         if (tempFramePoint.getZ() < minZ)
            minZ = tempFramePoint.getZ();
      }

      return minZ;
   }

   private double findMinZOfGroundContactPoints(RobotSide robotSide)
   {
      double minZ = Double.POSITIVE_INFINITY;
      ArrayList<FramePoint2d> footPoints = bipedFeet.get(robotSide).getFootPolygonInSoleFrame().getClockwiseOrderedListOfFramePoints();
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      for (FramePoint2d footPoint : footPoints)
      {
         tempFramePoint.setToZero(footPoint.getReferenceFrame());
         tempFramePoint.setXY(footPoint);
         tempFramePoint.changeFrame(referenceFrame);

         if (tempFramePoint.getZ() < minZ)
            minZ = tempFramePoint.getZ();
      }

      return minZ;
   }

   private double findMaxXOfGroundContactPoints(RobotSide robotSide)
   {
      double maxX = Double.NEGATIVE_INFINITY;
      ArrayList<FramePoint2d> footPoints = bipedFeet.get(robotSide).getFootPolygonInSoleFrame().getClockwiseOrderedListOfFramePoints();
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      for (FramePoint2d footPoint : footPoints)
      {
         tempFramePoint.setToZero(footPoint.getReferenceFrame());
         tempFramePoint.setXY(footPoint);
         tempFramePoint.changeFrame(referenceFrame);

         if (tempFramePoint.getX() > maxX)
            maxX = tempFramePoint.getX();
      }

      return maxX;
   }
   
   private double findMaxXOfGroundContactPoints(Footstep footstep)
   {
      RobotSide robotSide = footstep.getFootstepSide();
      FramePose footstepPose = footstep.getFootstepPose();
      Transform3D desiredFootToDesiredHeading = new Transform3D();
      footstepPose.getTransform3D(desiredFootToDesiredHeading);
      
      ArrayList<FramePoint2d> footPoints = bipedFeet.get(robotSide).getFootPolygonInSoleFrame().getClockwiseOrderedListOfFramePoints();
      double maxX = Double.NEGATIVE_INFINITY;
      FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      for (FramePoint2d footPoint : footPoints)
      {
         tempFramePoint.setToZero(footPoint.getReferenceFrame());
         tempFramePoint.setXY(footPoint);
         tempFramePoint.changeFrame(referenceFrames.getFootFrame(robotSide));
         
         tempFramePoint.changeFrameUsingTransform(referenceFrame, desiredFootToDesiredHeading);
         if (tempFramePoint.getX() > maxX)
            maxX = tempFramePoint.getX();
      }

      return maxX;
   }
}
