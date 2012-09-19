package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedFootInterface;
import us.ihmc.commonWalkingControlModules.calculators.OrbitalEnergyCalculator;
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
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.trajectory.PolynomialSpline;

public class FlatThenPolynomialCoMHeightTrajectoryGenerator implements CenterOfMassHeightTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final ProcessedSensorsInterface processedSensors;
   private final DesiredFootstepCalculator desiredFootstepCalculator;
   private final ReferenceFrame referenceFrame;
   private final SideDependentList<BipedFootInterface> bipedFeet;
   private final CommonWalkingReferenceFrames referenceFrames;

   private static final int numberOfCoefficients = 6;

   private final PolynomialSpline heightSplineInFootFrame;
   private final DoubleYoVariable footX;
   private final DoubleYoVariable footZ;
   
   private final DoubleYoVariable minXForSpline;
   private final DoubleYoVariable maxXForSpline;

   private final DoubleYoVariable initialHeightAboveGround;
   private final DoubleYoVariable nominalHeightAboveGround;
   private final BooleanYoVariable hasBeenInitialized;

   private final DoubleYoVariable desiredComHeightInWorld;
   private final DoubleYoVariable desiredComHeightSlope;
   private final DoubleYoVariable desiredComHeightSecondDerivative;
   
   private final DoubleYoVariable orbitalEnergy;
   
   private final PolynomialSpline testHeightSplineInFootFrame;
   private final DoubleYoVariable deltaZ;

   public FlatThenPolynomialCoMHeightTrajectoryGenerator(String namePrefix, ProcessedSensorsInterface processedSensors, DesiredFootstepCalculator desiredFootstepCalculator,
         ReferenceFrame desiredHeadingFrame, SideDependentList<BipedFootInterface> bipedFeet, CommonWalkingReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      this.desiredFootstepCalculator = desiredFootstepCalculator;
      this.referenceFrame = desiredHeadingFrame;
      this.bipedFeet = bipedFeet;
      this.referenceFrames = referenceFrames;

      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      
      heightSplineInFootFrame = new PolynomialSpline("heightSplineInFootFrame", numberOfCoefficients, registry);
      footX = new DoubleYoVariable("footX", registry);
      footZ = new DoubleYoVariable("footZ", registry);
      
      minXForSpline = new DoubleYoVariable("minXForSpline", registry);
      maxXForSpline = new DoubleYoVariable("maxXForSpline", registry);
      
      initialHeightAboveGround = new DoubleYoVariable("initialHeightAboveGround", registry);
      nominalHeightAboveGround = new DoubleYoVariable("nominalHeightAboveGround", registry);
      hasBeenInitialized = new BooleanYoVariable("hasBeenInitialized", registry);

      desiredComHeightInWorld = new DoubleYoVariable("desiredComHeightInWorld", registry);
      desiredComHeightSlope = new DoubleYoVariable("desiredComHeightSlope", registry);
      desiredComHeightSecondDerivative = new DoubleYoVariable("desiredComHeightSecondDerivative", registry);
      
      orbitalEnergy = new DoubleYoVariable("orbitalEnergy", registry);
      
      testHeightSplineInFootFrame = new PolynomialSpline("testHeightSplineInFootFrame", numberOfCoefficients, registry);
      deltaZ = new DoubleYoVariable("deltaZ", registry);
      
      nominalHeightAboveGround.set(1.35);
      initialHeightAboveGround.set(1.28);
      parentRegistry.addChild(registry);
   }

   public void initialize(RobotSide supportLeg, RobotSide upcomingSupportLeg)
   {
      if (supportLeg == null)
      {
         if (!hasBeenInitialized.getBooleanValue())
         {
            footZ.set(Math.min(findMinZOfGroundContactPoints(RobotSide.LEFT), findMinZOfGroundContactPoints(RobotSide.RIGHT)));
            heightSplineInFootFrame.setConstant(initialHeightAboveGround.getDoubleValue());
            hasBeenInitialized.set(true);
         }
         // else just keep everything the same
      }
      else
      {
         compute();
         footX.set(findMaxXOfGroundContactPoints(supportLeg));
         footZ.set(findMinZOfGroundContactPoints(supportLeg));
         
         double[] x = initializeSpline(supportLeg, heightSplineInFootFrame, footX.getDoubleValue(), footZ.getDoubleValue());
         minXForSpline.set(x[0]);
         maxXForSpline.set(x[1]);
      }
   }

   /**
    * initializes a spline based on the current information
    * @param supportLeg 
    * @param spline
    * @return the minimum and maximum abscissa for which the spline is valid
    */
   private double[] initializeSpline(RobotSide supportLeg, PolynomialSpline spline, double offsetX, double offsetZ)
   {
      FramePoint com = processedSensors.getCenterOfMassPositionInFrame(referenceFrame);
      double x0 = com.getX() - offsetX;
      double z0 = getDesiredCenterOfMassHeight() - offsetZ;
      double dzdx0 = getDesiredCenterOfMassHeightSlope();
      double d2zdx20 = 0.0;

      Footstep footstep = desiredFootstepCalculator.updateAndGetDesiredFootstep(supportLeg);
      double xf = computeXf(supportLeg, footstep) - offsetX;
      double zf = findMinZOfGroundContactPoints(supportLeg) + nominalHeightAboveGround.getDoubleValue() - offsetZ;
      double dzdxf = 0.0;
      double d2zdx2f = 0.0;
      
      if (spline == heightSplineInFootFrame)
         deltaZ.set(zf - z0);

      spline.setQuintic(x0, xf, z0, dzdx0, d2zdx20, zf, dzdxf, d2zdx2f);
//      spline.setCubic(x0, xf, z0, dzdx0, zf, dzdxf);

      return new double[]{x0, xf};
   }

   public void compute()
   {
      FramePoint com = processedSensors.getCenterOfMassPositionInFrame(referenceFrame);
      double x = com.getX() - footX.getDoubleValue();
      heightSplineInFootFrame.compute(MathTools.clipToMinMax(x, minXForSpline.getDoubleValue(), maxXForSpline.getDoubleValue()));

      desiredComHeightInWorld.set(heightSplineInFootFrame.getPosition() + footZ.getDoubleValue());
      if (MathTools.isInsideBoundsInclusive(x, minXForSpline.getDoubleValue(), maxXForSpline.getDoubleValue()))
      {
         desiredComHeightSlope.set(heightSplineInFootFrame.getVelocity());
         desiredComHeightSecondDerivative.set(heightSplineInFootFrame.getAcceleration());
         orbitalEnergy.set(computeOrbitalEnergy(heightSplineInFootFrame, x));
      }
      else
      {
         desiredComHeightSlope.set(0.0);
         desiredComHeightSecondDerivative.set(0.0);
         orbitalEnergy.set(Double.NaN);
      }
   }

   public double getDesiredCenterOfMassHeight()
   {
      return desiredComHeightInWorld.getDoubleValue();
   }

   public double getDesiredCenterOfMassHeightSlope()
   {
      return desiredComHeightSlope.getDoubleValue();
   }

   public double getDesiredCenterOfMassHeightSecondDerivative()
   {
      return desiredComHeightSecondDerivative.getDoubleValue();
   }

   public double computeOrbitalEnergyIfInitializedNow(RobotSide upcomingSupportLeg)
   {
      double footX = findMaxXOfGroundContactPoints(upcomingSupportLeg);
      double footZ = findMinZOfGroundContactPoints(upcomingSupportLeg);
      initializeSpline(upcomingSupportLeg, testHeightSplineInFootFrame, footX, footZ);
      FramePoint com = processedSensors.getCenterOfMassPositionInFrame(referenceFrame);
      double x = com.getX() - footX;
      return computeOrbitalEnergy(testHeightSplineInFootFrame, x);
   }
   
   private double computeOrbitalEnergy(PolynomialSpline spline, double x)
   {
      FrameVector comd = processedSensors.getCenterOfMassVelocityInFrame(referenceFrame);
      double xd = comd.getX();
      double g = -processedSensors.getGravityInWorldFrame().getZ();
      return OrbitalEnergyCalculator.computeOrbitalEnergy(spline, g, x, xd);
   }
   
   private double computeXf(RobotSide supportLeg, Footstep footstep)
   {
      double supportFootMaxX = findMaxXOfGroundContactPoints(supportLeg);
      double swingFootMaxX = findMaxXOfGroundContactPoints(footstep);
      return (supportFootMaxX + swingFootMaxX) / 2.0;
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
