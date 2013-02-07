package us.ihmc.commonWalkingControlModules.trajectories;

import javax.vecmath.Point2d;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.LineSegment2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class NewestCoMTrajectoryGenerator implements CenterOfMassHeightTrajectoryGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final TwoPointSpline1D spline = new TwoPointSpline1D(registry);
   private final DoubleYoVariable nominalHeightAboveGround = new DoubleYoVariable("nominalHeightAboveGround", registry);
   private final DoubleYoVariable desiredCenterOfMassHeight = new DoubleYoVariable("desiredCenterOfMassHeight", registry);
   private final BooleanYoVariable coMGeneratorHasBeenInitializedBefore = new BooleanYoVariable("coMGeneratorHasBeenInitializedBefore", registry);

   public NewestCoMTrajectoryGenerator(double nominalHeightAboveGround, YoVariableRegistry parentRegistry)
   {
      this.nominalHeightAboveGround.set(nominalHeightAboveGround);
      coMGeneratorHasBeenInitializedBefore.set(false);
      parentRegistry.addChild(registry);
   }

   public void initialize(RobotSide supportLeg, Footstep nextFootstep)
   {
      FramePoint supportCenter = getSupportCenter(supportLeg);
      FramePoint footstepCenter = getFootstepCenter(nextFootstep);
      if (!coMGeneratorHasBeenInitializedBefore.getBooleanValue())
      {
         desiredCenterOfMassHeight.set(supportCenter.getZ() + nominalHeightAboveGround.getDoubleValue());
         coMGeneratorHasBeenInitializedBefore.set(true);
      }

      double s0 = 0.0;
      double sF = footstepCenter.distance(supportCenter);
      double z0 = desiredCenterOfMassHeight.getDoubleValue();
      double zF = footstepCenter.getZ() + nominalHeightAboveGround.getDoubleValue();
      Point2d point0 = new Point2d(s0, z0);
      Point2d pointF = new Point2d(sF, zF);
      Point2d[] points = new Point2d[] {point0, pointF};
      double[] slopes = new double[] {0.0, 0.0};
      double[] secondDerivatives = new double[] {0.0, 0.0};
      spline.setPoints(points, slopes, secondDerivatives);
   }

   private Point2d getPoint2d(FramePoint point)
   {
      return new Point2d(point.getX(), point.getY());
   }

   public void solve(CenterOfMassHeightOutputData centerOfMassHeightOutputDataToPack, CenterOfMassHeightInputData centerOfMassHeightInputData)
   {
      FramePoint supportCenter = getSupportCenter(centerOfMassHeightInputData.getSupportLeg());
      FramePoint footstepCenter = getFootstepCenter(centerOfMassHeightInputData.getUpcomingFootstep());
      Point2d queryPoint = getCenterOfMass2d(centerOfMassHeightInputData.getCenterOfMassFrame());

      LineSegment2d projectionSegment = new LineSegment2d(getPoint2d(supportCenter), getPoint2d(footstepCenter));
      projectionSegment.orthogonalProjection(queryPoint);
      double splineQuery = projectionSegment.percentageAlongLineSegment(queryPoint) * projectionSegment.length();

      double[] splineOutput = spline.getZSlopeAndSecondDerivative(splineQuery);
      double z = splineOutput[0];
      double dzds = splineOutput[1];
      double ddzdds = splineOutput[2];
      desiredCenterOfMassHeight.set(z);

      double[] partialDerivativesWithRespectToS = getPartialDerivativesWithRespectToS(projectionSegment);
      double dsdx = partialDerivativesWithRespectToS[0];
      double dsdy = partialDerivativesWithRespectToS[1];
      double ddsddx = 0;
      double ddsddy = 0;
      double ddsdxdy = 0;
      
      double dzdx = dsdx * dzds;
      double dzdy = dsdy * dzds;
      double ddzddx = dzds * ddsddx + ddzdds * dsdx * dsdx;
      double ddzddy = dzds * ddsddy + ddzdds * dsdy * dsdy;
      double ddzdxdy = ddzdds * dsdx * dsdy + dzds * ddsdxdy;
      
      centerOfMassHeightOutputDataToPack.setDesiredCenterOfMassHeight(z);
      centerOfMassHeightOutputDataToPack.setDesiredCenterOfMassHeightSlope(new FrameVector2d(worldFrame, dzdx, dzdy));
      centerOfMassHeightOutputDataToPack.setDesiredCenterOfMassHeightSecondDerivative(new FrameVector2d(worldFrame, ddzddx, ddzddy));
      centerOfMassHeightOutputDataToPack.setDdzdxdy(ddzdxdy);
   }

   private double[] getPartialDerivativesWithRespectToS(LineSegment2d segment)
   {
      double dsdx = (segment.getSecondEndPointCopy().getX() - segment.getFirstEndPointCopy().getX()) / segment.length();
      double dsdy = (segment.getSecondEndPointCopy().getY() - segment.getFirstEndPointCopy().getY()) / segment.length();

      return new double[] {dsdx, dsdy};
   }

   private FramePoint getSupportCenter(RobotSide supportLeg)
   {
      return null;
   }

   private FramePoint getFootstepCenter(Footstep nextFootstep)
   {
      return nextFootstep.getPositionInFrame(worldFrame);
   }

   private Point2d getCenterOfMass2d(ReferenceFrame centerOfMassFrame)
   {
      FramePoint coM = new FramePoint(centerOfMassFrame);
      coM.changeFrame(worldFrame);

      return getPoint2d(coM);
   }
}
