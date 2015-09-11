package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.WalkOnTheEdgesManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;


public class SplineBasedHeightTrajectoryGenerator implements CoMHeightTrajectoryGenerator
{
   private static final boolean DEBUG = false;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final TwoPointSpline1D spline = new TwoPointSpline1D(registry);
   private final DoubleYoVariable nominalHeightAboveGround = new DoubleYoVariable("nominalHeightAboveGround", registry);
   private final DoubleYoVariable desiredCoMHeight = new DoubleYoVariable("desiredCoMHeight", registry);
   private LineSegment2d projectionSegment;

   private final YoFramePoint contactFrameZeroPosition = new YoFramePoint("contactFrameZeroPosition", worldFrame, registry);
   private final YoFramePoint contactFrameOnePosition = new YoFramePoint("contactFrameOnePosition", worldFrame, registry);

   public SplineBasedHeightTrajectoryGenerator(double nominalHeightAboveGround, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      this.nominalHeightAboveGround.set(nominalHeightAboveGround);
      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicPosition position0 = new YoGraphicPosition("contactFrame0", contactFrameZeroPosition, 0.03, YoAppearance.Purple());
         YoGraphicPosition position1 = new YoGraphicPosition("contactFrame1", contactFrameOnePosition, 0.03, YoAppearance.Gold());

         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", position0);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", position1);
      }
   }

   public void setNominalHeightAboveGround(double nominalHeightAboveGround)
   {
      this.nominalHeightAboveGround.set(nominalHeightAboveGround);
   }

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, RobotSide supportLeg, Footstep nextFootstep,
         List<PlaneContactState> contactStates)
   {
      FramePoint[] contactFramePositions = getContactStateCenters(contactStates, nextFootstep);

      contactFrameZeroPosition.set(contactFramePositions[0]);
      contactFrameOnePosition.set(contactFramePositions[1]);

      projectionSegment = new LineSegment2d(getPoint2d(contactFramePositions[0]), getPoint2d(contactFramePositions[1]));
      double s0 = 0.0;
      double sF = projectionSegment.length();
      double z0 = contactFramePositions[0].getZ() + nominalHeightAboveGround.getDoubleValue();
      double zF = contactFramePositions[1].getZ() + nominalHeightAboveGround.getDoubleValue();
      Point2d point0 = new Point2d(s0, z0);
      Point2d pointF = new Point2d(sF, zF);
      Point2d[] points = new Point2d[] { point0, pointF };
      double[] slopes = new double[] { 0.0, 0.0 };
      double[] secondDerivatives = new double[] { 0.0, 0.0 };
      spline.setPoints(points, slopes, secondDerivatives);
   }

   private Point2d getPoint2d(FramePoint point)
   {
      return new Point2d(point.getX(), point.getY());
   }

   public void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData)
   {
      Point2d queryPoint = getCenterOfMass2d(centerOfMassHeightInputData.getCenterOfMassFrame());

      projectionSegment.orthogonalProjection(queryPoint);
      double splineQuery = projectionSegment.percentageAlongLineSegment(queryPoint) * projectionSegment.length();

      double[] splineOutput = spline.getZSlopeAndSecondDerivative(splineQuery);
      double z = splineOutput[0];
      double dzds = splineOutput[1];
      double ddzdds = splineOutput[2];

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

      coMHeightPartialDerivativesDataToPack.setCoMHeight(worldFrame, z);
      coMHeightPartialDerivativesDataToPack.setPartialDzDx(dzdx);
      coMHeightPartialDerivativesDataToPack.setPartialDzDy(dzdy);
      coMHeightPartialDerivativesDataToPack.setPartialD2zDxDy(ddzdxdy);
      coMHeightPartialDerivativesDataToPack.setPartialD2zDx2(ddzddx);
      coMHeightPartialDerivativesDataToPack.setPartialD2zDy2(ddzddy);

      desiredCoMHeight.set(z);
   }

   private double[] getPartialDerivativesWithRespectToS(LineSegment2d segment)
   {
      double dsdx = (segment.getSecondEndPointCopy().getX() - segment.getFirstEndPointCopy().getX()) / segment.length();
      double dsdy = (segment.getSecondEndPointCopy().getY() - segment.getFirstEndPointCopy().getY()) / segment.length();

      return new double[] { dsdx, dsdy };
   }

   private FramePoint[] getContactStateCenters(List<PlaneContactState> contactStates, Footstep nextFootstep)
   {
      ReferenceFrame bodyFrame0 = contactStates.get(0).getFrameAfterParentJoint();

      FramePoint contactFramePosition0 = new FramePoint(bodyFrame0);
      contactFramePosition0.changeFrame(worldFrame);
      FramePoint contactFramePosition1;
      if (nextFootstep == null)
      {
         if (contactStates.size() != 2)
            throw new RuntimeException("contactStates.size() != 2");
         ReferenceFrame bodyFrame1 = contactStates.get(1).getFrameAfterParentJoint();

         contactFramePosition1 = new FramePoint(bodyFrame1);
         contactFramePosition1.changeFrame(worldFrame);
      }
      else
      {
         contactFramePosition1 = new FramePoint(nextFootstep.getPoseReferenceFrame());
         contactFramePosition1.changeFrame(worldFrame);
      }
      if (DEBUG)
      {
         System.out.println("nextFootstep: " + nextFootstep);
         System.out.println("contactFramePosition0: " + contactFramePosition0);
         System.out.println("contactFramePosition1: " + contactFramePosition1 + "\n");
      }
      return new FramePoint[] { contactFramePosition0, contactFramePosition1 };
   }

   private Point2d getCenterOfMass2d(ReferenceFrame centerOfMassFrame)
   {
      FramePoint coM = new FramePoint(centerOfMassFrame);
      coM.changeFrame(worldFrame);

      return getPoint2d(coM);
   }

   public boolean hasBeenInitializedWithNextStep()
   {
      return false;
   }

   public void attachWalkOnToesManager(WalkOnTheEdgesManager walkOnTheEdgesManager)
   {
   }

   @Override
   public void setSupportLeg(RobotSide supportLeg)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void initializeDesiredHeightToCurrent()
   {
      // TODO Auto-generated method stub
      
   }
}
