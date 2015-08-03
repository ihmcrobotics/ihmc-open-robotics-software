package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.WalkOnTheEdgesManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.BagOfBalls;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


public class FourPointCoMHeightTrajectoryGenerator implements CoMHeightTrajectoryGenerator
{
   private boolean VISUALIZE = true;

   private static final boolean DEBUG = false;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FourPointSpline1D spline = new FourPointSpline1D(registry);

   private final DoubleYoVariable nominalHeightAboveGround = new DoubleYoVariable("nominalHeightAboveGround", registry);
   private final DoubleYoVariable maximumHeightDeltaBetweenWaypoints = new DoubleYoVariable("maximumHeightDeltaBetweenWaypoints", registry);
   private final DoubleYoVariable doubleSupportPercentageIn = new DoubleYoVariable("doubleSupportPercentageIn", registry);

   private final DoubleYoVariable desiredCoMHeight = new DoubleYoVariable("desiredCoMHeight", registry);
   private LineSegment2d projectionSegment;

   private final YoFramePoint contactFrameZeroPosition = new YoFramePoint("contactFrameZeroPosition", worldFrame, registry);
   private final YoFramePoint contactFrameOnePosition = new YoFramePoint("contactFrameOnePosition", worldFrame, registry);

   private final YoGraphicPosition pointS0Viz, pointSFViz, pointD0Viz, pointDFViz;
   private final BagOfBalls bagOfBalls;

   public FourPointCoMHeightTrajectoryGenerator(double nominalHeightAboveGround, double doubleSupportPercentageIn,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      setNominalHeightAboveGround(nominalHeightAboveGround);
      this.doubleSupportPercentageIn.set(doubleSupportPercentageIn);

      this.maximumHeightDeltaBetweenWaypoints.set(0.2); //0.04);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry == null)
      {
         VISUALIZE = false;
      }
      if (VISUALIZE)
      {
         double pointSize = 0.03;

         YoGraphicPosition position0 = new YoGraphicPosition("contactFrame0", contactFrameZeroPosition, pointSize, YoAppearance.Purple());
         YoGraphicPosition position1 = new YoGraphicPosition("contactFrame1", contactFrameOnePosition, pointSize, YoAppearance.Gold());

         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", position0);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", position1);

         pointS0Viz = new YoGraphicPosition("pointS0", "", registry, pointSize, YoAppearance.CadetBlue());
         pointSFViz = new YoGraphicPosition("pointSF", "", registry, pointSize, YoAppearance.Chartreuse());
         pointD0Viz = new YoGraphicPosition("pointD0", "", registry, pointSize, YoAppearance.BlueViolet());
         pointDFViz = new YoGraphicPosition("pointDF", "", registry, pointSize, YoAppearance.Azure());

         bagOfBalls = new BagOfBalls(registry, yoGraphicsListRegistry);

         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointS0Viz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointSFViz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointD0Viz);
         yoGraphicsListRegistry.registerYoGraphic("CoMHeightTrajectoryGenerator", pointDFViz);
      }
      else
      {
         pointS0Viz = null;
         pointSFViz = null;
         pointD0Viz = null;
         pointDFViz = null;

         bagOfBalls = null;
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

      double s_d0 = doubleSupportPercentageIn.getDoubleValue() * sF;
      double s_df = (1.0 - doubleSupportPercentageIn.getDoubleValue()) * sF;

      double footHeight0 = contactFramePositions[0].getZ();
      double footHeight1 = contactFramePositions[1].getZ();

      double z0 = footHeight0 + nominalHeightAboveGround.getDoubleValue();
      double zF = footHeight1 + nominalHeightAboveGround.getDoubleValue();

      double z_d0 = findMaximumDoubleSupportHeight(s0, sF, s_d0, footHeight0, footHeight1);
      double z_dF = findMaximumDoubleSupportHeight(s0, sF, s_df, footHeight0, footHeight1);

      z_d0 = clipToMaximum(z_d0, z_dF + maximumHeightDeltaBetweenWaypoints.getDoubleValue());
      z_dF = clipToMaximum(z_dF, z_d0 + maximumHeightDeltaBetweenWaypoints.getDoubleValue());

      z0 = clipToMaximum(z0, z_d0 + maximumHeightDeltaBetweenWaypoints.getDoubleValue());
      zF = clipToMaximum(zF, z_dF + maximumHeightDeltaBetweenWaypoints.getDoubleValue());

      Point2d pointS0 = new Point2d(s0, z0);
      Point2d pointSF = new Point2d(sF, zF);
      Point2d pointD0 = new Point2d(s_d0, z_d0);
      Point2d pointDF = new Point2d(s_df, z_dF);

      Point2d[] points = new Point2d[] { pointS0, pointD0, pointDF, pointSF };
      double[] endpointSlopes = new double[] { 0.0, 0.0 };

      double[] waypointSlopes = new double[2];
      waypointSlopes[0] = (points[2].y - points[0].y) / (points[2].x - points[0].x);
      waypointSlopes[1] = (points[3].y - points[1].y) / (points[3].x - points[1].x);

      spline.setPoints(points, endpointSlopes, waypointSlopes);

      if (VISUALIZE)
      {
         FramePoint framePointD0 = new FramePoint(contactFramePositions[0].getReferenceFrame());
         framePointD0.interpolate(contactFramePositions[0], contactFramePositions[1], s_d0 / sF);
         framePointD0.setZ(z_d0);
         pointD0Viz.setPosition(framePointD0);

         FramePoint framePointDF = new FramePoint(contactFramePositions[0].getReferenceFrame());
         framePointDF.interpolate(contactFramePositions[0], contactFramePositions[1], (s_df) / sF);
         framePointDF.setZ(z_dF);
         pointDFViz.setPosition(framePointDF);

         FramePoint framePointS0 = new FramePoint(contactFramePositions[0]);
         framePointS0.setZ(z0);
         pointS0Viz.setPosition(framePointS0);

         FramePoint framePointSF = new FramePoint(contactFramePositions[1]);
         framePointSF.setZ(zF);
         pointSFViz.setPosition(framePointSF);

         bagOfBalls.reset();
         int numberOfPoints = 30;
         CoMHeightPartialDerivativesData coMHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();
         for (int i = 0; i < numberOfPoints; i++)
         {
            FramePoint framePoint = new FramePoint(contactFramePositions[0].getReferenceFrame());
            framePoint.interpolate(contactFramePositions[0], contactFramePositions[1], ((double) i) / ((double) numberOfPoints));
            Point2d queryPoint = new Point2d(framePoint.getX(), framePoint.getY());
            this.solve(coMHeightPartialDerivativesData, queryPoint);
            FramePoint framePointToPack = new FramePoint();
            coMHeightPartialDerivativesData.getCoMHeight(framePointToPack);
            framePointToPack.setX(framePoint.getX());
            framePointToPack.setY(framePoint.getY());

            bagOfBalls.setBallLoop(framePointToPack);
         }
      }
   }

   private double clipToMaximum(double value, double maxValue)
   {
      if (value < maxValue)
         return value;
      else
         return maxValue;
   }

   public double findMaximumDoubleSupportHeight(double s0, double sF, double s_d0, double foot0Height, double foot1Height)
   {
      double z_d0_A = foot0Height + Math.sqrt(MathTools.square(nominalHeightAboveGround.getDoubleValue()) - MathTools.square(s_d0 - s0));
      double z_d0_B = foot1Height + Math.sqrt(MathTools.square(nominalHeightAboveGround.getDoubleValue()) - MathTools.square((sF - s_d0)));
      double z_d0 = Math.min(z_d0_A, z_d0_B);
      return z_d0;
   }

   private Point2d getPoint2d(FramePoint point)
   {
      return new Point2d(point.getX(), point.getY());
   }

   public void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, ContactStatesAndUpcomingFootstepData centerOfMassHeightInputData)
   {
      Point2d queryPoint = getCenterOfMass2d(centerOfMassHeightInputData.getCenterOfMassFrame());
      solve(coMHeightPartialDerivativesDataToPack, queryPoint);
   }

   private final double[] splineOutput = new double[3];

   private void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, Point2d queryPoint)
   {
      projectionSegment.orthogonalProjection(queryPoint);
      double splineQuery = projectionSegment.percentageAlongLineSegment(queryPoint) * projectionSegment.length();

      spline.getZSlopeAndSecondDerivative(splineQuery, splineOutput);
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