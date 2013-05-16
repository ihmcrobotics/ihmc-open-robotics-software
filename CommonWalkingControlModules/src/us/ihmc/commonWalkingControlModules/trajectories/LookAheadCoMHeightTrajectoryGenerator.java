package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.LineSegment2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class LookAheadCoMHeightTrajectoryGenerator implements CoMHeightTrajectoryGenerator
{
   private boolean VISUALIZE = true;
   
   private static final boolean DEBUG = false; 
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FourPointSpline1D spline = new FourPointSpline1D(registry);
   
   private final DoubleYoVariable minimumHeightAboveGround = new DoubleYoVariable("minimumHeightAboveGround", registry);
   private final DoubleYoVariable maximumHeightAboveGround = new DoubleYoVariable("maximumHeightAboveGround", registry);
   
   private final DoubleYoVariable maximumHeightDeltaBetweenWaypoints = new DoubleYoVariable("maximumHeightDeltaBetweenWaypoints", registry);
   private final DoubleYoVariable doubleSupportPercentageIn = new DoubleYoVariable("doubleSupportPercentageIn", registry);
   
   private final DoubleYoVariable desiredCoMHeight = new DoubleYoVariable("desiredCoMHeight", registry);
   private LineSegment2d projectionSegment;

   private final YoFramePoint contactFrameZeroPosition = new YoFramePoint("contactFrameZeroPosition", worldFrame, registry);
   private final YoFramePoint contactFrameOnePosition = new YoFramePoint("contactFrameOnePosition", worldFrame, registry);
   
   private final DynamicGraphicPosition pointS0Viz, pointSFViz, pointD0Viz, pointDFViz, pointSNextViz;
   private final DynamicGraphicPosition pointS0MinViz, pointSFMinViz, pointD0MinViz, pointDFMinViz, pointSNextMinViz;
   private final DynamicGraphicPosition pointS0MaxViz, pointSFMaxViz, pointD0MaxViz, pointDFMaxViz, pointSNextMaxViz;
   
   private final BagOfBalls bagOfBalls;
   
   public LookAheadCoMHeightTrajectoryGenerator(double minimumHeightAboveGround, double maximumHeightAboveGround, double doubleSupportPercentageIn, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      setMinimumHeightAboveGround(minimumHeightAboveGround);
      setMaximumHeightAboveGround(maximumHeightAboveGround);

      this.doubleSupportPercentageIn.set(doubleSupportPercentageIn);
      
      this.maximumHeightDeltaBetweenWaypoints.set(0.2); //0.04);

      parentRegistry.addChild(registry);
      
      if (dynamicGraphicObjectsListRegistry == null)
      {
         VISUALIZE = false;
      }
      if (VISUALIZE)
      {
         double pointSize = 0.03;
         
         DynamicGraphicPosition position0 = new DynamicGraphicPosition("contactFrame0", contactFrameZeroPosition, pointSize, YoAppearance.Purple());
         DynamicGraphicPosition position1 = new DynamicGraphicPosition("contactFrame1", contactFrameOnePosition, pointSize, YoAppearance.Gold());

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", position0);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", position1);

         pointS0Viz = new DynamicGraphicPosition("pointS0", "", registry, pointSize, YoAppearance.CadetBlue());
         pointSFViz = new DynamicGraphicPosition("pointSF", "", registry, pointSize, YoAppearance.Chartreuse());
         pointD0Viz = new DynamicGraphicPosition("pointD0", "", registry, pointSize, YoAppearance.BlueViolet());
         pointDFViz = new DynamicGraphicPosition("pointDF", "", registry, pointSize, YoAppearance.Azure());
         pointSNextViz = new DynamicGraphicPosition("pointSNext", "", registry, pointSize, YoAppearance.Azure());
         
         pointS0MinViz = new DynamicGraphicPosition("pointS0Min", "", registry, 0.8*pointSize, YoAppearance.CadetBlue());
         pointSFMinViz = new DynamicGraphicPosition("pointSFMin", "", registry, 0.8*pointSize, YoAppearance.Chartreuse());
         pointD0MinViz = new DynamicGraphicPosition("pointD0Min", "", registry, 0.8*pointSize, YoAppearance.BlueViolet());
         pointDFMinViz = new DynamicGraphicPosition("pointDFMin", "", registry, 0.8*pointSize, YoAppearance.Azure());
         pointSNextMinViz = new DynamicGraphicPosition("pointSNextMin", "", registry, 0.8*pointSize, YoAppearance.Azure());
         
         pointS0MaxViz = new DynamicGraphicPosition("pointS0Max", "", registry, 0.9*pointSize, YoAppearance.CadetBlue());
         pointSFMaxViz = new DynamicGraphicPosition("pointSFMax", "", registry, 0.9*pointSize, YoAppearance.Chartreuse());
         pointD0MaxViz = new DynamicGraphicPosition("pointD0Max", "", registry, 0.9*pointSize, YoAppearance.BlueViolet());
         pointDFMaxViz = new DynamicGraphicPosition("pointDFMax", "", registry, 0.9*pointSize, YoAppearance.Azure());
         pointSNextMaxViz = new DynamicGraphicPosition("pointSNextMax", "", registry, 0.9*pointSize, YoAppearance.Azure());
         
         bagOfBalls = new BagOfBalls(registry, dynamicGraphicObjectsListRegistry);
         
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointS0Viz);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointSFViz);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointD0Viz);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointDFViz);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointSNextViz);
         
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointS0MinViz);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointSFMinViz);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointD0MinViz);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointDFMinViz);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointSNextMinViz);
         
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointS0MaxViz);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointSFMaxViz);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointD0MaxViz);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointDFMaxViz);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("CoMHeightTrajectoryGenerator", pointSNextMaxViz);
      }
      else
      {
         pointS0Viz = null;
         pointSFViz = null;
         pointD0Viz = null;
         pointDFViz = null;
         pointSNextViz = null;
         
         pointS0MinViz = null;
         pointSFMinViz = null;
         pointD0MinViz = null;
         pointDFMinViz = null;
         pointSNextMinViz = null;
         
         pointS0MaxViz = null;
         pointSFMaxViz = null;
         pointD0MaxViz = null;
         pointDFMaxViz = null;
         pointSNextMaxViz = null;
         
         bagOfBalls = null;
      }
   }
   
   public void setMinimumHeightAboveGround(double minimumHeightAboveGround)
   {
      this.minimumHeightAboveGround.set(minimumHeightAboveGround);
   }
   
   public void setMaximumHeightAboveGround(double maximumHeightAboveGround)
   {
      this.maximumHeightAboveGround.set(maximumHeightAboveGround);
   }

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, RobotSide supportLeg2, Footstep nextFootstep2, List<PlaneContactState> contactStates2)
   {
      Footstep transferFromFootstep = transferToAndNextFootstepsData.getTransferFromFootstep();
      Footstep transferToFootstep = transferToAndNextFootstepsData.getTransferToFootstep();
      Footstep nextFootstep = transferToAndNextFootstepsData.getNextFootstep();
      
//      FramePoint[] contactFramePositions = getContactStateCenters(contactStates, nextFootstep);
      
      FramePoint transferFromContactFramePosition = new FramePoint(transferFromFootstep.getPoseReferenceFrame());
      FramePoint transferToContactFramePosition = new FramePoint(transferToFootstep.getPoseReferenceFrame());
      
      transferFromContactFramePosition.changeFrame(worldFrame);
      transferToContactFramePosition.changeFrame(worldFrame);
      
      FramePoint nextContactFramePosition = null;
      if (nextFootstep != null)
      {
         nextContactFramePosition = new FramePoint(nextFootstep.getPoseReferenceFrame());
         nextContactFramePosition.changeFrame(worldFrame);
      }
      
      contactFrameZeroPosition.set(transferFromContactFramePosition);
      contactFrameOnePosition.set(transferToContactFramePosition);
      
      projectionSegment = new LineSegment2d(getPoint2d(transferFromContactFramePosition), getPoint2d(transferToContactFramePosition));
      double s0 = 0.0;
      double sF = projectionSegment.length();
      
      double s_d0 = doubleSupportPercentageIn.getDoubleValue() * sF;
      double s_df = (1.0 - doubleSupportPercentageIn.getDoubleValue()) * sF;

      double footHeight0 = transferFromContactFramePosition.getZ();
      double footHeight1 = transferToContactFramePosition.getZ();
      
      double nextFootHeight = Double.NaN;
      if (nextContactFramePosition != null)
      {
         nextFootHeight = nextContactFramePosition.getZ();
      }
      
      double z0Min = footHeight0 + minimumHeightAboveGround.getDoubleValue();
      double z0Max = footHeight0 + maximumHeightAboveGround.getDoubleValue();

      double zFMin = footHeight1 + minimumHeightAboveGround.getDoubleValue();
      double zFMax = footHeight1 + maximumHeightAboveGround.getDoubleValue();
      
      double z_d0Min = findMinimumDoubleSupportHeight(s0, sF, s_d0, footHeight0, footHeight1);      
      double z_d0Max = findMaximumDoubleSupportHeight(s0, sF, s_d0, footHeight0, footHeight1);
      
      double z_dFMin = findMinimumDoubleSupportHeight(s0, sF, s_df, footHeight0, footHeight1);
      double z_dFMax = findMaximumDoubleSupportHeight(s0, sF, s_df, footHeight0, footHeight1);
      
      double zNextMin = nextFootHeight + minimumHeightAboveGround.getDoubleValue();
      double zNextMax = nextFootHeight + maximumHeightAboveGround.getDoubleValue();
  
      double z0 = z0Max;
      double z_d0 = z_d0Max;
      double z_dF = z_dFMax;
      double zF = zFMax;
      double zNext = zNextMax;

      z_d0 = clipToMaximum(z_d0, z_dF + maximumHeightDeltaBetweenWaypoints.getDoubleValue());
      z_dF = clipToMaximum(z_dF, z_d0 + maximumHeightDeltaBetweenWaypoints.getDoubleValue());
      
      z0 = clipToMaximum(z0, z_d0 + maximumHeightDeltaBetweenWaypoints.getDoubleValue());
      zF = clipToMaximum(zF, z_dF + maximumHeightDeltaBetweenWaypoints.getDoubleValue());

      Point2d pointS0 = new Point2d(s0, z0);
      Point2d pointSF = new Point2d(sF, zF);
      Point2d pointD0 = new Point2d(s_d0, z_d0);
      Point2d pointDF = new Point2d(s_df, z_dF);
      
      Point2d[] points = new Point2d[] {pointS0, pointD0, pointDF, pointSF};
      double[] endpointSlopes = new double[] {0.0, 0.0};
      
      double[] waypointSlopes = new double[2];
      waypointSlopes[0] = (points[2].y - points[0].y) / (points[2].x - points[0].x);
      waypointSlopes[1] = (points[3].y - points[1].y) / (points[3].x - points[1].x);
      
      spline.setPoints(points, endpointSlopes, waypointSlopes);
      
      if (VISUALIZE)
      {
         FramePoint framePointS0 = new FramePoint(transferFromContactFramePosition);
         framePointS0.setZ(z0);
         pointS0Viz.setPosition(framePointS0);
         
         framePointS0.setZ(z0Min);
         pointS0MinViz.setPosition(framePointS0);
         
         framePointS0.setZ(z0Max);
         pointS0MaxViz.setPosition(framePointS0);
         
         FramePoint framePointD0 = FramePoint.morph(transferFromContactFramePosition, transferToContactFramePosition, s_d0/sF);
         framePointD0.setZ(z_d0);
         pointD0Viz.setPosition(framePointD0);
         
         framePointD0.setZ(z_d0Min);
         pointD0MinViz.setPosition(framePointD0);
         
         framePointD0.setZ(z_d0Max);
         pointD0MaxViz.setPosition(framePointD0);
         
         FramePoint framePointDF = FramePoint.morph(transferFromContactFramePosition, transferToContactFramePosition, (s_df)/sF);
         framePointDF.setZ(z_dF);
         pointDFViz.setPosition(framePointDF);
         
         framePointDF.setZ(z_dFMin);
         pointDFMinViz.setPosition(framePointDF);
         
         framePointDF.setZ(z_dFMax);
         pointDFMaxViz.setPosition(framePointDF);
         
         FramePoint framePointSF = new FramePoint(transferToContactFramePosition);
         framePointSF.setZ(zF);
         pointSFViz.setPosition(framePointSF);     
         
         framePointSF.setZ(zFMin);
         pointSFMinViz.setPosition(framePointSF);     
         
         framePointSF.setZ(zFMax);
         pointSFMaxViz.setPosition(framePointSF);     
         
         if (nextContactFramePosition != null)
         {
            FramePoint framePointSNext = new FramePoint(nextContactFramePosition);
            framePointSNext.setZ(zNext);
            pointSNextViz.setPosition(framePointSNext);     

            framePointSNext.setZ(zNextMin);
            pointSNextMinViz.setPosition(framePointSNext);     

            framePointSNext.setZ(zNextMax);
            pointSNextMaxViz.setPosition(framePointSNext);     
         }
         else
         {
            pointSNextViz.setPositionToNaN();
            pointSNextMinViz.setPositionToNaN();
            pointSNextMaxViz.setPositionToNaN();
         }
         
         bagOfBalls.reset();
         int numberOfPoints = 30;
         CoMHeightPartialDerivativesData coMHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();
         for (int i=0; i<numberOfPoints; i++)
         {
            FramePoint framePoint = FramePoint.morph(transferFromContactFramePosition, transferToContactFramePosition, ((double) i)/((double) numberOfPoints));
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
   
//   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, RobotSide supportLeg, Footstep nextFootstep, List<PlaneContactState> contactStates)
//   {
//      FramePoint[] contactFramePositions = getContactStateCenters(contactStates, nextFootstep);
//      
//      contactFrameZeroPosition.set(contactFramePositions[0]);
//      contactFrameOnePosition.set(contactFramePositions[1]);
//      
//      projectionSegment = new LineSegment2d(getPoint2d(contactFramePositions[0]), getPoint2d(contactFramePositions[1]));
//      double s0 = 0.0;
//      double sF = projectionSegment.length();
//      
//      double s_d0 = doubleSupportPercentageIn.getDoubleValue() * sF;
//      double s_df = (1.0 - doubleSupportPercentageIn.getDoubleValue()) * sF;
//
//      double footHeight0 = contactFramePositions[0].getZ();
//      double footHeight1 = contactFramePositions[1].getZ();
//      
//      double z0Min = footHeight0 + minimumHeightAboveGround.getDoubleValue();
//      double zFMin = footHeight1 + minimumHeightAboveGround.getDoubleValue();
//      
//      double z0Max = footHeight0 + maximumHeightAboveGround.getDoubleValue();
//      double zFMax = footHeight1 + maximumHeightAboveGround.getDoubleValue();
//      
//      double z_d0Min = findMinimumDoubleSupportHeight(s0, sF, s_d0, footHeight0, footHeight1);
//      double z_dFMin = findMinimumDoubleSupportHeight(s0, sF, s_df, footHeight0, footHeight1);
//      
//      double z_d0Max = findMaximumDoubleSupportHeight(s0, sF, s_d0, footHeight0, footHeight1);
//      double z_dFMax = findMaximumDoubleSupportHeight(s0, sF, s_df, footHeight0, footHeight1);
//  
//      double z0 = z0Max;
//      double z_d0 = z_d0Max;
//      double z_dF = z_dFMax;
//      double zF = zFMax;
//
//      z_d0 = clipToMaximum(z_d0, z_dF + maximumHeightDeltaBetweenWaypoints.getDoubleValue());
//      z_dF = clipToMaximum(z_dF, z_d0 + maximumHeightDeltaBetweenWaypoints.getDoubleValue());
//      
//      z0 = clipToMaximum(z0, z_d0 + maximumHeightDeltaBetweenWaypoints.getDoubleValue());
//      zF = clipToMaximum(zF, z_dF + maximumHeightDeltaBetweenWaypoints.getDoubleValue());
//
//      Point2d pointS0 = new Point2d(s0, z0);
//      Point2d pointSF = new Point2d(sF, zF);
//      Point2d pointD0 = new Point2d(s_d0, z_d0);
//      Point2d pointDF = new Point2d(s_df, z_dF);
//      
//      Point2d[] points = new Point2d[] {pointS0, pointD0, pointDF, pointSF};
//      double[] endpointSlopes = new double[] {0.0, 0.0};
//      
//      double[] waypointSlopes = new double[2];
//      waypointSlopes[0] = (points[2].y - points[0].y) / (points[2].x - points[0].x);
//      waypointSlopes[1] = (points[3].y - points[1].y) / (points[3].x - points[1].x);
//      
//      spline.setPoints(points, endpointSlopes, waypointSlopes);
//      
//      if (VISUALIZE)
//      {
//         FramePoint framePointD0 = FramePoint.morph(contactFramePositions[0], contactFramePositions[1], s_d0/sF);
//         framePointD0.setZ(z_d0);
//         pointD0Viz.setPosition(framePointD0);
//         
//         framePointD0.setZ(z_d0Min);
//         pointD0MinViz.setPosition(framePointD0);
//         
//         framePointD0.setZ(z_d0Max);
//         pointD0MaxViz.setPosition(framePointD0);
//         
//         FramePoint framePointDF = FramePoint.morph(contactFramePositions[0], contactFramePositions[1], (s_df)/sF);
//         framePointDF.setZ(z_dF);
//         pointDFViz.setPosition(framePointDF);
//         
//         framePointDF.setZ(z_dFMin);
//         pointDFMinViz.setPosition(framePointDF);
//         
//         framePointDF.setZ(z_dFMax);
//         pointDFMaxViz.setPosition(framePointDF);
//         
//         FramePoint framePointS0 = new FramePoint(contactFramePositions[0]);
//         framePointS0.setZ(z0);
//         pointS0Viz.setPosition(framePointS0);
//         
//         framePointS0.setZ(z0Min);
//         pointS0MinViz.setPosition(framePointS0);
//         
//         framePointS0.setZ(z0Max);
//         pointS0MaxViz.setPosition(framePointS0);
//         
//         FramePoint framePointSF = new FramePoint(contactFramePositions[1]);
//         framePointSF.setZ(zF);
//         pointSFViz.setPosition(framePointSF);     
//         
//         framePointSF.setZ(zFMin);
//         pointSFMinViz.setPosition(framePointSF);     
//         
//         framePointSF.setZ(zFMax);
//         pointSFMaxViz.setPosition(framePointSF);     
//         
//         bagOfBalls.reset();
//         int numberOfPoints = 30;
//         CoMHeightPartialDerivativesData coMHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();
//         for (int i=0; i<numberOfPoints; i++)
//         {
//            FramePoint framePoint = FramePoint.morph(contactFramePositions[0], contactFramePositions[1], ((double) i)/((double) numberOfPoints));
//            Point2d queryPoint = new Point2d(framePoint.getX(), framePoint.getY());
//            this.solve(coMHeightPartialDerivativesData, queryPoint);
//            FramePoint framePointToPack = new FramePoint();
//            coMHeightPartialDerivativesData.getCoMHeight(framePointToPack);
//            framePointToPack.setX(framePoint.getX());
//            framePointToPack.setY(framePoint.getY());
//            
//            bagOfBalls.setBallLoop(framePointToPack);
//         }
//      }
//   }
//   
   private double clipToMaximum(double value, double maxValue)
   {
      if (value < maxValue) return value;
      else return maxValue;
   }

   public double findMinimumDoubleSupportHeight(double s0, double sF, double s_d0, double foot0Height, double foot1Height)
   {
      double z_d0_A = foot0Height + Math.sqrt(MathTools.square(minimumHeightAboveGround.getDoubleValue()) - MathTools.square(s_d0 - s0));
      double z_d0_B = foot1Height + Math.sqrt(MathTools.square(minimumHeightAboveGround.getDoubleValue()) - MathTools.square((sF - s_d0)));
      double z_d0 = Math.min(z_d0_A, z_d0_B);
      return z_d0;
   }
   
   public double findMaximumDoubleSupportHeight(double s0, double sF, double s_d0, double foot0Height, double foot1Height)
   {
      double z_d0_A = foot0Height + Math.sqrt(MathTools.square(maximumHeightAboveGround.getDoubleValue()) - MathTools.square(s_d0 - s0));
      double z_d0_B = foot1Height + Math.sqrt(MathTools.square(maximumHeightAboveGround.getDoubleValue()) - MathTools.square((sF - s_d0)));
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

   private void solve(CoMHeightPartialDerivativesData coMHeightPartialDerivativesDataToPack, Point2d queryPoint)
   {
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

      return new double[] {dsdx, dsdy};
   }

   private FramePoint[] getContactStateCenters(List<PlaneContactState> contactStates, Footstep nextFootstep)
   {
      ReferenceFrame bodyFrame0 = contactStates.get(0).getBodyFrame();
//      contactFrameZero.setToReferenceFrame(bodyFrame0);
      
      FramePoint contactFramePosition0 = new FramePoint(bodyFrame0);
      contactFramePosition0.changeFrame(worldFrame);
      FramePoint contactFramePosition1;
      if (nextFootstep == null)
      {
         if (contactStates.size() != 2) throw new RuntimeException("contactStates.size() != 2");
         ReferenceFrame bodyFrame1 = contactStates.get(1).getBodyFrame();
//         contactFrameOne.setToReferenceFrame(bodyFrame1);

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
      return new FramePoint[]{contactFramePosition0, contactFramePosition1};
   }

   private Point2d getCenterOfMass2d(ReferenceFrame centerOfMassFrame)
   {
      FramePoint coM = new FramePoint(centerOfMassFrame);
      coM.changeFrame(worldFrame);

      return getPoint2d(coM);
   }
}


