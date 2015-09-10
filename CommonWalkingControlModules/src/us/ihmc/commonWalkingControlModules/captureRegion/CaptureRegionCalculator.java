package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.time.GlobalTimer;



//import us.ihmc.plotting.shapes.PointArtifact;

/**
 * <p>Title: YoboticsBipedCaptureRegionCalculator</p>
 *
 * <p>Description: Computes the Capture Region, using the Capture Point, the line of sight vertices on the support foot, and the swing time remaining.</p>
 *
 * <p>Copyright: Copyright (c) 2009</p>
 *
 * <p>Company: </p>
 *
 * @author Yobotics-IHMC biped team (with an assist by Michigan)
 * @version 1.0
 */
public class CaptureRegionCalculator
{
   // Warning! The following may not be rewindable if not made a Yo...
   // private FrameConvexPolygon2d captureRegion;
   private final CaptureRegionMathTools captureRegionMath = new CaptureRegionMathTools();
   YoVariableRegistry registry = new YoVariableRegistry("captureRegion");

   private final SideDependentList<FrameConvexPolygon2d> reachableRegions;

   private final YoFrameConvexPolygon2d captureRegionGraphic;
   private final YoFramePoint[] captureRegionBestCaseVertices;
   private final YoFramePoint[] captureRegionKinematicLimitVertices;
   private final YoFramePoint[] estimatedCOPExtremes;

   private final YoFramePoint[] additionalKinematicLimitPoints;

   private final ReferenceFrame worldFrame;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final CapturePointCalculatorInterface capturePointCalculator;

   public static boolean DRAW_CAPTURE_REGION = true;    //
   public static final double DRAWN_POINT_BASE_SIZE = 0.004;

   private final DoubleYoVariable kinematicRangeFromContactReferencePoint;
   public static final int NUMBER_OF_POINTS_TO_APPROXIMATE_KINEMATIC_LIMITS = 5;    // 3; //1; //10; //
   public static final int MAX_CAPTURE_REGION_POLYGON_POINTS = 26;    // 20;    // 4 + NUMBER_OF_POINTS_TO_APPROXIMATE_KINEMATIC_LIMITS + 8;

   public static final double SWING_TIME_TO_ADD_FOR_CAPTURING_SAFETY_FACTOR = 0.001; // want this to be zero, but then rewindability tests fail due to numVertices not being the same; // 0.05;    // 0.1; //

// public static final double FINAL_CAPTURE_REGION_SAFETY_MARGIN = 0.5; //0.1; //

//   private RobotSide currentSide = null;
//   private int index = 0;
//   private boolean DRAW_SCORE_ON_GROUND = false;    // true;

//   private final StepLocationScorer weightedDistanceScorer;

   private GlobalTimer globalTimer;

   
   public CaptureRegionCalculator(SideDependentList<ReferenceFrame> ankleZUpFrames, double midFootAnkleXOffset, double footWidth, double kinematicRangeFromContactReferencePoint,
                                  CapturePointCalculatorInterface capturePointCalculator, YoVariableRegistry yoVariableRegistry,
                                  YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      globalTimer = new GlobalTimer("captureRegionCalculator", registry);

      this.worldFrame = ReferenceFrame.getWorldFrame();
      this.ankleZUpFrames = ankleZUpFrames;

      this.capturePointCalculator = capturePointCalculator;
      this.kinematicRangeFromContactReferencePoint = new DoubleYoVariable("kinematicRangeFromContactReferencePoint", registry);
      this.kinematicRangeFromContactReferencePoint.set(kinematicRangeFromContactReferencePoint);
      
      int numPoints = MAX_CAPTURE_REGION_POLYGON_POINTS - 1;
      ArrayList<Point2d> reachableRegionPoints = new ArrayList<Point2d>(numPoints + 1);
      double radius = kinematicRangeFromContactReferencePoint;

      for (int i = 0; i < numPoints; i++)
      {
         double angle = -0.03 * Math.PI - 0.70 * Math.PI * (i) / ((numPoints - 1));
         reachableRegionPoints.add(computeReachablePoint(angle, radius, midFootAnkleXOffset));
      }

      reachableRegionPoints.add(new Point2d(0.0, -footWidth / 2.0));
      FrameConvexPolygon2d leftReachableRegionInSupportFootFrame = new FrameConvexPolygon2d(ankleZUpFrames.get(RobotSide.LEFT), reachableRegionPoints);

      reachableRegionPoints = new ArrayList<Point2d>(numPoints + 1);

      for (int i = 0; i < numPoints; i++)
      {
         double angle = 0.03 * Math.PI + 0.70 * Math.PI * (i) / ((numPoints - 1));

         reachableRegionPoints.add(computeReachablePoint(angle, radius, midFootAnkleXOffset));
      }

      reachableRegionPoints.add(new Point2d(0.0, footWidth / 2.0));
      FrameConvexPolygon2d rightReachableRegionInSupportFootFrame = new FrameConvexPolygon2d(ankleZUpFrames.get(RobotSide.RIGHT), reachableRegionPoints);

      this.reachableRegions = new SideDependentList<FrameConvexPolygon2d>(leftReachableRegionInSupportFootFrame, rightReachableRegionInSupportFootFrame);


      captureRegionBestCaseVertices = new YoFramePoint[3];
      captureRegionKinematicLimitVertices = new YoFramePoint[3];
      estimatedCOPExtremes = new YoFramePoint[3];
      additionalKinematicLimitPoints = new YoFramePoint[NUMBER_OF_POINTS_TO_APPROXIMATE_KINEMATIC_LIMITS];

      // Set up the scoring function:
//    DoubleYoVariable stanceWidthForScore = new DoubleYoVariable("stanceWidthForScore", "Stance width for the scoring function.", registry);
//    DoubleYoVariable stanceLengthForScore = new DoubleYoVariable("stanceLengthForScore", "Stance length for the scoring function.", registry);
//    DoubleYoVariable stepAngleForScore = new DoubleYoVariable("stepAngleForScore", "Step angle for the scoring function.", registry);
//    DoubleYoVariable stepDistanceForScore = new DoubleYoVariable("stepDistanceForScore", "Step distance for the scoring function.", registry);
//
//    stanceWidthForScore.set(0.22);
//    stanceLengthForScore.set(M2V2CaptureRegionCalculator.KINEMATIC_RANGE_FROM_COP * 0.7);    // 0.40
//    stepAngleForScore.set(Math.atan(stanceWidthForScore.getDoubleValue() / stanceLengthForScore.getDoubleValue()));
//    stepDistanceForScore.set(Math.sqrt((stanceWidthForScore.getDoubleValue() * stanceWidthForScore.getDoubleValue())
//                                       + (stanceLengthForScore.getDoubleValue() * stanceLengthForScore.getDoubleValue())));

//    weightedDistanceScorer = new WeightedDistanceScorer(this, footZUpFrames, stanceWidthForScore, stanceLengthForScore, stepAngleForScore,
//            stepDistanceForScore);



      // Done setting up the scoring function

      YoGraphicsList yoGraphicsList = null;
      ArtifactList artifactList = null;
      captureRegionGraphic = new YoFrameConvexPolygon2d("captureRegion", "", worldFrame, MAX_CAPTURE_REGION_POLYGON_POINTS, registry);

      if (yoGraphicsListRegistry == null)
      {
         DRAW_CAPTURE_REGION = false;
      }

      if (DRAW_CAPTURE_REGION && (yoGraphicsListRegistry != null))
      {
         yoGraphicsList = new YoGraphicsList("CaptureRegionCalculator");
         artifactList = new ArtifactList("CaptureRegionCalculator");

         YoArtifactPolygon dynamicGraphicYoPolygonArtifact = new YoArtifactPolygon("CaptureRegion", captureRegionGraphic,
                                                                              Color.LIGHT_GRAY, false);
         artifactList.add(dynamicGraphicYoPolygonArtifact);
      }



      for (int i = 0; i < captureRegionBestCaseVertices.length; i++)
      {
         String pointName = "captureRegionBestCaseVertex" + i;
         captureRegionBestCaseVertices[i] = new YoFramePoint(pointName, "", worldFrame, registry);

         if (DRAW_CAPTURE_REGION && (yoGraphicsListRegistry != null))
         {
            YoGraphicPosition position = new YoGraphicPosition("BestCaseVertices" + i, 
                  captureRegionBestCaseVertices[i], 
                  0.004,
//                  DRAWN_POINT_BASE_SIZE * ((4.0 + ((double) i)) / 4.0),
                  YoAppearance.Green(), YoGraphicPosition.GraphicType.BALL);
            yoGraphicsList.add(position);
            artifactList.add(position.createArtifact());
         }
      }

      for (int i = 0; i < captureRegionKinematicLimitVertices.length; i++)
      {
         String pointName = "captureRegionKinematicLimitVertex" + i;
         captureRegionKinematicLimitVertices[i] = new YoFramePoint(pointName, "", worldFrame, registry);

         if (DRAW_CAPTURE_REGION && (yoGraphicsListRegistry != null))
         {
            YoGraphicPosition position = new YoGraphicPosition(pointName, captureRegionKinematicLimitVertices[i],
                  0.004, 
//                  DRAWN_POINT_BASE_SIZE * ((4 + i) / 4), 
                  YoAppearance.Blue(), 
                  YoGraphicPosition.GraphicType.BALL);
            yoGraphicsList.add(position);
            artifactList.add(position.createArtifact());
         }
      }

      for (int i = 0; i < estimatedCOPExtremes.length; i++)
      {
         String pointName = "estimatedCOPExtreme" + i;
         estimatedCOPExtremes[i] = new YoFramePoint(pointName, "", worldFrame, registry);

         if (DRAW_CAPTURE_REGION && (yoGraphicsListRegistry != null))
         {
            YoGraphicPosition position = new YoGraphicPosition(pointName, estimatedCOPExtremes[i], 
                  0.004,
//                  DRAWN_POINT_BASE_SIZE * ((4 + i) / 4),
                  YoAppearance.Black(), 
                  YoGraphicPosition.GraphicType.BALL);
            yoGraphicsList.add(position);
            artifactList.add(position.createArtifact());
         }
      }


      for (int i = 0; i < additionalKinematicLimitPoints.length; i++)
      {
         String pointName = "additionalKinematicLimitPoint" + i;
         additionalKinematicLimitPoints[i] = new YoFramePoint(pointName, "", worldFrame, registry);

         if (DRAW_CAPTURE_REGION && (yoGraphicsListRegistry != null))
         {
            YoGraphicPosition position = new YoGraphicPosition(pointName, additionalKinematicLimitPoints[i], DRAWN_POINT_BASE_SIZE * 0.5,
                                                 YoAppearance.Aqua(), YoGraphicPosition.GraphicType.BALL);

            yoGraphicsList.add(position);
            artifactList.add(position.createArtifact());
         }
      }

      if (DRAW_CAPTURE_REGION && (yoGraphicsListRegistry != null))
      {
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);
      }

      if (yoVariableRegistry != null)
      {
         yoVariableRegistry.addChild(registry);
      }
   }

   private Point2d computeReachablePoint(double angle, double radius, double midFootAnkleXOffset)
   {
      double x = radius * Math.cos(angle) + midFootAnkleXOffset;
      double y = radius * Math.sin(angle);
      
      return new Point2d(x, y);
   }
   
   public void setKinematicRangeFromFootCenter(double kinematicRangeFromFootCenter)
   {
      kinematicRangeFromContactReferencePoint.set(kinematicRangeFromFootCenter);
   }
   
   public double getKinematicRangeFromContactReferencePoint()
   {
      return kinematicRangeFromContactReferencePoint.getDoubleValue();
   }

   private final FramePoint2d footCentroid = new FramePoint2d(ReferenceFrame.getWorldFrame());
   private final FramePoint2d copExtremeInSupportAnkleZUp = new FramePoint2d(ReferenceFrame.getWorldFrame());
   private final FramePoint copExtreme3d = new FramePoint(ReferenceFrame.getWorldFrame());
   
   public FrameConvexPolygon2d calculateCaptureRegion(RobotSide supportLeg, FrameConvexPolygon2d supportFoot, double swingTimeRemaining)
   {      
      // The general idea is to predict where we think we can drive the capture point
      // to by the end of the swing (as determined just by swing time), given our current COM state and support foot. We first
      // find the extremes on the foot where we can put the COP, then predict where each extreme would drive us.
      // We will also consider the points between the line of sight points to get the inner points of the polygon.
      // Since the foot is convex, these lines should be convex too.
      // If we step
      // in front of these points, we would fall to the outside of the swing, so we have to step beyond these points. The
      // limits on the extent we can reach to is dominated by kinematics, so we will just choose a conservative distance to
      // specify the distal constraints of the capture region. See Jerry Pratt's or John Rebula's notes from 2009.1.4.
      // We also will have a reachable region that we intersect with. If the Capture Point is inside the foot, then we'll just
      // return the reachable region.

      globalTimer.startTimer();
      swingTimeRemaining = MathTools.clipToMinMax(swingTimeRemaining, 0.0, Double.POSITIVE_INFINITY);
      ReferenceFrame supportAnkleZUpFrame = ankleZUpFrames.get(supportLeg);

      // first get all of the objects we will need to calculate the capture region
      FramePoint2d capturePoint = capturePointCalculator.getCapturePoint2dInFrame(supportAnkleZUpFrame);

//      footCentroid.set(supportAnkleZUpFrame, 0.0, 0.0);
      supportFoot.getCentroid(footCentroid);
      
      // 0. hmm, weird things are happening when we predict the capture point given the cop extremes as calculated by
      // the line of sightlines from the capture point when the capture point is close to the foot polygon. Let's try returning
      // no capture region when the center of mass is still over the support polygon

//    FramePoint com = processedSensors.centerOfMassBodyZUp.getFramePointCopy().changeFrameCopy(supportAnkleZUpFrame);
//    FramePoint2d com2d = new FramePoint2d(com.getReferenceFrame(), com.getX(), com.getY());
//    FramePoint2d[] extremesOfFeasibleCOPFromCOM = supportFoot.getLineOfSightVertices(com2d);
//    if(extremesOfFeasibleCOPFromCOM == null)
//    {
//       return null;
//    }

      // 1. find visible points on polygon...
//    FramePoint2d[] extremesOfFeasibleCOP = supportFoot.getLineOfSightVertices(capturePoint);

      
      ArrayList<FramePoint2d> extremesOfFeasibleCOP = supportFoot.getAllVisibleVerticesFromOutsideLeftToRight(capturePoint);
      
      if (extremesOfFeasibleCOP == null)    // Inside the polygon, Capture Region is everywhere reachable. Make it reachable region...
      {
         FrameConvexPolygon2d captureRegion = reachableRegions.get(supportLeg);

         if (DRAW_CAPTURE_REGION)
         {
            FrameConvexPolygon2d captureRegionInWorld = new FrameConvexPolygon2d(captureRegion);
            captureRegionInWorld.changeFrame(worldFrame);
            captureRegionGraphic.setFrameConvexPolygon2d(captureRegionInWorld);
         }
         globalTimer.stopTimer();

         return captureRegion;
      }


      // 2. predict extreme capture points
      FrameVector2d[] directionLimits = new FrameVector2d[extremesOfFeasibleCOP.size()];
      ArrayList<FramePoint2d> captureRegionVertices = new ArrayList<FramePoint2d>();
      for (int i = 0; i < extremesOfFeasibleCOP.size(); i++)
      {
         FramePoint2d copExtremeInWorld = new FramePoint2d(extremesOfFeasibleCOP.get(i));
         copExtremeInWorld.changeFrame(worldFrame);

         if (i < estimatedCOPExtremes.length)
            estimatedCOPExtremes[i].set(copExtremeInWorld.getX(), copExtremeInWorld.getY(), 0.0);

         FramePoint2d copExtreme = extremesOfFeasibleCOP.get(i);
         copExtremeInSupportAnkleZUp.setIncludingFrame(copExtreme);
         copExtremeInSupportAnkleZUp.changeFrame(supportAnkleZUpFrame);
         
         copExtreme3d.setIncludingFrame(copExtreme.getReferenceFrame(), copExtreme.getX(), copExtreme.getY(), 0.0);

         FramePoint predictedExtremeCapturePoint = capturePointCalculator.computePredictedCapturePoint(supportLeg, swingTimeRemaining + SWING_TIME_TO_ADD_FOR_CAPTURING_SAFETY_FACTOR, copExtreme3d);
         
         predictedExtremeCapturePoint.changeFrame(supportAnkleZUpFrame);
         
//         FramePoint predictedExtremeCapturePoint = capturePointCalculator.getPredictedCapturePointInFrame(supportAnkleZUpFrame);
         FramePoint2d predictedExtremeCapturePoint2d = new FramePoint2d(predictedExtremeCapturePoint.getReferenceFrame(), predictedExtremeCapturePoint.getX(),
                                                          predictedExtremeCapturePoint.getY());

         captureRegionVertices.add(predictedExtremeCapturePoint2d);

         // update position in plotter
         if (DRAW_CAPTURE_REGION)
         {
            FramePoint predictedCapturePointInWorld = new FramePoint(predictedExtremeCapturePoint);
            predictedCapturePointInWorld.changeFrame(worldFrame);
            if (i < captureRegionBestCaseVertices.length)
               captureRegionBestCaseVertices[i].set(predictedCapturePointInWorld);
         }

         // 3. project from cop extremes to capture point extremes, find point at a given distance to determine kinematic limits of the capture region.

         FrameVector2d projectedLine = new FrameVector2d(predictedExtremeCapturePoint2d);

         projectedLine.sub(copExtremeInSupportAnkleZUp);

         // Look at JPratt Notes February 18, 2009 for details on the following:
         FramePoint2d kinematicExtreme = solveIntersectionOfRayAndCircle(footCentroid, predictedExtremeCapturePoint2d, projectedLine, kinematicRangeFromContactReferencePoint.getDoubleValue());

         if (kinematicExtreme == null)

         {
            FrameConvexPolygon2d captureRegion = null;

            if (DRAW_CAPTURE_REGION)
               captureRegionGraphic.setFrameConvexPolygon2d(null);
            globalTimer.stopTimer();

            return captureRegion;
         }


//       // Don't invert the Capture Region. IF the projected line isn't kinematically reachable, then the Capture Region is null.
//       if (projectedLine.lengthSquared() > KINEMATIC_RANGE_FROM_COP * KINEMATIC_RANGE_FROM_COP)
//       {
//          FrameConvexPolygon2d captureRegion = null;
//
//          if (DRAW_CAPTURE_REGION) captureRegionGraphic.setFrameConvexPolygon2d(null);
//          globalTimer.stopTimer();
//          return captureRegion;
//       }
//
//       projectedLine.normalize();
//       projectedLine.scale(KINEMATIC_RANGE_FROM_COP);
//       projectedLine.add(copExtremeInSupportAnkleZUp);
//
//       FramePoint2d kinematicExtreme = new FramePoint2d(projectedLine.getReferenceFrame(), projectedLine.getX(), projectedLine.getY());
         captureRegionVertices.add(kinematicExtreme);

         FrameVector2d footToKinematicLimitDirection = new FrameVector2d(kinematicExtreme);
         footToKinematicLimitDirection.sub(footCentroid);
         directionLimits[i] = footToKinematicLimitDirection;

         // update position in plotter
         if (DRAW_CAPTURE_REGION)
         {
            FramePoint2d kinematicExtremeInWorld = new FramePoint2d(kinematicExtreme);
            kinematicExtremeInWorld.changeFrame(worldFrame);

            if (i < captureRegionKinematicLimitVertices.length)
               captureRegionKinematicLimitVertices[i].set(kinematicExtremeInWorld.getX(), kinematicExtremeInWorld.getY(), 0.0);
         }
      }

      // we want to add a certain number of points to form the kinematic perimeter. We will compute these by approximating a sector centered
      // at the foot center with a given radius.
      for (int i = 0; i < NUMBER_OF_POINTS_TO_APPROXIMATE_KINEMATIC_LIMITS; i++)
      {
         double alphaFromAToB = ((double) (i + 1)) / ((double) (NUMBER_OF_POINTS_TO_APPROXIMATE_KINEMATIC_LIMITS + 1));
         FramePoint2d additionalKinematicPoint = getPointBetweenVectorsAtDistanceFromOriginCircular(directionLimits[0], directionLimits[directionLimits.length-1], alphaFromAToB,
                                                    kinematicRangeFromContactReferencePoint.getDoubleValue(), footCentroid);
         captureRegionVertices.add(new FramePoint2d(additionalKinematicPoint));
         additionalKinematicPoint.changeFrame(worldFrame);
         additionalKinematicLimitPoints[i].set(additionalKinematicPoint.getX(), additionalKinematicPoint.getY(), 0.0);
      }

      // connect the 4 dots (2 extreme predicted proximal capture points, 2 distal kinematic limit points) + the extra kinematic dots
      FrameConvexPolygon2d captureRegion = new FrameConvexPolygon2d(captureRegionVertices);

      // Intersect this with the reachableRegion:
      FrameConvexPolygon2d reachableRegion = reachableRegions.get(supportLeg);
      captureRegion = captureRegion.intersectionWith(reachableRegion);

      
//    if (DRAW_SCORE_ON_GROUND && (captureRegion != null))
//    {
//       if (currentSide != supportLeg)
//       {
//          // clear the old
//          for (double i = 0; i < index; i++)
//          {
//             YoboticsBipedPlotter.deregisterArtifactNoRepaint("Point" + index);
//          }
//
//          YoboticsBipedPlotter.repaint();
//          index = 0;
//
//          currentSide = supportLeg;
//          Point2d minPoint = new Point2d();
//          Point2d maxPoint = new Point2d();
//          BoundingBox2d boundingBox = captureRegion.getBoundingBox();
//
//          boundingBox.getMinPoint(minPoint);
//          boundingBox.getMaxPoint(maxPoint);
//
//          // add colored points for score
//          for (double i = minPoint.getX(); i <= maxPoint.getX(); i = i + 0.01)
//          {
//             for (double j = minPoint.getY(); j <= maxPoint.getY(); j = j + 0.01)
//             {
//                Point2d desiredPoint = new Point2d(i, j);
//                FramePoint desiredFootLocation = new FramePoint(ankleZUpFrames.get(supportLeg), desiredPoint.getX(),
//                                                    desiredPoint.getY(), 0.0);
//                Footstep desiredFootstep = new Footstep(supportLeg, desiredFootLocation, 0.0);
//                double stepLocationScore = weightedDistanceScorer.getStepLocationScore(supportLeg, desiredFootstep);
//
//                if (stepLocationScore != 0.0)
//                {
//                   FramePoint desiredFootLocationInWorld = desiredFootLocation.changeFrameCopy(ReferenceFrame.getWorldFrame());
//                   us.ihmc.plotting.shapes.PointArtifact point1 = new us.ihmc.plotting.shapes.PointArtifact("Point" + index,
//                                                                     new Point2d(desiredFootLocationInWorld.getX(), desiredFootLocationInWorld.getY()));
//                   double colorIndex = (1.0 - (stepLocationScore * 0.6));    // 0.6 to lighten the color a bit
//                   point1.setColor(new Color((float) 0.0, (float) colorIndex, (float) 0.0, (float) 0.4));
//                   point1.setLevel(86);
//                   YoboticsBipedPlotter.registerArtifactNoRepaint(point1);
//                   index++;
//                }
//             }
//          }
//
//          YoboticsBipedPlotter.repaint();
//       }
//    }

//    // now shrink that polygon
//    ArrayList<FramePoint2d> shrunkPoints = new ArrayList<FramePoint2d>();
//    for (int i = 0; i < captureRegionVertices.size(); i++)
//    {
//     FramePoint2d shrunkPoint = new FramePoint2d(captureRegionVertices.get(i));
//     captureRegionFromPoints.pullPointTowardsCentroid(shrunkPoint,
//                                                      captureRegionVertices.get(i).distance(captureRegionFromPoints.getCentroidCopy()) *
//                                                      FINAL_CAPTURE_REGION_SAFETY_MARGIN);
//     shrunkPoints.add(shrunkPoint);
//    }
//    captureRegionFromPoints = new FrameConvexPolygon2d(shrunkPoints);

      if (DRAW_CAPTURE_REGION)
      {
         if (captureRegion == null)
            captureRegionGraphic.setFrameConvexPolygon2d(null);
         else
         {
            FrameConvexPolygon2d captureRegionInWorld = new FrameConvexPolygon2d(captureRegion);
            captureRegionInWorld.changeFrame(worldFrame);
            captureRegionGraphic.setFrameConvexPolygon2d(captureRegionInWorld);
         }
      }

      globalTimer.stopTimer();

      return captureRegion;
   }


   public FrameConvexPolygon2d getReachableRegion(RobotSide supportSide)
   {
      return reachableRegions.get(supportSide);
   }



   private static FramePoint2d solveIntersectionOfRayAndCircle(FramePoint2d pointA, FramePoint2d pointB, FrameVector2d vector, double R)
   {
      FramePoint2d ret = new FramePoint2d();
      CaptureRegionMathTools.solveIntersectionOfRayAndCircle(pointA, pointB, vector, R, ret);
      return ret;
   }

   /**
    * Finds a point that lies at a given distance along a direction vector from a given origin. The direction vector is specified
    * by two extreme direction vectors, a and b, along with an alpha. If the alpha is zero, direction a is used, if alpha is 1,
    * direction b is used. An intermediate alpha will yield a direction vector morphed from a to b by the direction value.
    * @param directionA FrameVector2d
    * @param directionB FrameVector2d
    * @param alphaFromAToB double
    * @param distance double
    * @param origin FramePoint2d
    * @return FramePoint2d
    */
   public static FramePoint2d getPointBetweenVectorsAtDistanceFromOrigin(FrameVector2d directionA, FrameVector2d directionB, double alphaFromAToB,
           double distance, FramePoint2d origin)
   {
      directionA.checkReferenceFrameMatch(directionB.getReferenceFrame());
      directionA.checkReferenceFrameMatch(origin.getReferenceFrame());
      alphaFromAToB = MathTools.clipToMinMax(alphaFromAToB, 0.0, 1.0);

      FrameVector2d aToB = new FrameVector2d(directionB);
      aToB.sub(directionA);

//    aToB.normalize();
      aToB.scale(alphaFromAToB);

      FrameVector2d directionVector = new FrameVector2d(directionA);
      directionVector.add(aToB);
      directionVector.normalize();

      directionVector.scale(distance);

      FramePoint2d ret = new FramePoint2d(directionVector);
      ret.add(origin);

      return ret;
   }


   public FramePoint2d getPointBetweenVectorsAtDistanceFromOriginCircular(FrameVector2d directionA, FrameVector2d directionB, double alphaFromAToB,
           double distance, FramePoint2d origin)
   {
      FramePoint2d ret = new FramePoint2d();
      captureRegionMath.getPointBetweenVectorsAtDistanceFromOriginCircular(directionA, directionB, alphaFromAToB, distance, origin, ret);
      return ret;
   }

   /**
    * hideCaptureRegion
    */
   public void hideCaptureRegion()
   {
      if (captureRegionGraphic != null)
      {
         this.captureRegionGraphic.setFrameConvexPolygon2d(null);

         for (int i = 0; i < captureRegionBestCaseVertices.length; i++)
         {
            captureRegionBestCaseVertices[i].set(Double.NaN, Double.NaN, Double.NaN);
         }

         for (int i = 0; i < captureRegionKinematicLimitVertices.length; i++)
         {
            captureRegionKinematicLimitVertices[i].set(Double.NaN, Double.NaN, Double.NaN);
         }

         for (int i = 0; i < estimatedCOPExtremes.length; i++)
         {
            estimatedCOPExtremes[i].set(Double.NaN, Double.NaN, Double.NaN);
         }

         for (int i = 0; i < additionalKinematicLimitPoints.length; i++)
         {
            additionalKinematicLimitPoints[i].set(Double.NaN, Double.NaN, Double.NaN);
         }
      }
   }

}
