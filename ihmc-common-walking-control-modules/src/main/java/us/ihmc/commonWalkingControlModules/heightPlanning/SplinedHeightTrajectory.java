package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.StringStretcher2d;
import us.ihmc.robotics.math.trajectories.YoConfigurablePolynomial;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class SplinedHeightTrajectory
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final List<CoMHeightTrajectoryWaypoint> waypoints = new ArrayList<>();
   private final Comparator<CoMHeightTrajectoryWaypoint> sorter = Comparator.comparingDouble(CoMHeightTrajectoryWaypoint::getX);

   private final CoMHeightTrajectoryWaypoint startWaypoint = new CoMHeightTrajectoryWaypoint();
   private final CoMHeightTrajectoryWaypoint firstMidpoint = new CoMHeightTrajectoryWaypoint();
   private final CoMHeightTrajectoryWaypoint secondMidpoint = new CoMHeightTrajectoryWaypoint();
   private final CoMHeightTrajectoryWaypoint thirdMidpoint = new CoMHeightTrajectoryWaypoint();
   private final CoMHeightTrajectoryWaypoint endWaypoint = new CoMHeightTrajectoryWaypoint();

   private final YoFramePoint3D contactFrameZeroPosition;
   private final YoFramePoint3D contactFrameOnePosition;

   private final BagOfBalls bagOfBalls;
   private final DoubleProvider waypointInterpolation;
   private final YoConfigurablePolynomial spline;

   private ReferenceFrame referenceFrame;

   private final CoMHeightPartialDerivativesData comHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();

   private final StringStretcher2d stringStretcher = new StringStretcher2d();
   private final List<Point2DBasics> stretchedStringWaypoints = new ArrayList<>();

   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final FramePoint3D tempFramePoint2 = new FramePoint3D();
   private final Point2D pointToThrowAway = new Point2D();

   public SplinedHeightTrajectory(DoubleProvider waypointInterpolation, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.waypointInterpolation = waypointInterpolation;

      spline = new YoConfigurablePolynomial("height", 8, registry);

      startWaypoint.createYoVariables("startWaypoint", registry);
      firstMidpoint.createYoVariables("firstMidpoint", registry);
      secondMidpoint.createYoVariables("secondMidpoint", registry);
      thirdMidpoint.createYoVariables("thirdMidpoint", registry);
      endWaypoint.createYoVariables("endWaypoint", registry);

      contactFrameZeroPosition = new YoFramePoint3D("contactFrameZeroPosition", worldFrame, registry);
      contactFrameOnePosition = new YoFramePoint3D("contactFrameOnePosition", worldFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
         String prefix = "better_";

         AppearanceDefinition startColor = YoAppearance.CadetBlue();
         AppearanceDefinition firstMidpointColor = YoAppearance.Chartreuse();
         AppearanceDefinition secondMidpointColor = YoAppearance.BlueViolet();
         AppearanceDefinition middleColor = YoAppearance.Azure();

         String graphicListName = "BetterCoMHeightTrajectoryGenerator";

         startWaypoint.setupViz(graphicListName, prefix + "StartWaypoint", startColor, yoGraphicsListRegistry);
         firstMidpoint.setupViz(graphicListName, prefix + "FirstMidpoint", firstMidpointColor, yoGraphicsListRegistry);
         secondMidpoint.setupViz(graphicListName, prefix + "SecondMidpoint", secondMidpointColor, yoGraphicsListRegistry);
         thirdMidpoint.setupViz(graphicListName, prefix + "ThirdMidpoint", secondMidpointColor, yoGraphicsListRegistry);
         endWaypoint.setupViz(graphicListName, prefix + "EndWaypoint", middleColor, yoGraphicsListRegistry);

         bagOfBalls = new BagOfBalls(15, 0.01, "height", registry, yoGraphicsListRegistry);
      }
      else
      {
         bagOfBalls = null;
      }
   }

   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public void computeWaypoints(FramePoint3DReadOnly startCoMPosition,
                                FramePoint3DReadOnly endCoMPosition,
                                double startGroundHeight,
                                double endGroundHeight,
                                double minimumHeight,
                                double maximumHeight)
   {
      double midstanceWidth = 0.5 * (startCoMPosition.getY() + endCoMPosition.getY());

      double startX = startCoMPosition.getX();
      double endX = endCoMPosition.getX();

      double firstMidpointX = InterpolationTools.linearInterpolate(startX, endX, waypointInterpolation.getValue());
      double secondMidpointX = InterpolationTools.linearInterpolate(startX, endX, 0.5);
      double thirdMidpointX = InterpolationTools.linearInterpolate(endX, startX, waypointInterpolation.getValue());

      setWaypointFrames(referenceFrame);

      startWaypoint.setXY(startX, midstanceWidth);
      firstMidpoint.setXY(firstMidpointX, midstanceWidth);
      secondMidpoint.setXY(secondMidpointX, midstanceWidth);
      thirdMidpoint.setXY(thirdMidpointX, midstanceWidth);
      endWaypoint.setXY(endX, midstanceWidth);

      startWaypoint.setHeight(startCoMPosition.getZ());
      endWaypoint.setHeight(endCoMPosition.getZ());

      double secondMinHeight = Math.max(findWaypointHeight(minimumHeight, startX, secondMidpointX, startGroundHeight),
                                        findWaypointHeight(minimumHeight, endX, secondMidpointX, endGroundHeight));
      double secondMaxHeight = Math.min(findWaypointHeight(maximumHeight, startX, secondMidpointX, startGroundHeight),
                                        findWaypointHeight(maximumHeight, endX, secondMidpointX, endGroundHeight));
      startWaypoint.setMinMax(minimumHeight, maximumHeight);
      firstMidpoint.setMinMax(findWaypointHeight(minimumHeight, startX, firstMidpointX, startGroundHeight),
                              findWaypointHeight(maximumHeight, startX, firstMidpointX, startGroundHeight));
      secondMidpoint.setMinMax(secondMinHeight, secondMaxHeight);
      thirdMidpoint.setMinMax(findWaypointHeight(minimumHeight, endX, thirdMidpointX, endGroundHeight),
                              findWaypointHeight(maximumHeight, endX, thirdMidpointX, endGroundHeight));
      endWaypoint.setMinMax(minimumHeight + endGroundHeight, maximumHeight + endGroundHeight);

      waypoints.clear();
      waypoints.add(startWaypoint);
      waypoints.add(firstMidpoint);
      waypoints.add(thirdMidpoint);
      waypoints.add(secondMidpoint);
      waypoints.add(endWaypoint);
      waypoints.sort(sorter);
   }

   private void setWaypointFrames(ReferenceFrame referenceFrame)
   {
      startWaypoint.setToZero(referenceFrame);
      firstMidpoint.setToZero(referenceFrame);
      secondMidpoint.setToZero(referenceFrame);
      thirdMidpoint.setToZero(referenceFrame);
      endWaypoint.setToZero(referenceFrame);
   }

   public void computeSpline()
   {
      computeHeightsToUseByStretchingString(waypoints);

      spline.reshape(waypoints.size());
      for (int i = 0; i < waypoints.size(); i++)
         spline.addPositionConstraint(waypoints.get(i).getX(), waypoints.get(i).getHeight());
      spline.solve();

      contactFrameZeroPosition.setMatchingFrame(waypoints.get(0).getWaypoint());
      contactFrameOnePosition.setMatchingFrame(waypoints.get(waypoints.size() - 1).getWaypoint());

      for (int i = 0; i < waypoints.size(); i++)
      {
         waypoints.get(i).update();
      }

      if (bagOfBalls != null)
      {
         bagOfBalls.reset();
         int numberOfPoints = bagOfBalls.getNumberOfBalls();

         for (int i = 0; i < numberOfPoints; i++)
         {
            double alpha = ((double) (i + 1)) / ((double) numberOfPoints);

            tempFramePoint2.interpolate(contactFrameZeroPosition, contactFrameOnePosition, alpha);
            this.solve(comHeightPartialDerivativesData, tempFramePoint2, pointToThrowAway);

            tempFramePoint2.checkReferenceFrameMatch(comHeightPartialDerivativesData.getFrameOfCoMHeight());
            tempFramePoint2.setZ(comHeightPartialDerivativesData.getComHeight());

            bagOfBalls.setBallLoop(tempFramePoint2);
         }
      }
   }

   private void computeHeightsToUseByStretchingString(List<CoMHeightTrajectoryWaypoint> waypoints)
   {
      CoMHeightTrajectoryWaypoint startWaypoint = waypoints.get(0);
      CoMHeightTrajectoryWaypoint lastWaypoint = waypoints.get(waypoints.size() - 1);
      stringStretcher.reset();
      stringStretcher.setStartPoint(startWaypoint.getX(), startWaypoint.getHeight());
      stringStretcher.setEndPoint(lastWaypoint.getX(), lastWaypoint.getHeight());

      for (int i = 1; i < waypoints.size() - 1; i++)
      {
         CoMHeightTrajectoryWaypoint waypoint = waypoints.get(i);
         stringStretcher.addMinMaxPoints(waypoint.getX(), waypoint.getMinHeight(), waypoint.getX(), waypoint.getMaxHeight());
      }

      stringStretcher.stretchString(stretchedStringWaypoints);

      for (int i = 0; i < waypoints.size(); i++)
      {
         CoMHeightTrajectoryWaypoint waypoint = waypoints.get(i);
         waypoint.setX(stretchedStringWaypoints.get(i).getX());
         waypoint.setHeight(stretchedStringWaypoints.get(i).getY());
      }
   }

   private static double findWaypointHeight(double desiredDistanceFromFoot, double startAnkleX, double queryX, double extraToeOffHeight)
   {
      return Math.sqrt(MathTools.square(desiredDistanceFromFoot) - MathTools.square(queryX - startAnkleX)) + extraToeOffHeight;
   }

   public double solve(CoMHeightPartialDerivativesData comHeightPartialDerivativesDataToPack, FramePoint3DBasics queryPoint, Point2DBasics pointOnSplineToPack)
   {
      EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(queryPoint, contactFrameZeroPosition, contactFrameOnePosition, queryPoint);
      double percentAlongSegment = EuclidGeometryTools.percentageAlongLineSegment3D(queryPoint, contactFrameZeroPosition, contactFrameOnePosition);

      CoMHeightTrajectoryWaypoint startWaypoint = waypoints.get(0);
      CoMHeightTrajectoryWaypoint endWaypoint = waypoints.get(waypoints.size() - 1);

      double splineQuery = InterpolationTools.linearInterpolate(startWaypoint.getX(), endWaypoint.getX(), percentAlongSegment);

      spline.compute(splineQuery);

      double z = spline.getPosition();
      double dzds = spline.getVelocity();
      double ddzdds = spline.getAcceleration();

      double length = startWaypoint.getWaypoint().distance(endWaypoint.getWaypoint());
      double dsdx = (endWaypoint.getX() - startWaypoint.getX()) / length;
      double dsdy = (endWaypoint.getHeight() - startWaypoint.getHeight()) / length;

      double ddsddx = 0;
      double ddsddy = 0;
      double ddsdxdy = 0;

      double dzdx = dsdx * dzds;
      double dzdy = dsdy * dzds;
      double ddzddx = dzds * ddsddx + ddzdds * dsdx * dsdx;
      double ddzddy = dzds * ddsddy + ddzdds * dsdy * dsdy;
      double ddzdxdy = ddzdds * dsdx * dsdy + dzds * ddsdxdy;

      tempFramePoint.setIncludingFrame(referenceFrame, 0.0, 0.0, z);
      tempFramePoint.changeFrame(worldFrame);
      comHeightPartialDerivativesDataToPack.setCoMHeight(worldFrame, tempFramePoint.getZ());
      comHeightPartialDerivativesDataToPack.setPartialDzDx(dzdx);
      comHeightPartialDerivativesDataToPack.setPartialDzDy(dzdy);
      comHeightPartialDerivativesDataToPack.setPartialD2zDxDy(ddzdxdy);
      comHeightPartialDerivativesDataToPack.setPartialD2zDx2(ddzddx);
      comHeightPartialDerivativesDataToPack.setPartialD2zDy2(ddzddy);

      pointOnSplineToPack.set(splineQuery, z);

      return percentAlongSegment;
   }
}
