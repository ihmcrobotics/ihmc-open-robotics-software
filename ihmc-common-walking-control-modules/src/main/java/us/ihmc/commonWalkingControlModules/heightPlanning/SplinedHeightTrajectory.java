package us.ihmc.commonWalkingControlModules.heightPlanning;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.trajectories.OptimizedTrajectoryGenerator;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.StringStretcher2d;
import us.ihmc.tools.lists.ListSorter;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SplinedHeightTrajectory
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double constraintWeight = 1000.0;

   private final List<CoMHeightTrajectoryWaypoint> waypoints = new ArrayList<>();
   private final Comparator<CoMHeightTrajectoryWaypoint> sorter = Comparator.comparingDouble(CoMHeightTrajectoryWaypoint::getX);
   private final YoFramePoint3D contactFrameZeroPosition;
   private final YoFramePoint3D contactFrameOnePosition;

   private final BagOfBalls bagOfBalls;
   private final OptimizedTrajectoryGenerator trajectoryGenerator;

   private ReferenceFrame referenceFrame;

   private final CoMHeightPartialDerivativesData comHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();

   private final StringStretcher2d stringStretcher = new StringStretcher2d();
   private final List<Point2DBasics> stretchedStringWaypoints = new ArrayList<>();

   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final FramePoint3D tempFramePoint2 = new FramePoint3D();
   private final Point2D pointToThrowAway = new Point2D();

   private final YoDouble partialDzDs;
   private final YoDouble partialD2zDs2;
   private final YoDouble partialD3zDs3;
   private final YoDouble partialDsDx;
   private final YoDouble partialDsDy;

   public SplinedHeightTrajectory(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      trajectoryGenerator = new OptimizedTrajectoryGenerator("height", 10, 4, registry);

      partialDzDs = new YoDouble("partialDzDs", registry);
      partialDsDx = new YoDouble("partialDsDx", registry);
      partialDsDy = new YoDouble("partialDsDy", registry);
      partialD2zDs2 = new YoDouble("partialD2zDs2", registry);
      partialD3zDs3 = new YoDouble("partialD3zDs3", registry);

      contactFrameZeroPosition = new YoFramePoint3D("contactFrameZeroPosition", worldFrame, registry);
      contactFrameOnePosition = new YoFramePoint3D("contactFrameOnePosition", worldFrame, registry);

      if (yoGraphicsListRegistry != null)
      {
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

   public void clearWaypoints()
   {
      waypoints.clear();
   }

   public void addWaypoints(List<CoMHeightTrajectoryWaypoint> waypoints)
   {
      for (int i = 0; i < waypoints.size(); i++)
         addWaypoint(waypoints.get(i));
   }

   public void addWaypoint(CoMHeightTrajectoryWaypoint waypoint)
   {
      waypoints.add(waypoint);
   }

   private final TDoubleArrayList heightWaypoints = new TDoubleArrayList();
   private final TDoubleArrayList alphaWaypoints = new TDoubleArrayList();

   public void computeSpline()
   {
      ListSorter.sort(waypoints, sorter);
      computeHeightsToUseByStretchingString(waypoints);

      int numberOfWaypoints = waypoints.size();

      alphaWaypoints.reset();
      heightWaypoints.reset();

      CoMHeightTrajectoryWaypoint startWaypoint = waypoints.get(0);
      CoMHeightTrajectoryWaypoint endWaypoint = waypoints.get(numberOfWaypoints - 1);

      for (int i = 1; i < numberOfWaypoints - 1; i++)
      {
         CoMHeightTrajectoryWaypoint waypoint = waypoints.get(i);
         double alpha = (waypoint.getX() - startWaypoint.getX()) / (endWaypoint.getX() - startWaypoint.getX());
         heightWaypoints.add(waypoint.getHeight());
         alphaWaypoints.add(alpha);
      }

      trajectoryGenerator.reset();
      trajectoryGenerator.setEndpointConditions(startWaypoint.getHeight(), 0.0, endWaypoint.getHeight(), 0.0);
      trajectoryGenerator.setWaypoints(heightWaypoints);
      trajectoryGenerator.setWaypointTimes(alphaWaypoints);

      trajectoryGenerator.initialize();

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

   private final Point2D tempPoint = new Point2D();

   public double solve(CoMHeightPartialDerivativesDataBasics comHeightPartialDerivativesDataToPack,
                       FramePoint3DBasics queryPoint,
                       Point2DBasics pointOnSplineToPack)
   {
      tempPoint.set(queryPoint);
      EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(tempPoint,
                                                              contactFrameZeroPosition.getX(),
                                                              contactFrameZeroPosition.getY(),
                                                              contactFrameOnePosition.getX(),
                                                              contactFrameOnePosition.getY(),
                                                              tempPoint);
      double percentAlongSegment = EuclidGeometryTools.percentageAlongLineSegment2D(tempPoint.getX(),
                                                                                    tempPoint.getY(),
                                                                                    contactFrameZeroPosition.getX(),
                                                                                    contactFrameZeroPosition.getY(),
                                                                                    contactFrameOnePosition.getX(),
                                                                                    contactFrameOnePosition.getY());
      queryPoint.interpolate(contactFrameZeroPosition, contactFrameOnePosition, percentAlongSegment);

      CoMHeightTrajectoryWaypoint startWaypoint = waypoints.get(0);
      CoMHeightTrajectoryWaypoint endWaypoint = waypoints.get(waypoints.size() - 1);

      double xLength = Math.max(1.0, Math.abs(endWaypoint.getX() - startWaypoint.getX()));
      double splineQuery = InterpolationTools.linearInterpolate(startWaypoint.getX(), endWaypoint.getX(), percentAlongSegment);

      trajectoryGenerator.compute(percentAlongSegment);

      double z = trajectoryGenerator.getPosition();
      double dzds = trajectoryGenerator.getVelocity() / xLength ;
      double d2zds2 = trajectoryGenerator.getAcceleration() / MathTools.square(xLength);
      double d3zds3 = trajectoryGenerator.getJerk() / MathTools.pow(xLength, 3);
      partialDzDs.set(dzds);
      partialD2zDs2.set(d2zds2);
      partialD3zDs3.set(d3zds3);

      double length = contactFrameZeroPosition.distance(contactFrameOnePosition);
      double dsdx = (contactFrameOnePosition.getX() - contactFrameZeroPosition.getX()) / length;
      double dsdy = (contactFrameOnePosition.getY() - contactFrameZeroPosition.getY()) / length;
      partialDsDx.set(dsdx);
      partialDsDy.set(dsdy);

      double d2sdx2 = 0.0;
      double d2sdy2 = 0.0;
      double d2sdxdy = 0.0;

      double d3sdx3 = 0.0;
      double d3sdy3 = 0.0;
      double d3sdx2dy = 0.0;
      double d3sdxdy2 = 0.0;

      double dzdx = dsdx * dzds;
      double dzdy = dsdy * dzds;

      double d2zdx2 = dzds * d2sdx2 + d2zds2 * dsdx * dsdx;
      double d2zdy2 = dzds * d2sdy2 + d2zds2 * dsdy * dsdy;
      double d2zdxdy = d2zds2 * dsdx * dsdy + dzds * d2sdxdy;

      double d3zdx3 = d3zds3 * dsdx * dsdx * dsdx + 3.0 * d2zds2 * dsdx * d2sdx2 + dzds * d3sdx3;
      double d3zdy3 = d3zds3 * dsdy * dsdy * dsdy + 3.0 * d2zds2 * dsdy * d2sdy2 + dzds * d3sdy3;
      double d3zdx2dy = d3zds3 * dsdx * dsdx * dsdy + 2.0 * d2zds2 * dsdx * d2sdxdy + d2zds2 * d2sdx2 * dsdy + dzds * d3sdx2dy;
      double d3zdxdy2 = d3zds3 * dsdx * dsdy * dsdy + 2.0 * d2zds2 * dsdy * d2sdxdy + d2zds2 * dsdx * d2sdy2 + dzds * d3sdxdy2;

      tempFramePoint.setIncludingFrame(referenceFrame, 0.0, 0.0, z);
      tempFramePoint.changeFrame(worldFrame);

      comHeightPartialDerivativesDataToPack.setCoMHeight(worldFrame, tempFramePoint.getZ());
      comHeightPartialDerivativesDataToPack.setPartialDzDx(dzdx);
      comHeightPartialDerivativesDataToPack.setPartialDzDy(dzdy);
      comHeightPartialDerivativesDataToPack.setPartialD2zDxDy(d2zdxdy);
      comHeightPartialDerivativesDataToPack.setPartialD2zDx2(d2zdx2);
      comHeightPartialDerivativesDataToPack.setPartialD2zDy2(d2zdy2);
      comHeightPartialDerivativesDataToPack.setPartialD3zDx3(d3zdx3);
      comHeightPartialDerivativesDataToPack.setPartialD3zDy3(d3zdy3);
      comHeightPartialDerivativesDataToPack.setPartialD3zDx2Dy(d3zdx2dy);
      comHeightPartialDerivativesDataToPack.setPartialD3zDxDy2(d3zdxdy2);

      pointOnSplineToPack.set(splineQuery, z);

      return percentAlongSegment;
   }

   public double getHeightSplineSetpoint()
   {
      return trajectoryGenerator.getPosition();
   }
}
