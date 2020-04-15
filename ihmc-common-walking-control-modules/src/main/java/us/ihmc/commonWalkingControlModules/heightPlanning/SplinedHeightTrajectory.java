package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
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
import us.ihmc.tools.lists.ListSorter;
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
   private final YoFramePoint3D contactFrameZeroPosition;
   private final YoFramePoint3D contactFrameOnePosition;

   private final BagOfBalls bagOfBalls;
   private final YoConfigurablePolynomial spline;

   private ReferenceFrame referenceFrame;

   private final CoMHeightPartialDerivativesData comHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();

   private final StringStretcher2d stringStretcher = new StringStretcher2d();
   private final List<Point2DBasics> stretchedStringWaypoints = new ArrayList<>();

   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final FramePoint3D tempFramePoint2 = new FramePoint3D();
   private final Point2D pointToThrowAway = new Point2D();

   public SplinedHeightTrajectory(YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {

      spline = new YoConfigurablePolynomial("height", 9, registry);

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

   public void computeSpline()
   {
      ListSorter.sort(waypoints, sorter);
      computeHeightsToUseByStretchingString(waypoints);

      int numberOfWaypoints = waypoints.size();

      spline.reshape(numberOfWaypoints);
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

      double length = contactFrameZeroPosition.distance(contactFrameOnePosition);
      double dsdx = (contactFrameOnePosition.getX() - contactFrameZeroPosition.getX()) / length;
      double dsdy = (contactFrameOnePosition.getY() - contactFrameZeroPosition.getY()) / length;

      double ddsddx = 0.0;
      double ddsddy = 0.0;
      double ddsdxdy = 0.0;

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

   public double getHeightSplineSetpoint()
   {
      return spline.getPosition();
   }
}
