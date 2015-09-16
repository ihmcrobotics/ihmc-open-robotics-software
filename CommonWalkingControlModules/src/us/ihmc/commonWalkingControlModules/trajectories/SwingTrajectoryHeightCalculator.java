package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.InclusionFunction;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Created by agrabertilton on 2/11/15.
 */
public class SwingTrajectoryHeightCalculator
{
   private class FootSwingInclusionFunction implements InclusionFunction<Point3d>
   {
      Point2d pathOrigin;
      Vector2d trajectoryDirection;
      double pathLength;
      double pathWidth;

      public FootSwingInclusionFunction(Point2d start, Point2d end, double pathWidth)
      {
         pathOrigin = new Point2d(start.x, start.y);
         trajectoryDirection = new Vector2d(end.x - start.x, end.y - start.y);
         pathLength = trajectoryDirection.length();
         trajectoryDirection.normalize();
         this.pathWidth = pathWidth;
      }

      @Override
      public boolean isIncluded(Point3d test)
      {
         Vector2d vectorFromPath = new Vector2d(test.x, test.y);
         vectorFromPath.sub(pathOrigin);
         double distanceAlongPath = vectorFromPath.dot(trajectoryDirection);
         if ((distanceAlongPath < 0) || (distanceAlongPath > pathLength))
            return false;

         double distanceFromPath = Math.abs(vectorFromPath.x * trajectoryDirection.y - vectorFromPath.y * trajectoryDirection.x);
         if (distanceFromPath > pathWidth)
            return false;

         return true;
      }
   }

   private HeightCalculatorParameters calculatorParameters;
   private SteppingParameters steppingParameters;

   public SwingTrajectoryHeightCalculator(HeightCalculatorParameters heightCalculatorParameters, SteppingParameters steppingParameters)
   {
      this.calculatorParameters = heightCalculatorParameters;
      this.steppingParameters = steppingParameters;
   }

   public double getSwingHeight(Footstep initialFootstep, Footstep stanceFootstep, Footstep endFootstep, HeightMapWithPoints heightMap)
   {
      FramePose startPose = new FramePose();
      initialFootstep.getSolePose(startPose);
      FramePose endPose = new FramePose();
      endFootstep.getSolePose(endPose);
      Point3d position = new Point3d();
      stanceFootstep.getPosition(position);

      return getSwingHeight(startPose, endPose, position.getZ(), heightMap);
   }

   public double getSwingHeight(FramePose startPose, FramePose endPose, double stanceHeight, HeightMapWithPoints groundProfile)
   {
      FramePoint startFramePoint = startPose.getFramePointCopy();
      FramePoint endFramePoint = endPose.getFramePointCopy();
      Point3d startPoint = startFramePoint.getPointCopy();
      Point3d endPoint = endFramePoint.getPointCopy();
      return getSwingHeight(startPoint, endPoint, stanceHeight, groundProfile);
   }

   public double getSwingHeight(FootstepData initialFootstep, FootstepData stanceFootstep, FootstepData endFootstep, HeightMapWithPoints heightMap)
   {
      return getSwingHeight(initialFootstep.getLocation(), endFootstep.getLocation(), stanceFootstep.getLocation().getZ(), heightMap);
   }

   public double getSwingHeight(Point3d startPoint, Point3d endPoint, double stanceHeight, HeightMapWithPoints groundProfile)
   {
      Vector3d startToEnd2d = new Vector3d(endPoint);
      startToEnd2d.sub(startPoint);
      startToEnd2d.setZ(0);
      double horizonalDistance = startToEnd2d.length();

      // search for points in area between the foot positions (range decreased by horizontal buffer size)
      Point2d startPoint2d = new Point2d(startPoint.x, startPoint.y);
      Point2d endPoint2d = new Point2d(endPoint.x, endPoint.y);
      FootSwingInclusionFunction inclusionFunction = new FootSwingInclusionFunction(startPoint2d, endPoint2d, calculatorParameters.getPathWidth());

      // get all points in the buffered region
      double x = (startPoint2d.x + endPoint2d.x) / 2.0;
      double y = (startPoint2d.y + endPoint2d.y) / 2.0;
      List<Point3d> pointsBetweenFeet = groundProfile.getAllPointsWithinArea(x, y, horizonalDistance / 2.0, horizonalDistance / 2.0, inclusionFunction);
      double z0 = startPoint.z;
      double maxZDiffFromStart = Math.max(0.0, endPoint.z - z0);

      Collections.sort(pointsBetweenFeet, new Comparator<Point3d>() {
         @Override
         public int compare(Point3d o1, Point3d o2)
         {
            if (o1.getZ() == o2.getZ()) return 0;
            if (o1.getZ() > o2.getZ()) return -1;
            return 1;
         }
      });

      if (pointsBetweenFeet.size() >= 1){
    	  double[] zHeightsFromStart = new double[pointsBetweenFeet.size()];
    	  int index = 0;
    	  for (Point3d point : pointsBetweenFeet)
    	  {
    		  zHeightsFromStart[index] = point.z - z0;
    		  index ++;
    	  }

    	  //use the max height
    	  //TODO filter using the first few points
    	  int numberOfOutliersToIgnore = calculatorParameters.getNumberOfOutliersToIgnore();
    	  double zHeightForOutlierQualification = calculatorParameters.getzHeightForOutlierQualification();
    	  int firstInvalidIndex = zHeightsFromStart.length - numberOfOutliersToIgnore;

    	  int indexOfMax;
    	  for (indexOfMax = 0; indexOfMax < numberOfOutliersToIgnore && indexOfMax < firstInvalidIndex; indexOfMax++){
    		  if (zHeightsFromStart[indexOfMax] - zHeightsFromStart[indexOfMax + numberOfOutliersToIgnore] < zHeightForOutlierQualification){
    			  break;
    		  }
    	  }
    	  maxZDiffFromStart = Math.max(maxZDiffFromStart, zHeightsFromStart[indexOfMax]);
      }
      
      double swingHeight = maxZDiffFromStart + calculatorParameters.getVerticalBuffer();
      //crop based on stance foot
      double distanceAboveStanceFoot = startPoint.getZ() + swingHeight - stanceHeight;
      if (distanceAboveStanceFoot > steppingParameters.getMaxSwingHeightFromStanceFoot()){
         swingHeight = stanceHeight + steppingParameters.getMaxSwingHeightFromStanceFoot() - startPoint.getZ();
      }
      return swingHeight;
   }

   public List<FramePoint> computeSwingTrajectoryPoints(FramePose startPose, FramePose endPose, HeightMapWithPoints groundProfile)
   {
      List<FramePoint> trajectoryPoints = new ArrayList<FramePoint>();

      FramePoint startFramePoint = startPose.getFramePointCopy();
      FramePoint endFramePoint = endPose.getFramePointCopy();
      Point3d startPoint = startFramePoint.getPointCopy();
      Point3d endPoint = endFramePoint.getPointCopy();

      Vector3d startToEnd3d = new Vector3d(endPoint);
      startToEnd3d.sub(startPoint);

      Vector2d startToEnd2dDirection = new Vector2d(startToEnd3d.x, startToEnd3d.y);
      double horizonalDistance = startToEnd2dDirection.length();
      startToEnd2dDirection.normalize();

      if (horizonalDistance <= 2 * calculatorParameters.getHorizontalBuffer())
      {
         // vertical step
         FramePoint midpoint = new FramePoint(startPose.getReferenceFrame());
         midpoint.set(endPoint);
         midpoint.add(startPoint);
         midpoint.scale(0.5);
         midpoint.add(0.0, 0.0, calculatorParameters.getVerticalBuffer());

         trajectoryPoints.add(startFramePoint);
         trajectoryPoints.add(midpoint);
         trajectoryPoints.add(endFramePoint);

         return trajectoryPoints;
      }

      // search for points in area between the foot positions (range decreased by horizontal buffer size)
      Point2d bufferedStartPoint = new Point2d(startPoint.x + calculatorParameters.getHorizontalBuffer() * startToEnd2dDirection.x,
                                      startPoint.y + calculatorParameters.getHorizontalBuffer() * startToEnd2dDirection.y);

      Point2d bufferedEndPoint = new Point2d(endPoint.x - calculatorParameters.getHorizontalBuffer() * startToEnd2dDirection.x, endPoint.y - calculatorParameters.getHorizontalBuffer() * startToEnd2dDirection.y);

      FootSwingInclusionFunction inclusionFunction = new FootSwingInclusionFunction(bufferedStartPoint, bufferedEndPoint, calculatorParameters.getPathWidth());


      // get all points in the buffered region
      double x = (bufferedStartPoint.x + bufferedEndPoint.x) / 2.0;
      double y = (bufferedStartPoint.y + bufferedEndPoint.y) / 2.0;
      List<Point3d> innerPoints = groundProfile.getAllPointsWithinArea(x, y, horizonalDistance / 2.0, horizonalDistance / 2.0, inclusionFunction);

      // project all points onto the vertical plane along direction line from start to end.
      List<Point2d> projectedInnerPoints = new ArrayList<Point2d>();
      Point2d startPoint2d = new Point2d(startPoint.x, startPoint.y);
      Vector2d horizontalVectorToPoint = new Vector2d();
      for (Point3d point : innerPoints)
      {
         horizontalVectorToPoint.set(point.x, point.y);
         horizontalVectorToPoint.sub(startPoint2d);
         double distanceAlongLine = horizontalVectorToPoint.dot(startToEnd2dDirection);
         if ((distanceAlongLine > 0) || (distanceAlongLine < horizonalDistance))
            projectedInnerPoints.add(new Point2d(distanceAlongLine, point.z));
      }

      // add dummyPoints below to allow bottom removal
      Point2d dummyPointA = new Point2d(calculatorParameters.getHorizontalBuffer(), startPoint.z - 100);
      Point2d dummyPointB = new Point2d(horizonalDistance - calculatorParameters.getHorizontalBuffer(), startPoint.z - 100);
      projectedInnerPoints.add(dummyPointA);
      projectedInnerPoints.add(dummyPointB);

      // find verticesOnConvexHull, then remove bottom points
      ConvexPolygon2d convexPolygon2d = new ConvexPolygon2d(projectedInnerPoints);
      convexPolygon2d.update();

      // add all non-dummy points to new list
      List<Point2d> newInnerPoints = new ArrayList<Point2d>();
      Point2d currentPoint;
      int numberOfVertices = convexPolygon2d.getNumberOfVertices();
      for (int i = 0; i < numberOfVertices; i++)
      {
         currentPoint = convexPolygon2d.getVertex(i);

         if (!currentPoint.epsilonEquals(dummyPointA, 1e-13) &&!currentPoint.epsilonEquals(dummyPointB, 1e-13))
         {
            newInnerPoints.add(currentPoint);
         }
      }

      // add buffer to projected inner points in horizontal and vertical directions
      List<Point2d> bufferedPoints = new ArrayList<>();
      for (Point2d point2d : newInnerPoints)
      {
         bufferedPoints.add(new Point2d(point2d.x + calculatorParameters.getHorizontalBuffer(), point2d.y));
         bufferedPoints.add(new Point2d(point2d.x - calculatorParameters.getHorizontalBuffer(), point2d.y));
         bufferedPoints.add(new Point2d(point2d.x, point2d.y + calculatorParameters.getVerticalBuffer()));
      }

      // add start and end points to the list, points far below for the bottom of the hull
      bufferedPoints.add(new Point2d(0, startPoint.z));
      bufferedPoints.add(new Point2d(horizonalDistance, endPoint.z));
      Point2d dummyPoint1 = new Point2d(0, startPoint.z - 100.0);
      Point2d dummyPoint2 = new Point2d(horizonalDistance, endPoint.z - 100.0);
      bufferedPoints.add(dummyPoint1);
      bufferedPoints.add(dummyPoint2);

      // add extra points to ensure ground clearance
      bufferedPoints.add(new Point2d(0 + calculatorParameters.getHorizontalBuffer(), startPoint.z + calculatorParameters.getVerticalBuffer()));
      bufferedPoints.add(new Point2d(horizonalDistance - calculatorParameters.getHorizontalBuffer(), endPoint.z + calculatorParameters.getVerticalBuffer()));


      convexPolygon2d.clear();
      convexPolygon2d.addVertices(bufferedPoints, bufferedPoints.size());
      convexPolygon2d.update();

      int numberOfPoints = convexPolygon2d.getNumberOfVertices();
      double currentX;
      double currentY;
      for (int i = 0; i < numberOfPoints; i++)
      {
         currentPoint = convexPolygon2d.getVertex(i);

         if (!currentPoint.epsilonEquals(dummyPoint1, 1e-13) &&!currentPoint.epsilonEquals(dummyPoint2, 1e-13))
         {
            currentX = startPoint.x + currentPoint.x * startToEnd2dDirection.x;
            currentY = startPoint.y + currentPoint.x * startToEnd2dDirection.y;
            trajectoryPoints.add(new FramePoint(startFramePoint.getReferenceFrame(), currentX, currentY, currentPoint.y));
         }
      }

      return trajectoryPoints;
   }
}
