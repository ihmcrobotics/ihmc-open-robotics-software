package us.ihmc.commonWalkingControlModules.polygonWiggling;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.EuclidCoreMissingTools;

import java.util.List;

public class FootPlacementConstraintCalculator
{
   private final Point2D closestPerimeterPoint = new Point2D();
   private final Vector2D directionToClosestPoint = new Vector2D();

   void calculateFootAreaGradient(List<? extends Point2DReadOnly> polygonToWiggle,
                                  List<? extends Vector2DReadOnly> rotationVectors,
                                  Vertex2DSupplier concavePolygonToWiggleInto,
                                  WiggleParameters wiggleParameters,
                                  Tuple3DBasics gradientToPack)
   {
      gradientToPack.setToZero();

      for (int i = 0; i < polygonToWiggle.size(); i++)
      {
         Point2DReadOnly vertex = polygonToWiggle.get(i);

         boolean pointIsInside = PointInPolygonSolver.isPointInsidePolygon(concavePolygonToWiggleInto, vertex);
         double distanceSquaredFromPerimeter = distanceSquaredFromPerimeter(concavePolygonToWiggleInto, vertex, closestPerimeterPoint);

         double signedDistanceSquared = pointIsInside ? -distanceSquaredFromPerimeter : distanceSquaredFromPerimeter;
         double deltaOutside = -wiggleParameters.deltaInside;
         double signedDeltaOutsideSquared = Math.signum(deltaOutside) * MathTools.square(deltaOutside);
         boolean pointDoesNotMeetConstraints = signedDistanceSquared > signedDeltaOutsideSquared;

         if (pointDoesNotMeetConstraints)
         {
            directionToClosestPoint.sub(closestPerimeterPoint, vertex);
            if (signedDistanceSquared < 0.0)
            {
               directionToClosestPoint.scale(-1.0);
            }

            double distanceSquaredFromConstraint = signedDistanceSquared - signedDeltaOutsideSquared;
            directionToClosestPoint.scale(Math.sqrt(distanceSquaredFromConstraint / directionToClosestPoint.lengthSquared()));

            gradientToPack.addX(directionToClosestPoint.getX());
            gradientToPack.addY(directionToClosestPoint.getY());
            gradientToPack.addZ(directionToClosestPoint.dot(rotationVectors.get(i)) / wiggleParameters.rotationWeight);
         }
      }
   }

   private static double distanceSquaredFromPerimeter(Vertex2DSupplier polygon, Point2DReadOnly queryPoint, Point2D closestPointToPack)
   {
      double minimumDistanceSquared = Double.MAX_VALUE;
      Point2D tempPoint = new Point2D();

      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = polygon.getVertex(i);
         Point2DReadOnly nextVertex = polygon.getVertex((i + 1) % polygon.getNumberOfVertices());

         double distanceSquared = EuclidCoreMissingTools.distanceSquaredFromPoint2DToLineSegment2D(queryPoint.getX(),
                                                                                                   queryPoint.getY(),
                                                                                                   vertex.getX(),
                                                                                                   vertex.getY(),
                                                                                                   nextVertex.getX(),
                                                                                                   nextVertex.getY(),
                                                                                                   tempPoint);
         if (distanceSquared < minimumDistanceSquared)
         {
            minimumDistanceSquared = distanceSquared;
            closestPointToPack.set(tempPoint);
         }
      }

      return minimumDistanceSquared;
   }
}
