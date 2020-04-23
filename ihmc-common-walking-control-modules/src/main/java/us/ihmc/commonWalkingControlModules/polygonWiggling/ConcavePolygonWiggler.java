package us.ihmc.commonWalkingControlModules.polygonWiggling;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameLineSegment2D;

import java.awt.*;

public class ConcavePolygonWiggler
{
   private final String name = getClass().getSimpleName();
   private int maximumIterations = 1000;
   private double alpha = 0.06;
   private double gradientMagnitudeToTerminate = 1e-7;

   private final Vector3D previousGradient = new Vector3D();
   private final Vector3D gradient = new Vector3D();
   private final Vector3D gradientDelta = new Vector3D();
   private final Vector3D accumulatedTransform = new Vector3D();

   private final Point2D tempPoint = new Point2D();
   private final Point2D closestPerimeterPoint = new Point2D();
   private final Vector2D directionToClosestPoint = new Vector2D();
   private final RecyclingArrayList<Point2D> transformedVertices = new RecyclingArrayList<>(4, Point2D::new);
   private final RecyclingArrayList<Vector2D> rotationVectors = new RecyclingArrayList<>(4, Vector2D::new);

   private final TickAndUpdatable tickAndUpdatable;
   private final YoFrameLineSegment2D[] initialPolygonToWiggle = new YoFrameLineSegment2D[4];
   private final YoFrameLineSegment2D[] transformedPolygonToWiggle = new YoFrameLineSegment2D[4];
   private final YoFrameLineSegment2D[] constraintPolygon = new YoFrameLineSegment2D[100];

   public ConcavePolygonWiggler(TickAndUpdatable tickAndUpdatable, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      this.tickAndUpdatable = tickAndUpdatable;

      for (int i = 0; i < 4; i++)
      {
         initialPolygonToWiggle[i] = new YoFrameLineSegment2D("initialFootV" + i, ReferenceFrame.getWorldFrame(), registry);
         graphicsListRegistry.registerArtifact(name, new YoArtifactLineSegment2d("graphicInitialFootV" + i, initialPolygonToWiggle[i], Color.RED, 0.0, 0.0));

         transformedPolygonToWiggle[i] = new YoFrameLineSegment2D("transformedFootV" + i, ReferenceFrame.getWorldFrame(), registry);
         graphicsListRegistry.registerArtifact(name, new YoArtifactLineSegment2d("graphicFootV" + i, transformedPolygonToWiggle[i], Color.BLUE.darker(), 0.0, 0.0));
      }

      for (int i = 0; i < constraintPolygon.length; i++)
      {
         constraintPolygon[i] = new YoFrameLineSegment2D("polygonEdge" + i, ReferenceFrame.getWorldFrame(), registry);
         graphicsListRegistry.registerArtifact(name, new YoArtifactLineSegment2d("graphicPolygonEdge" + i, constraintPolygon[i], Color.BLACK, 0.0, 0.0));
      }
   }

   public RigidBodyTransform wigglePolygon(ConvexPolygon2DReadOnly polygonToWiggle, Vertex2DSupplier concavePolygonToWiggleInto, WiggleParameters wiggleParameters)
   {
      int iterations = 0;
      accumulatedTransform.setToZero();
      rotationVectors.clear();
      transformedVertices.clear();

      for (int i = 0; i < concavePolygonToWiggleInto.getNumberOfVertices(); i++)
      {
         constraintPolygon[i].getFirstEndpoint().set(concavePolygonToWiggleInto.getVertex(i));
         constraintPolygon[i].getSecondEndpoint().set(concavePolygonToWiggleInto.getVertex((i + 1) % concavePolygonToWiggleInto.getNumberOfVertices()));
      }

      for (int i = 0; i < polygonToWiggle.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = polygonToWiggle.getVertex(i);
         Vector2D rotationVector = rotationVectors.add();

         rotationVector.sub(vertex, polygonToWiggle.getCentroid());
         rotationVector.set(- rotationVector.getY(), rotationVector.getX());
         rotationVector.normalize();
         transformedVertices.add().set(vertex);

         initialPolygonToWiggle[i].getFirstEndpoint().set(polygonToWiggle.getVertex(i));
         initialPolygonToWiggle[i].getSecondEndpoint().set(polygonToWiggle.getVertex((i + 1) % polygonToWiggle.getNumberOfVertices()));
         transformedPolygonToWiggle[i].set(initialPolygonToWiggle[i]);
      }

      tickAndUpdatable.tickAndUpdate();

      while (true)
      {
         if (iterations > maximumIterations)
         {
            break;
         }

         if (!MathTools.intervalContains(accumulatedTransform.getX(), wiggleParameters.minX, wiggleParameters.maxX)
             || !MathTools.intervalContains(accumulatedTransform.getY(), wiggleParameters.minY, wiggleParameters.maxY)
             || !MathTools.intervalContains(accumulatedTransform.getZ(), wiggleParameters.minYaw, wiggleParameters.maxYaw))
         {
            break;
         }

         boolean allPointsInside = true;
         gradient.setToZero();

         for (int i = 0; i < polygonToWiggle.getNumberOfVertices(); i++)
         {
            Point2DReadOnly vertex = transformedVertices.get(i);
            transformedPolygonToWiggle[i].getFirstEndpoint().set(transformedVertices.get(i));
            transformedPolygonToWiggle[i].getSecondEndpoint().set(transformedVertices.get((i + 1) % transformedVertices.size()));

            boolean pointIsInside = PointInPolygonSolver.isPointInsidePolygon(concavePolygonToWiggleInto, vertex);
            double distanceSquaredFromPerimeter = distanceSquaredFromPerimeter(concavePolygonToWiggleInto, vertex);

            double signedDistanceSquared = pointIsInside ? - distanceSquaredFromPerimeter : distanceSquaredFromPerimeter;
            double deltaOutside = - wiggleParameters.deltaInside;
            double signedDeltaOutsideSquared = Math.signum(deltaOutside) * MathTools.square(deltaOutside);
            boolean pointDoesNotMeetConstraints = signedDistanceSquared > signedDeltaOutsideSquared;

            if (pointDoesNotMeetConstraints)
            {
               allPointsInside = false;

               directionToClosestPoint.sub(closestPerimeterPoint, vertex);
               if (signedDistanceSquared < 0.0)
               {
                  directionToClosestPoint.scale(-1.0);
               }

               double distanceSquaredFromConstraint = signedDistanceSquared - signedDeltaOutsideSquared;
               directionToClosestPoint.scale(Math.sqrt(distanceSquaredFromConstraint / directionToClosestPoint.lengthSquared()));

               gradient.addX(directionToClosestPoint.getX());
               gradient.addY(directionToClosestPoint.getY());
               gradient.addZ(directionToClosestPoint.dot(rotationVectors.get(i)) / wiggleParameters.rotationWeight);
            }
         }

         tickAndUpdatable.tickAndUpdate();

         if (allPointsInside)
         {
            System.out.println("All points inside");
            break;
         }

         gradient.scale(alpha);
         accumulatedTransform.add(gradient);

         for (int i = 0; i < polygonToWiggle.getNumberOfVertices(); i++)
         {
            transformedVertices.get(i).add(gradient.getX(), gradient.getY());
            transformedVertices.get(i).addX(gradient.getZ() * rotationVectors.get(i).getX());
            transformedVertices.get(i).addY(gradient.getZ() * rotationVectors.get(i).getY());
         }

         if (iterations > 0)
         {
            gradientDelta.sub(gradient, previousGradient);
            if (gradientDelta.lengthSquared() < MathTools.square(gradientMagnitudeToTerminate))
            {
               break;
            }
         }

         previousGradient.set(gradient);
         iterations++;
      }

      System.out.println("Iterations: " + iterations);
      System.out.println("x:   " + accumulatedTransform.getX());
      System.out.println("y:   " + accumulatedTransform.getY());
      System.out.println("yaw: " + Math.toDegrees(accumulatedTransform.getZ()) + " (deg)");

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation().set(accumulatedTransform.getX(), accumulatedTransform.getY(), 0.0);
      transform.getRotation().setToYawOrientation(accumulatedTransform.getZ());

      return transform;
   }

   private double distanceSquaredFromPerimeter(Vertex2DSupplier polygon, Point2DReadOnly queryPoint)
   {
      double minimumDistanceSquared = Double.MAX_VALUE;

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
            closestPerimeterPoint.set(tempPoint);
         }
      }

      return minimumDistanceSquared;
   }

   public RecyclingArrayList<Point2D> getTransformedVertices()
   {
      return transformedVertices;
   }

   public void setGradientMagnitudeToTerminate(double gradientMagnitudeToTerminate)
   {
      this.gradientMagnitudeToTerminate = gradientMagnitudeToTerminate;
   }

   public void setMaximumIterations(int maximumIterations)
   {
      this.maximumIterations = maximumIterations;
   }

   public void setAlpha(double alpha)
   {
      this.alpha = alpha;
   }
}
