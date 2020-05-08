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
import us.ihmc.log.LogTools;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameLineSegment2D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.awt.*;

public class ConcavePolygonWiggler
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private int maximumIterations = 1000;
   private double alpha = 0.125;
   private double alphaForSmallGradient = 0.35;
   private double gradientMagnitudeToTerminate = 1e-5;
   private double gradientMagnitudeToApplyStandardAlpha = 1e-3;
   private final YoDouble gradientMagnitude = new YoDouble("gradientMagnitude", registry);

   private final Vector3D previousGradient = new Vector3D();
   private final YoFrameVector3D gradient = new YoFrameVector3D("gradient", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D accumulatedTransform = new YoFrameVector3D("accumulatedTransformient", ReferenceFrame.getWorldFrame(), registry);

   private final Point2D closestPerimeterPoint = new Point2D();
   private final Vector2D directionToClosestPoint = new Vector2D();
   private final RecyclingArrayList<Point2D> transformedVertices = new RecyclingArrayList<>(5, Point2D::new);
   private final RecyclingArrayList<Vector2D> rotationVectors = new RecyclingArrayList<>(5, Vector2D::new);

   private final TickAndUpdatable tickAndUpdatable;
   private final YoFrameLineSegment2D[] initialPolygonToWiggle = new YoFrameLineSegment2D[5];
   private final YoFrameLineSegment2D[] linearizedTransformedPolygonToWiggle = new YoFrameLineSegment2D[5];
   private final YoFrameLineSegment2D[] transformedPolygonToWiggle = new YoFrameLineSegment2D[5];
   private final YoFrameLineSegment2D[] constraintPolygon = new YoFrameLineSegment2D[100];
   private final YoFrameLineSegment2D[] yoRotationVectors = new YoFrameLineSegment2D[5];

   public ConcavePolygonWiggler()
   {
      this.tickAndUpdatable = null;
   }

   public ConcavePolygonWiggler(TickAndUpdatable tickAndUpdatable, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      registry.addChild(this.registry);
      this.tickAndUpdatable = tickAndUpdatable;

      for (int i = 0; i < 5; i++)
      {
         initialPolygonToWiggle[i] = new YoFrameLineSegment2D("initialFootV" + i, ReferenceFrame.getWorldFrame(), registry);
         graphicsListRegistry.registerArtifact(name, new YoArtifactLineSegment2d("graphicInitialFootV" + i, initialPolygonToWiggle[i], Color.RED, 0.0, 0.0));

         transformedPolygonToWiggle[i] = new YoFrameLineSegment2D("transformedFootV" + i, ReferenceFrame.getWorldFrame(), registry);
         graphicsListRegistry.registerArtifact(name, new YoArtifactLineSegment2d("graphicFootV" + i, transformedPolygonToWiggle[i], Color.GREEN.darker(), 0.0, 0.0));

         linearizedTransformedPolygonToWiggle[i] = new YoFrameLineSegment2D("linearizedTransformedFootV" + i, ReferenceFrame.getWorldFrame(), registry);
         graphicsListRegistry.registerArtifact(name, new YoArtifactLineSegment2d("linearizedGraphicFootV" + i, linearizedTransformedPolygonToWiggle[i], Color.BLUE.darker(), 0.0, 0.0));

         yoRotationVectors[i] = new YoFrameLineSegment2D("rotationVector" + i, ReferenceFrame.getWorldFrame(), registry);
         graphicsListRegistry.registerArtifact(name, new YoArtifactLineSegment2d("rotationVectorGraphic" + i, yoRotationVectors[i], Color.ORANGE.darker(), 0.01, 0.01));
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

      for (int i = 0; i < polygonToWiggle.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = polygonToWiggle.getVertex(i);
         Vector2D rotationVector = rotationVectors.add();

         rotationVector.sub(vertex, polygonToWiggle.getCentroid());
         rotationVector.set(- rotationVector.getY(), rotationVector.getX());
         transformedVertices.add().set(vertex);
      }

      if (tickAndUpdatable != null)
      {
         initializeConstraintGraphics(polygonToWiggle, concavePolygonToWiggleInto);
         tickAndUpdatable.tickAndUpdate();
      }

      while (true)
      {
         if (iterations > maximumIterations)
         {
            break;
         }

         boolean xTranslationAllowed = MathTools.intervalContains(accumulatedTransform.getX(), wiggleParameters.minX, wiggleParameters.maxX);
         boolean yTranslationAllowed = MathTools.intervalContains(accumulatedTransform.getY(), wiggleParameters.minY, wiggleParameters.maxY);
         boolean rotationAllowed = MathTools.intervalContains(accumulatedTransform.getZ(), wiggleParameters.minYaw, wiggleParameters.maxYaw);

         if (!xTranslationAllowed && !yTranslationAllowed && !rotationAllowed)
         {
            break;
         }

         gradient.setToZero();
         updateGraphics(polygonToWiggle);

         for (int i = 0; i < polygonToWiggle.getNumberOfVertices(); i++)
         {
            Point2DReadOnly vertex = transformedVertices.get(i);

            boolean pointIsInside = PointInPolygonSolver.isPointInsidePolygon(concavePolygonToWiggleInto, vertex);
            double distanceSquaredFromPerimeter = distanceSquaredFromPerimeter(concavePolygonToWiggleInto, vertex, closestPerimeterPoint);

            double signedDistanceSquared = pointIsInside ? - distanceSquaredFromPerimeter : distanceSquaredFromPerimeter;
            double deltaOutside = - wiggleParameters.deltaInside;
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
               
               if (xTranslationAllowed)
               {
                  gradient.addX(directionToClosestPoint.getX());
               }
               if (yTranslationAllowed)
               {
                  gradient.addY(directionToClosestPoint.getY());
               }
               if (rotationAllowed)
               {
                  gradient.addZ(directionToClosestPoint.dot(rotationVectors.get(i)) / wiggleParameters.rotationWeight);
               }
            }
         }

         if (tickAndUpdatable != null)
         {
            gradientMagnitude.set(gradient.length());
            tickAndUpdatable.tickAndUpdate();
         }


         if (gradient.lengthSquared() > MathTools.square(gradientMagnitudeToApplyStandardAlpha))
         {
            gradient.scale(alpha);
         }
         else
         {
            gradient.scale(alphaForSmallGradient);
         }

         accumulatedTransform.add(gradient);
         for (int i = 0; i < polygonToWiggle.getNumberOfVertices(); i++)
         {
            transformedVertices.get(i).add(gradient.getX(), gradient.getY());
            transformedVertices.get(i).addX(gradient.getZ() * rotationVectors.get(i).getX());
            transformedVertices.get(i).addY(gradient.getZ() * rotationVectors.get(i).getY());
         }

         if (iterations > 0)
         {
            if (gradient.lengthSquared() < MathTools.square(gradientMagnitudeToTerminate))
            {
               break;
            }
         }

         previousGradient.set(gradient);
         iterations++;
      }

      RigidBodyTransform transform = new RigidBodyTransform();
      if (accumulatedTransform.containsNaN())
      {
         LogTools.warn(name + ": NaN found in solution for concave hull constraint. returning identity transform");
      }
      else
      {
         packWorldFrameTransform(polygonToWiggle, transform);
      }

      return transform;
   }

   private void packWorldFrameTransform(ConvexPolygon2DReadOnly polygonToWiggle, RigidBodyTransform transform)
   {
      transform.getTranslation().set(accumulatedTransform.getX(), accumulatedTransform.getY(), 0.0);
      transform.getRotation().setToYawOrientation(accumulatedTransform.getZ());
      transform.prependTranslation(polygonToWiggle.getCentroid().getX(), polygonToWiggle.getCentroid().getY(), 0.0);
      transform.appendTranslation(-polygonToWiggle.getCentroid().getX(), -polygonToWiggle.getCentroid().getY(), 0.0);
   }

   private void initializeConstraintGraphics(ConvexPolygon2DReadOnly polygonToWiggle, Vertex2DSupplier concavePolygonToWiggleInto)
   {
      if (tickAndUpdatable == null)
      {
         return;
      }

      for (int i = 0; i < polygonToWiggle.getNumberOfVertices(); i++)
      {
         initialPolygonToWiggle[i].getFirstEndpoint().set(polygonToWiggle.getVertex(i));
         initialPolygonToWiggle[i].getSecondEndpoint().set(polygonToWiggle.getVertex((i + 1) % polygonToWiggle.getNumberOfVertices()));
         linearizedTransformedPolygonToWiggle[i].set(initialPolygonToWiggle[i]);
         yoRotationVectors[i].getFirstEndpoint().set(polygonToWiggle.getVertex(i));
         yoRotationVectors[i].getSecondEndpoint().set(rotationVectors.get(i));
         yoRotationVectors[i].getSecondEndpoint().add(polygonToWiggle.getVertex(i));
      }

      for (int i = 0; i < concavePolygonToWiggleInto.getNumberOfVertices(); i++)
      {
         constraintPolygon[i].getFirstEndpoint().set(concavePolygonToWiggleInto.getVertex(i));
         constraintPolygon[i].getSecondEndpoint().set(concavePolygonToWiggleInto.getVertex((i + 1) % concavePolygonToWiggleInto.getNumberOfVertices()));
      }
   }

   private void updateGraphics(ConvexPolygon2DReadOnly polygonToWiggle)
   {
      if (tickAndUpdatable == null)
      {
         return;
      }

      for (int i = 0; i < transformedVertices.size(); i++)
      {
         linearizedTransformedPolygonToWiggle[i].getFirstEndpoint().set(transformedVertices.get(i));
         linearizedTransformedPolygonToWiggle[i].getSecondEndpoint().set(transformedVertices.get((i + 1) % transformedVertices.size()));
      }

      RigidBodyTransform transform = new RigidBodyTransform();
      packWorldFrameTransform(polygonToWiggle, transform);

      for (int i = 0; i < initialPolygonToWiggle.length; i++)
      {
         transformedPolygonToWiggle[i].set(initialPolygonToWiggle[i]);
         transformedPolygonToWiggle[i].applyTransform(transform);
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

   public RecyclingArrayList<Point2D> getTransformedVertices()
   {
      return transformedVertices;
   }

   public void setMaximumIterations(int maximumIterations)
   {
      this.maximumIterations = maximumIterations;
   }

   public void setAlpha(double alpha)
   {
      this.alpha = alpha;
   }

   public void setGradientMagnitudeToTerminate(double gradientMagnitudeToTerminate)
   {
      this.gradientMagnitudeToTerminate = gradientMagnitudeToTerminate;
   }

   public double getGradientMagnitudeToTerminate()
   {
      return gradientMagnitudeToTerminate;
   }

   public double getAlpha()
   {
      return alpha;
   }
}
