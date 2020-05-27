package us.ihmc.commonWalkingControlModules.polygonWiggling;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCylinder;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.awt.*;

public class GradientDescentStepConstraintSolver
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private int maximumIterations = 1000;
   private double alpha = 0.125;
   private double alphaForSmallGradient = 0.35;
   private double gradientMagnitudeToTerminate = 1e-5;
   private double gradientMagnitudeToApplyStandardAlpha = 1e-3;
   private double legCollisionSolverGain = 15.0;
   private final YoDouble gradientMagnitude = new YoDouble("gradientMagnitude", registry);

   private final FootPlacementConstraintCalculator footPlacementConstraintCalculator = new FootPlacementConstraintCalculator();
   private final LegCollisionConstraintCalculator legCollisionConstraintCalculator;

   private final Vector3D previousGradient = new Vector3D();
   private final YoFrameVector3D gradient = new YoFrameVector3D("gradient", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D footPlacementGradient = new YoFrameVector3D("footPlacementGradient", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D legCollisionGradient = new YoFrameVector3D("legCollisionGradient", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D accumulatedTransform = new YoFrameVector3D("accumulatedTransformient", ReferenceFrame.getWorldFrame(), registry);

   private final RecyclingArrayList<Point2D> transformedVertices = new RecyclingArrayList<>(5, Point2D::new);
   private final RecyclingArrayList<Vector2D> rotationVectors = new RecyclingArrayList<>(5, Vector2D::new);
   private final Pose2D initialPose = new Pose2D();
   private final Pose2D transformedPose = new Pose2D();

   private final TickAndUpdatable tickAndUpdatable;
   private final YoFrameLineSegment2D[] initialPolygonToWiggle = new YoFrameLineSegment2D[5];
   private final YoFrameLineSegment2D[] linearizedTransformedPolygonToWiggle = new YoFrameLineSegment2D[5];
   private final YoFrameLineSegment2D[] transformedPolygonToWiggle = new YoFrameLineSegment2D[5];
   private final YoFrameLineSegment2D[] constraintPolygon = new YoFrameLineSegment2D[100];
   private final YoFrameLineSegment2D[] yoRotationVectors = new YoFrameLineSegment2D[5];
   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;

   public GradientDescentStepConstraintSolver()
   {
      this.tickAndUpdatable = null;
      this.legCollisionConstraintCalculator = new LegCollisionConstraintCalculator();
      this.yoGraphicPlanarRegionsList = null;
   }

   public GradientDescentStepConstraintSolver(TickAndUpdatable tickAndUpdatable, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      registry.addChild(this.registry);
      this.tickAndUpdatable = tickAndUpdatable;
      this.legCollisionConstraintCalculator = new LegCollisionConstraintCalculator(graphicsListRegistry, registry);

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

      this.yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("planarRegions", 150, 100, registry);
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), yoGraphicPlanarRegionsList);
   }

   /**
    * Wiggles into concave hull without leg collision check
    */
   public RigidBodyTransform wigglePolygon(ConvexPolygon2DReadOnly polygonToWiggle,
                                           Vertex2DSupplier concavePolygonToWiggleInto,
                                           WiggleParameters wiggleParameters)
   {
      return wigglePolygon(polygonToWiggle, concavePolygonToWiggleInto, wiggleParameters, null, null, null);
   }

   /**
    * Wiggles into concave hull with leg collision check
    */
   public RigidBodyTransform wigglePolygon(ConvexPolygon2DReadOnly polygonToWiggle,
                                           WiggleParameters wiggleParameters,
                                           RigidBodyTransformReadOnly footTransformInLocal,
                                           PlanarRegion regionToStep,
                                           PlanarRegionsList allRegions)
   {
      return wigglePolygon(polygonToWiggle,
                           Vertex2DSupplier.asVertex2DSupplier(regionToStep.getConcaveHull()),
                           wiggleParameters,
                           footTransformInLocal,
                           regionToStep.getTransformToWorld(),
                           allRegions);
   }

   private RigidBodyTransform wigglePolygon(ConvexPolygon2DReadOnly polygonToWiggle,
                                            Vertex2DSupplier concavePolygonToWiggleInto,
                                            WiggleParameters wiggleParameters,
                                            RigidBodyTransformReadOnly footTransformInLocal,
                                            RigidBodyTransformReadOnly localToWorld,
                                            PlanarRegionsList planarRegionsList)
   {
      int iterations = 0;
      accumulatedTransform.setToZero();
      rotationVectors.clear();
      transformedVertices.clear();
      initialPose.set(footTransformInLocal.getTranslationX(), footTransformInLocal.getTranslationY(), footTransformInLocal.getRotation().getYaw());

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
         initializeConstraintGraphics(polygonToWiggle, concavePolygonToWiggleInto, planarRegionsList);
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

         updateGraphics(polygonToWiggle);

         transformedPose.set(initialPose);
         transformedPose.appendTranslation(accumulatedTransform.getX(), accumulatedTransform.getY());
         transformedPose.appendRotation(accumulatedTransform.getZ());

         footPlacementConstraintCalculator.calculateFootAreaGradient(transformedVertices, rotationVectors, concavePolygonToWiggleInto, wiggleParameters,
                                                                     footPlacementGradient);
         legCollisionConstraintCalculator.calculateLegCollisionGradient(transformedPose, localToWorld, planarRegionsList, legCollisionGradient);

         double alphaFootPlacement = footPlacementGradient.lengthSquared();
         double alphaLegCollision = legCollisionSolverGain * legCollisionGradient.lengthSquared();
         footPlacementGradient.scale(alphaFootPlacement / (alphaFootPlacement + alphaLegCollision));
         legCollisionGradient.scale(alphaLegCollision / (alphaFootPlacement + alphaLegCollision));
         gradient.add(footPlacementGradient, legCollisionGradient);

         if (!xTranslationAllowed)
         {
            gradient.setX(0.0);
         }
         if (!yTranslationAllowed)
         {
            gradient.setY(0.0);
         }
         if (!rotationAllowed)
         {
            gradient.setZ(0.0);
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

   private void initializeConstraintGraphics(ConvexPolygon2DReadOnly polygonToWiggle, Vertex2DSupplier concavePolygonToWiggleInto, PlanarRegionsList planarRegionsList)
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

      if (planarRegionsList != null)
      {
         yoGraphicPlanarRegionsList.clear();
         yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(planarRegionsList);
         for (int i = 0; i < 100; i++)
         {
            yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
            tickAndUpdatable.tickAndUpdate();
         }
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

   public void setLegCollisionShape(Cylinder3D legCollisionShape, double solverGain, RigidBodyTransform legShapeTransformToSoleFrame)
   {
      this.legCollisionSolverGain = solverGain;
      this.legCollisionConstraintCalculator.setLegCollisionShape(legCollisionShape, legShapeTransformToSoleFrame);
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
