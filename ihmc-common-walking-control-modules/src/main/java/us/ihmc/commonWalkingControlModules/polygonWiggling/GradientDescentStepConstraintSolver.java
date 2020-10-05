package us.ihmc.commonWalkingControlModules.polygonWiggling;

import java.awt.Color;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameLineSegment2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class GradientDescentStepConstraintSolver
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private int maximumIterations = 1000;
   private double alpha = 0.125;
   private double alphaForSmallGradient = 0.35;
   private double gradientMagnitudeToTerminate = 1e-5;
   private double gradientMagnitudeToApplyStandardAlpha = 1e-3;
   private final YoDouble gradientMagnitude = new YoDouble("gradientMagnitude", registry);

   private final FootPlacementConstraintCalculator footPlacementConstraintCalculator = new FootPlacementConstraintCalculator();
   private final LegCollisionConstraintCalculator legCollisionConstraintCalculator;

   private final Vector3D previousGradient = new Vector3D();
   private final YoFrameVector3D gradient = new YoFrameVector3D("gradient", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D footPlacementGradient = new YoFrameVector3D("footPlacementGradient", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D legCollisionGradient = new YoFrameVector3D("legCollisionGradient", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D accumulatedTransform = new YoFrameVector3D("accumulatedTransformient", ReferenceFrame.getWorldFrame(), registry);
   private final YoEnum<Constraint> activeConstraint = new YoEnum<>("activeConstraint", "Active constraint", registry, Constraint.class, true);

   private final RecyclingArrayList<Point2D> transformedVertices = new RecyclingArrayList<>(5, Point2D::new);
   private final RecyclingArrayList<Vector2D> rotationVectors = new RecyclingArrayList<>(5, Vector2D::new);
   private final RigidBodyTransform transformedSoleToRegionFrame = new RigidBodyTransform();

   private final TickAndUpdatable tickAndUpdatable;
   private final YoFrameLineSegment2D[] initialPolygonToWiggle = new YoFrameLineSegment2D[5];
   private final YoFrameLineSegment2D[] linearizedTransformedPolygonToWiggle = new YoFrameLineSegment2D[5];
   private final YoFrameLineSegment2D[] transformedPolygonToWiggle = new YoFrameLineSegment2D[5];
   private final YoFrameLineSegment2D[] constraintPolygon = new YoFrameLineSegment2D[500];
   private final YoFrameLineSegment2D[] yoRotationVectors = new YoFrameLineSegment2D[5];
   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;
   private final YoGraphicCoordinateSystem soleFrameGraphic;

   private enum Constraint
   {
      FOOT_AREA,
      LEG_COLLISION
   }

   public GradientDescentStepConstraintSolver()
   {
      this.tickAndUpdatable = null;
      this.legCollisionConstraintCalculator = new LegCollisionConstraintCalculator();
      this.yoGraphicPlanarRegionsList = null;
      this.soleFrameGraphic = null;
   }

   public GradientDescentStepConstraintSolver(TickAndUpdatable tickAndUpdatable, YoGraphicsListRegistry graphicsListRegistry, YoRegistry registry)
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

      this.soleFrameGraphic = new YoGraphicCoordinateSystem("soleFrame", "", registry, false, 0.3);
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), soleFrameGraphic);
   }

   public RigidBodyTransform wigglePolygon(GradientDescentStepConstraintInput input)
   {
      input.checkInputs();

      int iterations = 0;
      accumulatedTransform.setToZero();
      rotationVectors.clear();
      transformedVertices.clear();
      footPlacementGradient.setToZero();
      legCollisionGradient.setToZero();

      ConvexPolygon2D polygonToWiggle = input.getInitialStepPolygon();
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
         initializeConstraintGraphics(polygonToWiggle, input);
         tickAndUpdatable.tickAndUpdate();
      }

      if (input.containsInputForLegCollisionCheck())
      {
         transformedSoleToRegionFrame.set(input.getFootstepInRegionFrame());
      }

      activeConstraint.set(computeActiveConstraint(input));
      if (activeConstraint.getValue() == null)
      {
         return new RigidBodyTransform();
      }

      WiggleParameters wiggleParameters = input.getWiggleParameters();

      while (iterations <= maximumIterations)
      {
         updateGraphics(polygonToWiggle);

         // Calculate leg collision gradient regardless of active constraint if possible. If active constraint is foot area we want to stop if there's a leg collision
         if (input.containsInputForLegCollisionCheck())
         {
            legCollisionConstraintCalculator.calculateLegCollisionGradient(transformedSoleToRegionFrame,
                                                                           input.getLocalToWorld(),
                                                                           input.getPlanarRegionsList(),
                                                                           legCollisionGradient);
         }

         // Only calculate foot area constraint if it's the active constraint
         if (activeConstraint.getValue() == Constraint.FOOT_AREA)
         {
            footPlacementConstraintCalculator.calculateFootAreaGradient(transformedVertices, rotationVectors, input.getPolygonToWiggleInto(), wiggleParameters,
                                                                        footPlacementGradient);
         }

         if (activeConstraint.getValue() == Constraint.FOOT_AREA)
         {
            if (legCollisionGradient.lengthSquared() > 0.0)
            {
               break;
            }
            else
            {
               gradient.set(footPlacementGradient);
            }
         }
         else
         {
            gradient.set(legCollisionGradient);
         }

         boolean xTranslationAllowed = MathTools.intervalContains(accumulatedTransform.getX(), wiggleParameters.minX, wiggleParameters.maxX);
         boolean yTranslationAllowed = MathTools.intervalContains(accumulatedTransform.getY(), wiggleParameters.minY, wiggleParameters.maxY);
         boolean rotationAllowed = MathTools.intervalContains(accumulatedTransform.getZ(), wiggleParameters.minYaw, wiggleParameters.maxYaw);

         if (!xTranslationAllowed && !yTranslationAllowed && !rotationAllowed)
         {
            break;
         }
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

         transformedSoleToRegionFrame.getTranslation().add(gradient.getX(), gradient.getY(), 0.0);
         transformedSoleToRegionFrame.getRotation().appendYawRotation(gradient.getZ());

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

   private Constraint computeActiveConstraint(GradientDescentStepConstraintInput input)
   {
      double gradientEpsilonToActivateConstraint = 1e-10;

      if (input.containsInputForLegCollisionCheck())
      {
         legCollisionConstraintCalculator.calculateLegCollisionGradient(transformedSoleToRegionFrame,
                                                                        input.getLocalToWorld(),
                                                                        input.getPlanarRegionsList(),
                                                                        gradient);
         if (gradient.length() > gradientEpsilonToActivateConstraint)
         {
            return Constraint.LEG_COLLISION;
         }
      }

      footPlacementConstraintCalculator.calculateFootAreaGradient(transformedVertices,
                                                                  rotationVectors,
                                                                  input.getPolygonToWiggleInto(),
                                                                  input.getWiggleParameters(),
                                                                  gradient);
      if (gradient.length() > gradientEpsilonToActivateConstraint)
      {
         return Constraint.FOOT_AREA;
      }

      if (tickAndUpdatable != null)
      {
         tickAndUpdatable.tickAndUpdate();
      }

      return null;
   }

   private void packWorldFrameTransform(ConvexPolygon2DReadOnly polygonToWiggle, RigidBodyTransform transform)
   {
      transform.getTranslation().set(accumulatedTransform.getX(), accumulatedTransform.getY(), 0.0);
      transform.getRotation().setToYawOrientation(accumulatedTransform.getZ());
      transform.prependTranslation(polygonToWiggle.getCentroid().getX(), polygonToWiggle.getCentroid().getY(), 0.0);
      transform.appendTranslation(-polygonToWiggle.getCentroid().getX(), -polygonToWiggle.getCentroid().getY(), 0.0);
   }

   private void initializeConstraintGraphics(ConvexPolygon2DReadOnly polygonToWiggle, GradientDescentStepConstraintInput input)
   {
      if (tickAndUpdatable == null)
      {
         return;
      }

      Vertex2DSupplier concavePolygonToWiggleInto = input.getPolygonToWiggleInto();
      RigidBodyTransform footstepInRegionFrame = input.getFootstepInRegionFrame();

      // reset
      for (int i = 0; i < constraintPolygon.length; i++)
      {
         constraintPolygon[i].setToNaN();
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

      RigidBodyTransform soleFrame = new RigidBodyTransform(footstepInRegionFrame);
      soleFrame.preMultiply(input.getLocalToWorld());
      soleFrameGraphic.setPosition(soleFrame.getTranslation());
      soleFrameGraphic.setOrientation(new Quaternion(soleFrame.getRotation()));
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

   public void setLegCollisionShape(Cylinder3D legCollisionShape, RigidBodyTransform legShapeTransformToSoleFrame)
   {
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
