package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapDataReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class HeightMapCliffAvoider
{
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepSnapperReadOnly snapper;

   private HeightMapData heightMapData;
   private final ConvexPolygon2D scaledFootPolygon = new ConvexPolygon2D();

   private final SideDependentList<ConvexPolygon2D> defaultFootPolygons;
   private final SideDependentList<ConvexPolygon2DReadOnly> scaledFootPolygons = new SideDependentList<>();
   private final YoDouble highestCliffHeight;

   private final Plane3D bestFitPlane = new Plane3D();

   private double previousScaledFootPolygonPercentage = Double.NaN;

   public HeightMapCliffAvoider(FootstepPlannerParametersReadOnly parameters,
                                FootstepSnapperReadOnly snapper,
                                SideDependentList<ConvexPolygon2D> defaultFootPolygons,
                                YoRegistry registry)
   {
      this.parameters = parameters;
      this.snapper = snapper;
      this.defaultFootPolygons = defaultFootPolygons;
      highestCliffHeight = new YoDouble("highestCliffHeight", registry);
      updateScaledFootPolygons();
   }

   private void updateScaledFootPolygons()
   {
      double scaledFootPolygonPercentage = parameters.getScaledFootPolygonPercentage();
      if (MathTools.epsilonEquals(previousScaledFootPolygonPercentage, scaledFootPolygonPercentage, 0.001))
      {
         return;
      }

      for (RobotSide robotSide : RobotSide.values())
      {
         ConvexPolygon2D scaledFootPolygon = new ConvexPolygon2D(defaultFootPolygons.get(robotSide));
         addVerticesToFootPolygon(scaledFootPolygon);
         scaledFootPolygon.scale(scaledFootPolygonPercentage);
         scaledFootPolygons.put(robotSide, scaledFootPolygon);
      }

      previousScaledFootPolygonPercentage = scaledFootPolygonPercentage;
   }

   public void addVerticesToFootPolygon(ConvexPolygon2D scaledFootPolygon)
   {
      int numberOfVertices = scaledFootPolygon.getNumberOfVertices();
      ConvexPolygon2D copyFootPolygon = new ConvexPolygon2D();
      copyFootPolygon.set(scaledFootPolygon);

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2DReadOnly start = copyFootPolygon.getVertex(i);
         Point2DReadOnly end = copyFootPolygon.getNextVertex(i);

         int numInterpolations = 5;

         for (int j = 0; j < numInterpolations; j++)
         {
            double alpha = (j + 1.0) / (numInterpolations + 1.0);
            Point2D pointToAdd = new Point2D();
            pointToAdd.interpolate(start, end, alpha);
            pointToAdd.scale(1.02);
            scaledFootPolygon.addVertex(pointToAdd);
         }

         scaledFootPolygon.update();
      }
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
   }

   public boolean isStepValid(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep)
   {
      updateScaledFootPolygons();

      /* Transform to step location */
      FootstepSnapDataReadOnly snapData = snapper.snapFootstep(candidateStep);

      // Something went wrong when snapping the data, this is a bad step
      if (!snapData.getSnappedToHeightMap())
         return false;

      DiscreteFootstepTools.getFootPolygon(candidateStep, scaledFootPolygons.get(candidateStep.getRobotSide()), scaledFootPolygon);
      RigidBodyTransformReadOnly snapTransform = snapData.getSnapTransform();
      scaledFootPolygon.applyTransform(snapTransform, false);

      /* Compute best-fit plane */
      RigidBodyTransformReadOnly snappedStepTransform = snapData.getSnappedStepTransform(candidateStep);
      bestFitPlane.getPoint().set(snappedStepTransform.getTranslation());
      bestFitPlane.getNormal().set(Axis3D.Z);
      snappedStepTransform.getRotation().transform(bestFitPlane.getNormal());
      highestCliffHeight.set(Double.NEGATIVE_INFINITY);

      for (int pointIdx = 0; pointIdx < scaledFootPolygon.getNumberOfVertices(); pointIdx++)
      {
         Point2DReadOnly polygonVertex = scaledFootPolygon.getVertex(pointIdx);
         double zBestFitPlane = bestFitPlane.getZOnPlane(polygonVertex.getX(), polygonVertex.getY());
         double zHeightMap = heightMapData.getHeightAt(polygonVertex.getX(), polygonVertex.getY());
         double distanceFromBestFitPlane = zHeightMap - zBestFitPlane;
         highestCliffHeight.set(Math.max(distanceFromBestFitPlane, highestCliffHeight.getValue()));

         if (distanceFromBestFitPlane > parameters.getCliffHeightThreshold())
         {
            return false;
         }
      }

      return true;
   }
}
