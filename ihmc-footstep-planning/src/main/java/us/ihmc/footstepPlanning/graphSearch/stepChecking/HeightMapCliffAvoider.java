package us.ihmc.footstepPlanning.graphSearch.stepChecking;

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

import java.util.concurrent.atomic.AtomicReference;

public class HeightMapCliffAvoider
{
   private final FootstepPlannerParametersReadOnly parameters;
   private final FootstepSnapperReadOnly snapper;

   private HeightMapData heightMapData;
   private final ConvexPolygon2D scaledFootPolygon = new ConvexPolygon2D();
   private final SideDependentList<ConvexPolygon2DReadOnly> scaledFootPolygons = new SideDependentList<>();

   private final Plane3D bestFitPlane = new Plane3D();

   public HeightMapCliffAvoider(FootstepPlannerParametersReadOnly parameters, FootstepSnapperReadOnly snapper, SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.parameters = parameters;
      this.snapper = snapper;

      for (RobotSide robotSide : RobotSide.values())
      {
         ConvexPolygon2D scaledFootPolygon = new ConvexPolygon2D(footPolygons.get(robotSide));

         addVertexToFootPolygon(scaledFootPolygon);
         scaledFootPolygon.scale(parameters.getScaledFootPolygonPercentage());
         scaledFootPolygons.put(robotSide, scaledFootPolygon);
      }
   }

   public void addVertexToFootPolygon(ConvexPolygon2D scaledFootPolygon)
   {
      int numberOfVertices = scaledFootPolygon.getNumberOfVertices();
      ConvexPolygon2D copyFootPolygon = new ConvexPolygon2D();
      copyFootPolygon.set(scaledFootPolygon);

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2DReadOnly start = copyFootPolygon.getVertex(i);
         Point2DReadOnly end;

         // Make sure we do the entire loop which includes the line segment back to the first vertex
         if (i + 1 < numberOfVertices)
            end = copyFootPolygon.getVertex(i + 1);
         else
            end = copyFootPolygon.getVertex(0);

         if (start.distance(end) > 0.1)
         {
            Point2D firstPointToAdd = new Point2D();
            firstPointToAdd.interpolate(start, end, 0.3);
            firstPointToAdd.scale(1.02);
            scaledFootPolygon.addVertex(firstPointToAdd);
            scaledFootPolygon.update();

            Point2D secondPointToAdd = new Point2D();
            secondPointToAdd.interpolate(start, end, 1 - 0.3);
            secondPointToAdd.scale(1.02);
            scaledFootPolygon.addVertex(secondPointToAdd);
            scaledFootPolygon.update();
         }
      }
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
   }

   public boolean isStepValid(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep)
   {

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

      for (int pointIdx = 0; pointIdx < scaledFootPolygon.getNumberOfVertices(); pointIdx++)
      {
         Point2DReadOnly polygonVertex = scaledFootPolygon.getVertex(pointIdx);
         double zBestFitPlane = bestFitPlane.getZOnPlane(polygonVertex.getX(), polygonVertex.getY());
         double zHeightMap = heightMapData.getHeightAt(polygonVertex.getX(), polygonVertex.getY());
         double distanceFromBestFitPlane = Math.abs(zBestFitPlane - zHeightMap);

         if (distanceFromBestFitPlane > parameters.getCliffHeightThreshold())
         {
            return false;
         }
      }

      return true;
   }
}
