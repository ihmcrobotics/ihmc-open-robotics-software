package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.commonWalkingControlModules.polygonWiggling.StepConstraintPolygonTools;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapSnapWiggler;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

public class FootstepSnapAndWiggler implements FootstepSnapperReadOnly
{
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private final DefaultFootstepPlannerParametersReadOnly parameters;

   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private double flatGroundHeight = 0.0;

   private final HashSet<DiscreteFootstep> snappedFootsteps = new HashSet<>();
   private final HashMap<DiscreteFootstep, FootstepSnapData> manuallySnappedFootsteps = new HashMap<>();
   private TerrainMapData terrainMapData;

   private final HeightMapPolygonSnapper heightMapSnapper = new HeightMapPolygonSnapper();
   private final HeightMapSnapWiggler heightMapSnapWiggler;

   // Use this by default
   public FootstepSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame,
                                 DefaultFootstepPlannerParametersReadOnly parameters)
   {
      this(footPolygonsInSoleFrame, parameters, null, null, null);
   }

   // Call this constructor only for testing
   public FootstepSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame,
                                 DefaultFootstepPlannerParametersReadOnly parameters,
                                 TickAndUpdatable tickAndUpdatable,
                                 YoGraphicsListRegistry graphicsListRegistry,
                                 YoRegistry parentRegistry)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.parameters = parameters;
      this.heightMapSnapWiggler = new HeightMapSnapWiggler(footPolygonsInSoleFrame, wiggleParameters);
   }

   public void setFlatGroundHeight(double flatGroundHeight)
   {
      this.flatGroundHeight = flatGroundHeight;
   }

   public void initialize()
   {
      updateWiggleParameters(wiggleParameters, parameters);
   }

   public void clearSnapData()
   {
      snappedFootsteps.forEach(DiscreteFootstep::clearSnapData);
      snappedFootsteps.clear();
      manuallySnappedFootsteps.clear();
   }

   public FootstepSnapData snapFootstep(DiscreteFootstep footstep)
   {
      return snapFootstep(footstep, null, false);
   }

   @Override
   public FootstepSnapData snapFootstep(DiscreteFootstep footstep, DiscreteFootstep stanceStep, boolean computeWiggleTransform)
   {
      if (footstep.hasSnapData())
      {
         FootstepSnapData snapData = footstep.getSnapData();
         if (snapData.getSnapTransform().containsNaN())
         {
            return snapData;
         }
         else if (snapData.getWiggleTransformInWorld().containsNaN() && computeWiggleTransform)
         {
            computeWiggleTransform(footstep, stanceStep, snapData);
            updateTheCroppedFoothold(footstep);
         }

         return snapData;
      }
      else if (manuallySnappedFootsteps.containsKey(footstep))
      {
         FootstepSnapData snapData = manuallySnappedFootsteps.get(footstep);
         footstep.setSnapData(snapData);
         return snapData;
      }
      else
      {
         if (terrainMapData == null)
         {
            return FootstepSnapData.identityData(flatGroundHeight);
         }
         else
         {
            FootstepSnapData snapData = computeSnapTransform(footstep, stanceStep);
            footstep.setSnapData(snapData);
            snappedFootsteps.add(footstep);

            if (snapData.getSnapTransform().containsNaN())
            {
               return snapData;
            }
            else if (computeWiggleTransform)
            {
               computeWiggleTransform(footstep, stanceStep, snapData);
            }

            return snapData;
         }
      }
   }

   protected FootstepSnapData computeSnapTransform(DiscreteFootstep footstepToSnap, DiscreteFootstep stanceStep)
   {
      return heightMapSnapper.computeSnapData(footstepToSnap, footPolygonsInSoleFrame.get(footstepToSnap.getRobotSide()), terrainMapData);
   }

   /**
    * Can manually add snap data for a footstep to bypass the snapper.
    */
   public void addSnapData(DiscreteFootstep footstep, FootstepSnapData snapData)
   {
      footstep.setSnapData(snapData);
      snappedFootsteps.add(footstep);
      manuallySnappedFootsteps.put(footstep, snapData);
   }

   protected void computeWiggleTransform(DiscreteFootstep footstepToWiggle, DiscreteFootstep stanceStep, FootstepSnapData snapData)
   {
      heightMapSnapWiggler.computeWiggleTransform(footstepToWiggle, terrainMapData, snapData);

      if (stanceStep != null && stanceStep.hasSnapData())
      {
         FootstepSnapData stanceStepSnapData = stanceStep.getSnapData();

         // check for overlap
         boolean overlapDetected = stepsAreTooClose(footstepToWiggle, snapData, stanceStep, stanceStepSnapData);
         if (overlapDetected)
         {
            snapData.getWiggleTransformInWorld().setIdentity();
         }

         // check for overlap after this steps wiggle is removed. if still overlapping, remove wiggle on stance step
         overlapDetected = stepsAreTooClose(footstepToWiggle, snapData, stanceStep, stanceStepSnapData);
         if (overlapDetected)
         {
            stanceStepSnapData.getWiggleTransformInWorld().setIdentity();
         }
      }
   }

   protected void updateTheCroppedFoothold(DiscreteFootstep footstepToCrop)
   {
      List<Point3DBasics> footPointsInEnvironment = new ArrayList<>();
      FootstepSnapData snapData = footstepToCrop.getSnapData();

      ConvexPolygon2D footPolygonInFootFrame = new ConvexPolygon2D(snapData.getCroppedFoothold());
      if (footPolygonInFootFrame.getNumberOfVertices() != 4)
      {
         footPolygonInFootFrame.set(footPolygonsInSoleFrame.get(footstepToCrop.getRobotSide()));
      }

      // Get the transform from the sole frame to the desired foot pose.
      RigidBodyTransform transformToFootPose = new RigidBodyTransform();
      RigidBodyTransform snapAndWiggleTransform = new RigidBodyTransform();

      DiscreteFootstepTools.getStepTransform(footstepToCrop, transformToFootPose);
      snapData.packSnapAndWiggleTransform(snapAndWiggleTransform);
      transformToFootPose.preMultiply(snapAndWiggleTransform);

      // Transform the foot polygon to its position in the world after the snap.
      ConvexPolygon2D snappedFootPolygonInWorld = new ConvexPolygon2D(footPolygonInFootFrame);
      snappedFootPolygonInWorld.applyTransform(transformToFootPose, false);

      heightMapSnapper.computeFootPointsInTheEnvironment(snappedFootPolygonInWorld,
                                                         terrainMapData,
                                                         parameters.getHeightMapSnapThreshold(),
                                                         parameters.getMinSurfaceIncline(),
                                                         -Double.MAX_VALUE,
                                                         footPointsInEnvironment);

      List<Point2D> footPointsInFoot = footPointsInEnvironment.stream().map(point ->
                                                                            {
                                                                               Point3D transformedPoint = new Point3D(point);
                                                                               transformedPoint.applyInverseTransform(transformToFootPose);
                                                                               return new Point2D(transformedPoint);
                                                                            }).toList();

      ConvexPolygon2D croppedFoothold = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPointsInFoot));

      if (croppedFoothold.getNumberOfVertices() < 4)
      {
         // something in gift wrapping the polygon failed. Fall back to the original polygon.
         croppedFoothold.set(footPolygonInFootFrame);
      }
      else
      {
         // Check that all points are inside the polygon. If not, gift wrapping failed, and we need to fall back to the original polygon.
         for (Point2DBasics footPoint : footPointsInFoot)
         {
            if (!croppedFoothold.isPointInside(footPoint, 1e-3))
            {
               croppedFoothold.set(footPolygonInFootFrame);
               break;
            }
         }
      }

      // Reduce the foothold down to 4 vertices.
      if (croppedFoothold.getNumberOfVertices() > 4)
         ConvexPolygonTools.limitVerticesConservative(croppedFoothold, 4);

      if (croppedFoothold.getArea() < 0.05 * footPolygonInFootFrame.getArea())
      {
         // something about the foot polygon is too small. Fall back to the original polygon.
         croppedFoothold.set(footPolygonInFootFrame);
      }

      // If this cropped foothold is almost the same as the original, don't bother storing it, and clear the stored one.
      if (footPolygonsInSoleFrame.get(footstepToCrop.getRobotSide()).getArea() - croppedFoothold.getArea() > 1e-3)
         snapData.getCroppedFoothold().set(croppedFoothold);
      else
         snapData.getCroppedFoothold().clear();
   }

   private final RigidBodyTransform transform1 = new RigidBodyTransform();
   private final RigidBodyTransform transform2 = new RigidBodyTransform();
   private final ConvexPolygon2D polygon1 = new ConvexPolygon2D();
   private final ConvexPolygon2D polygon2 = new ConvexPolygon2D();

   /**
    * Extracted to method for testing purposes
    */
   protected boolean stepsAreTooClose(DiscreteFootstep step1, FootstepSnapData snapData1, DiscreteFootstep step2, FootstepSnapData snapData2)
   {
      DiscreteFootstepTools.getFootPolygon(step1, footPolygonsInSoleFrame.get(step1.getRobotSide()), polygon1);
      DiscreteFootstepTools.getFootPolygon(step2, footPolygonsInSoleFrame.get(step2.getRobotSide()), polygon2);

      snapData1.packSnapAndWiggleTransform(transform1);
      snapData2.packSnapAndWiggleTransform(transform2);

      polygon1.applyTransform(transform1, false);
      polygon2.applyTransform(transform2, false);

      boolean intersection = StepConstraintPolygonTools.arePolygonsIntersecting(polygon1, polygon2);
      if (intersection)
      {
         return true;
      }

      double distance = StepConstraintPolygonTools.distanceBetweenPolygons(polygon1, polygon2);
      return distance < parameters.getMinClearanceFromStance();
   }

   private static void updateWiggleParameters(WiggleParameters wiggleParameters, DefaultFootstepPlannerParametersReadOnly parameters)
   {
      wiggleParameters.deltaInside = parameters.getWiggleInsideDeltaTarget();
      wiggleParameters.maxX = parameters.getMaxXYWiggleDistance();
      wiggleParameters.minX = -parameters.getMaxXYWiggleDistance();
      wiggleParameters.maxY = parameters.getMaxXYWiggleDistance();
      wiggleParameters.minY = -parameters.getMaxXYWiggleDistance();
      wiggleParameters.maxYaw = parameters.getMaxYawWiggle();
      wiggleParameters.minYaw = -parameters.getMaxYawWiggle();
   }

   /**
    * Clears snapper history
    */
   public void reset()
   {
      clearSnapData();
   }

   public void setTerrainMapData(TerrainMapData terrainMapData)
   {
      this.terrainMapData = terrainMapData;
   }
}
