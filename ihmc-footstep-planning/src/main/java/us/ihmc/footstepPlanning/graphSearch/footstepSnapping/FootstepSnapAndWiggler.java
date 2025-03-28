package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.commonWalkingControlModules.polygonWiggling.StepConstraintPolygonTools;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapSnapWiggler;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
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

   private final FootstepPlannerEnvironmentHandler environmentHandler;

   private final HeightMapPolygonSnapper heightMapSnapper = new HeightMapPolygonSnapper();
   private final HeightMapSnapWiggler heightMapSnapWiggler;

   // Use this by default
   public FootstepSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame,
                                 DefaultFootstepPlannerParametersReadOnly parameters,
                                 FootstepPlannerEnvironmentHandler environmentHandler)
   {
      this(footPolygonsInSoleFrame, parameters, null, environmentHandler,null, null);
   }

   // Call this constructor only for testing
   public FootstepSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame,
                                 DefaultFootstepPlannerParametersReadOnly parameters,
                                 TickAndUpdatable tickAndUpdatable,
                                 FootstepPlannerEnvironmentHandler environmentHandler,
                                 YoGraphicsListRegistry graphicsListRegistry,
                                 YoRegistry parentRegistry)
   {
      this.footPolygonsInSoleFrame = footPolygonsInSoleFrame;
      this.parameters = parameters;
      this.heightMapSnapWiggler = new HeightMapSnapWiggler(footPolygonsInSoleFrame, wiggleParameters);
      this.environmentHandler = environmentHandler;
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
         if (environmentHandler.flatGroundMode())
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
      return heightMapSnapper.computeSnapData(footstepToSnap,
                                              footPolygonsInSoleFrame.get(footstepToSnap.getRobotSide()),
                                              environmentHandler,
                                              parameters.getHeightMapSnapThreshold(),
                                              parameters.getMinSurfaceIncline());
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
      heightMapSnapWiggler.computeWiggleTransform(footstepToWiggle,
                                                  environmentHandler,
                                                  snapData,
                                                  parameters.getHeightMapSnapThreshold(),
                                                  parameters.getMinSurfaceIncline());

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
                                                         environmentHandler.getHeightMap(),
                                                         environmentHandler.getTerrainMapData(),
                                                         parameters.getHeightMapSnapThreshold(),
                                                         parameters.getMinSurfaceIncline(),
                                                         -Double.MAX_VALUE,
                                                         footPointsInEnvironment);

      ConvexPolygon2D croppedFoothold = new ConvexPolygon2D(Vertex3DSupplier.asVertex3DSupplier(footPointsInEnvironment));
      croppedFoothold.applyInverseTransform(transformToFootPose, false);

      // Reduce the foothold down to 4 vertices.
      if (croppedFoothold.getNumberOfVertices() > 4)
         ConvexPolygonTools.limitVerticesConservative(croppedFoothold, 4);

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

   public static void main(String[] args)
   {
      List<Point2D> points = new ArrayList<>();
      points.add(new Point2D(1.763, -0.008));
      points.add(new Point2D(1.776, 0.028));
      points.add(new Point2D(1.846, -0.031));
      points.add(new Point2D(1.888, -0.042));
      points.add(new Point2D(1.929, -0.053));
      points.add(new Point2D(1.971, -0.065));
      points.add(new Point2D(1.769, 0.010));
      points.add(new Point2D(1.810, -0.003));
      points.add(new Point2D(1.851, -0.015));
      points.add(new Point2D(1.893, -0.028));
      points.add(new Point2D(1.934, -0.041));
      points.add(new Point2D(1.975, -0.054));
      points.add(new Point2D(1.804, -0.019));
      points.add(new Point2D(1.816, -0.014));
      points.add(new Point2D(1.857, -0.000));
      points.add(new Point2D(1.898, -0.014));
      points.add(new Point2D(1.938, -0.028));
      points.add(new Point2D(1.979, -0.042));
      points.add(new Point2D(1.782, 0.046));
      points.add(new Point2D(1.822, 0.030));
      points.add(new Point2D(1.863, 0.015));
      points.add(new Point2D(1.903, 0.000));
      points.add(new Point2D(1.943, -0.031));
      points.add(new Point2D(1.789, 0.064));
      points.add(new Point2D(1.828, 0.047));
      points.add(new Point2D(1.868, 0.030));
      points.add(new Point2D(1.908, 0.014));
      points.add(new Point2D(1.947, -0.003));
      points.add(new Point2D(1.987, -0.020));
      points.add(new Point2D(1.795, 0.081));
      points.add(new Point2D(1.834, 0.063));
      points.add(new Point2D(1.874, 0.045));
      points.add(new Point2D(1.913, 0.027));
      points.add(new Point2D(1.952, 0.009));
      points.add(new Point2D(1.991, -0.009));

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      points.forEach(polygon::addVertex);
      polygon.update();

      for (Point2D point : points)
      {
         assert(polygon.isPointInside(point));
      }
   }
}
