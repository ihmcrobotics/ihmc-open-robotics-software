package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.commonWalkingControlModules.polygonWiggling.StepConstraintPolygonTools;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapSnapWiggler;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashSet;

public class FootstepSnapAndWiggler implements FootstepSnapperReadOnly
{
   private final SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame;
   private final FootstepPlannerParametersReadOnly parameters;

   private final WiggleParameters wiggleParameters = new WiggleParameters();
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private double flatGroundHeight = 0.0;

   private final HashSet<DiscreteFootstep> snappedFootsteps = new HashSet<>();

   private final FootstepPlannerEnvironmentHandler environmentHandler;

   private final HeightMapPolygonSnapper heightMapSnapper = new HeightMapPolygonSnapper();
   private final HeightMapSnapWiggler heightMapSnapWiggler;

   // Use this by default
   public FootstepSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame,
                                 FootstepPlannerParametersReadOnly parameters,
                                 FootstepPlannerEnvironmentHandler environmentHandler)
   {
      this(footPolygonsInSoleFrame, parameters, null, environmentHandler,null, null);
   }

   // Call this constructor only for testing
   public FootstepSnapAndWiggler(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame,
                                 FootstepPlannerParametersReadOnly parameters,
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
         }

         return snapData;
      }
      else if (environmentHandler.flatGroundMode())
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

   protected FootstepSnapData computeSnapTransform(DiscreteFootstep footstepToSnap, DiscreteFootstep stanceStep)
   {
      return heightMapSnapper.computeSnapData(footstepToSnap,
                                              footPolygonsInSoleFrame.get(footstepToSnap.getRobotSide()),
                                              environmentHandler.getHeightMap(),
                                              parameters.getHeightMapSnapThreshold(),
                                              parameters.getMinimumSurfaceInclineRadians());
   }

   /**
    * Can manually add snap data for a footstep to bypass the snapper.
    */
   public void addSnapData(DiscreteFootstep footstep, FootstepSnapData snapData)
   {
      footstep.setSnapData(snapData);
      snappedFootsteps.add(footstep);
   }

   protected void computeWiggleTransform(DiscreteFootstep footstepToWiggle, DiscreteFootstep stanceStep, FootstepSnapData snapData)
   {
      heightMapSnapWiggler.computeWiggleTransform(footstepToWiggle,
                                                  environmentHandler.getHeightMap(),
                                                  snapData,
                                                  parameters.getHeightMapSnapThreshold(),
                                                  parameters.getMinimumSurfaceInclineRadians());

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

   private static void updateWiggleParameters(WiggleParameters wiggleParameters, FootstepPlannerParametersReadOnly parameters)
   {
      wiggleParameters.deltaInside = parameters.getWiggleInsideDeltaTarget();
      wiggleParameters.maxX = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.minX = -parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxY = parameters.getMaximumXYWiggleDistance();
      wiggleParameters.minY = -parameters.getMaximumXYWiggleDistance();
      wiggleParameters.maxYaw = parameters.getMaximumYawWiggle();
      wiggleParameters.minYaw = -parameters.getMaximumYawWiggle();
   }

   /**
    * Clears snapper history
    */
   public void reset()
   {
      clearSnapData();
   }
}
