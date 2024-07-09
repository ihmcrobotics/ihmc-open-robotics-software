package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapDataReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ObstacleBetweenStepsChecker
{
   private static final boolean DEBUG = false;

   private HeightMapData heightMapData;
   private final FootstepSnapperReadOnly snapper;
   private final BooleanSupplier checkForPathCollisions;
   private final DoubleSupplier idealFootstepWidth;
   private final DoubleSupplier heightOffset;
   private final DoubleSupplier heightExtrusion;

   public ObstacleBetweenStepsChecker(DefaultFootstepPlannerParametersReadOnly parameters, FootstepSnapperReadOnly snapper)
   {
      this(snapper, parameters::getCheckForPathCollisions, parameters::getIdealFootstepWidth, parameters::getBodyBoxBaseZ, parameters::getBodyBoxHeight);
   }

   public ObstacleBetweenStepsChecker(FootstepSnapperReadOnly snapper,
                                      BooleanSupplier checkForPathCollisions,
                                      DoubleSupplier idealFootstepWidth,
                                      DoubleSupplier heightOffset,
                                      DoubleSupplier heightExtrusion)
   {
      this.snapper = snapper;
      this.checkForPathCollisions = checkForPathCollisions;
      this.idealFootstepWidth = idealFootstepWidth;
      this.heightOffset = heightOffset;
      this.heightExtrusion = heightExtrusion;
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      this.heightMapData = heightMapData;
   }

   boolean hasHeightMapData()
   {
      return heightMapData != null && !heightMapData.isEmpty();
   }

   public boolean isFootstepValid(DiscreteFootstep footstep, DiscreteFootstep previousStep)
   {
      if (previousStep == null || !checkForPathCollisions.getAsBoolean() || !hasHeightMapData())
         return true;

      FootstepSnapDataReadOnly snapData = snapper.snapFootstep(footstep);
      if (snapData == null)
      {
         return true;
      }

      RigidBodyTransformReadOnly snapTransform = snapData.getSnapTransform();

      FootstepSnapDataReadOnly previousStepSnapData = snapper.snapFootstep(previousStep);
      RigidBodyTransformReadOnly previousSnapTransform = previousStepSnapData.getSnapTransform();

      Point3D stepPosition = new Point3D(footstep.getOrComputeMidFootPoint(idealFootstepWidth.getAsDouble()));
      Point3D previousStepPosition = new Point3D(previousStep.getOrComputeMidFootPoint(idealFootstepWidth.getAsDouble()));

      snapTransform.transform(stepPosition);
      previousSnapTransform.transform(previousStepPosition);

      if (isObstacleBetweenSteps(stepPosition, previousStepPosition, heightMapData))
      {
         if (DEBUG)
         {
            LogTools.debug("Found a obstacle between the steps " + footstep + " and " + previousStep);
         }
         return false;
      }

      return true;
   }

   /**
    * This is meant to test if there is a wall that the body of the robot would run into when shifting
    * from one step to the next. It is not meant to eliminate swing overs.
    */
   private boolean isObstacleBetweenSteps(Point3DReadOnly stepPosition, Point3DReadOnly previousStepPosition, HeightMapData heightMapData)
   {
      double groundClearance = heightOffset.getAsDouble();
      double regionHeight = heightExtrusion.getAsDouble();

      double maxZofGroundForCollision = Math.max(stepPosition.getZ(), previousStepPosition.getZ()) + groundClearance;

      int pointsToCheck = computePointsToCheck(stepPosition, previousStepPosition, heightMapData.getGridResolutionXY());

      for (int i = 0; i <= pointsToCheck; i++)
      {
         double alpha = ((double) i) / pointsToCheck;
         double x = InterpolationTools.linearInterpolate(stepPosition.getX(), previousStepPosition.getX(), alpha);
         double y = InterpolationTools.linearInterpolate(stepPosition.getY(), previousStepPosition.getY(), alpha);

         if (heightMapData.getHeightAt(x, y) > maxZofGroundForCollision)
            return true;
      }

      return false;
   }

   private static int computePointsToCheck(Point3DReadOnly pointA, Point3DReadOnly pointB, double resolutionXY)
   {
      double distanceXY = pointA.distanceXY(pointB);
      return ((int) Math.ceil(distanceXY / resolutionXY)) + 1;
   }
}
