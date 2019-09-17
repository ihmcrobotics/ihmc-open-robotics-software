package us.ihmc.pathPlanning.visibilityGraphs.parameters;

import controller_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.tools.property.StoredPropertySetBasics;

import static us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphParametersKeys.*;

public interface VisibilityGraphsParametersBasics extends VisibilityGraphsParametersReadOnly, StoredPropertySetBasics
{
   default void set(VisibilityGraphsParametersReadOnly parameters)
   {
      setAll(parameters.getAll());
   }

   default void setMaxInterRegionConnectionLength(double maxLength)
   {
      set(maxInterRegionConnectionLength, maxLength);
   }

   default void setNormalZThresholdForAccessibleRegions(double threshold)
   {
      set(normalZThresholdForAccessibleRegions, threshold);
   }

   default void setNavigableExtrusionDistance(double distance)
   {
      set(navigableExtrusionDistance, distance);
   }

   default void setObstacleExtrusionDistance(double distance)
   {
      set(obstacleExtrusionDistance, distance);
   }

   default void setPreferredObstacleExtrusionDistance(double distance)
   {
      set(preferredObstacleExtrusionDistance, distance);
   }

   default void setObstacleExtrusionDistanceIfNotTooHighToStep(double distance)
   {
      set(obstacleExtrusionDistanceIfNotTooHighToStep, distance);
   }

   default void setTooHighToStepDistance(double distance)
   {
      set(tooHighToStepDistance, distance);
   }

   default void setClusterResolution(double resolution)
   {
      set(clusterResolution, resolution);
   }

   default void setExplorationDistanceFromStartGoal(double distance)
   {
      set(explorationDistanceFromStartGoal, distance);
   }

   default void setPlanarRegionMinArea(double area)
   {
      set(planarRegionMinArea, area);
   }

   default void setPlanarRegionMinSize(int size)
   {
      set(planarRegionMinSize, size);
   }

   default void setRegionOrthogonalAngle(double angle)
   {
      set(regionOrthogonalAngle, angle);
   }

   default void setSearchHostRegionEpsilon(double epsilon)
   {
      set(searchHostRegionEpsilon, epsilon);
   }

   default void setCanDuckUnderHeight(double height)
   {
      set(canDuckUnderHeight, height);
   }

   default void setCanEasilyStepOverHeight(double height)
   {
      set(canEasilyStepOverHeight, height);
   }

   default void setLengthForLongInterRegionEdge(double length)
   {
      set(lengthForLongInterRegionEdge, length);
   }

   default void setPerformPostProcessingNodeShifting(boolean perform)
   {
      set(performPostProcessingNodeShifting, perform);
   }

   default void setIntroduceMidpointsInPostProcessing(boolean introduce)
   {
      set(introduceMidpointsInPostProcessing, introduce);
   }

   default void setComputeOrientationsToAvoidObstacles(boolean compute)
   {
      set(computeOrientationsToAvoidObstacles, compute);
   }

   default void setHeuristicWeight(double weight)
   {
      set(heuristicWeight, weight);
   }

   default void setDistanceWeight(double weight)
   {
      set(distanceWeight, weight);
   }

   default void setElevationWeight(double weight)
   {
      set(elevationWeight, weight);
   }

   default void setOccludedGoalEdgeWeight(double weight)
   {
      set(occludedGoalEdgeWeight, weight);
   }

   default void set(VisibilityGraphsParametersPacket packet)
   {
      if (packet.getMaxInterRegionConnectionLength() != -1.0)
         setMaxInterRegionConnectionLength(packet.getMaxInterRegionConnectionLength());
      if (packet.getNormalZThresholdForAccessibleRegions() != -1.0)
         setNormalZThresholdForAccessibleRegions(packet.getNormalZThresholdForAccessibleRegions());
      if (packet.getNavigableExtrusionDistance() != -1.0)
         setNavigableExtrusionDistance(packet.getNavigableExtrusionDistance());
      if (packet.getObstacleExtrusionDistance() != -1.0)
         setObstacleExtrusionDistance(packet.getObstacleExtrusionDistance());
      if (packet.getPreferredObstacleExtrusionDistance() != -1.0)
         setPreferredObstacleExtrusionDistance(packet.getPreferredObstacleExtrusionDistance());
      if (packet.getObstacleExtrusionDistanceIfNotTooHighToStep() != -1.0)
         setObstacleExtrusionDistanceIfNotTooHighToStep(packet.getObstacleExtrusionDistanceIfNotTooHighToStep());
      if (packet.getTooHighToStepDistance() != -1.0)
         setTooHighToStepDistance(packet.getTooHighToStepDistance());
      if (packet.getClusterResolution() != -1.0)
         setClusterResolution(packet.getClusterResolution());
      if (packet.getExplorationDistanceFromStartGoal() != -1.0)
         setExplorationDistanceFromStartGoal(packet.getExplorationDistanceFromStartGoal());
      if (packet.getPlanarRegionMinArea() != -1.0)
         setPlanarRegionMinArea(packet.getPlanarRegionMinArea());
      if (packet.getPlanarRegionMinSize() != -1)
         setPlanarRegionMinSize((int) packet.getPlanarRegionMinSize());
      if (packet.getRegionOrthogonalAngle() != -1.0)
         setRegionOrthogonalAngle(packet.getRegionOrthogonalAngle());
      if (packet.getSearchHostRegionEpsilon() != -1.0)
         setSearchHostRegionEpsilon(packet.getSearchHostRegionEpsilon());
      if (packet.getCanDuckUnderHeight() != -1.0)
         setCanDuckUnderHeight(packet.getCanDuckUnderHeight());
      if (packet.getCanEasilyStepOverHeight() != -1.0)
         setCanEasilyStepOverHeight(packet.getCanEasilyStepOverHeight());
      if (packet.getLengthForLongInterRegionEdge() != -1.0)
         setLengthForLongInterRegionEdge(packet.getLengthForLongInterRegionEdge());
      if (packet.getHeuristicWeight() != -1.0)
         setHeuristicWeight(packet.getHeuristicWeight());
      if (packet.getDistanceWeight() != -1.0)
         setDistanceWeight(packet.getDistanceWeight());
      if (packet.getElevationWeight() != -1.0)
         setElevationWeight(packet.getElevationWeight());
      if (packet.getOccludedGoalEdgeWeight() != -1.0)
         setOccludedGoalEdgeWeight(packet.getOccludedGoalEdgeWeight());

      setPerformPostProcessingNodeShifting(packet.getPerformPostProcessingNodeShifting());
      setIntroduceMidpointsInPostProcessing(packet.getIntroduceMidpointsInPostProcessing());
      setComputeOrientationsToAvoidObstacles(packet.getComputeOrientationsToAvoidObstacles());
   }

}
