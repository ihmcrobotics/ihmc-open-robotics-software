package us.ihmc.pathPlanning.visibilityGraphs.parameters;

import toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket;
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


   default void setObstacleExtrusionDistanceIfNotTooHighToStep(double distance)
   {
      set(obstacleExtrusionDistanceIfNotTooHighToStep, distance);
   }

   default void setTooHighToStepDistance(double distance)
   {
      set(tooHighToStepDistance, distance);
   }

   default void setHeightForMaxAvoidance(double distance)
   {
      set(heightForMaxAvoidance, distance);
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

   default void setWeightForInterRegionEdge(double weight)
   {
      set(weightForInterRegionEdge, weight);
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

   default void setReturnBestEffortSolution(boolean returnBestEffort)
   {
      set(returnBestEffortSolution, returnBestEffort);
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

   default void setOptimizeForNarrowPassage(boolean performOptimization)
   {
      set(optimizeForNarrowPassage, performOptimization);
   }

   default void set(VisibilityGraphsParametersPacket packet)
   {
      double noValue = VisibilityGraphsParametersPacket.DEFAULT_NO_VALUE;

      if (packet.getMaxInterRegionConnectionLength() != noValue)
         setMaxInterRegionConnectionLength(packet.getMaxInterRegionConnectionLength());
      if (packet.getNormalZThresholdForAccessibleRegions() != noValue)
         setNormalZThresholdForAccessibleRegions(packet.getNormalZThresholdForAccessibleRegions());
      if (packet.getNavigableExtrusionDistance() != noValue)
         setNavigableExtrusionDistance(packet.getNavigableExtrusionDistance());
      if (packet.getObstacleExtrusionDistance() != noValue)
         setObstacleExtrusionDistance(packet.getObstacleExtrusionDistance());
      if (packet.getObstacleExtrusionDistanceIfNotTooHighToStep() != noValue)
         setObstacleExtrusionDistanceIfNotTooHighToStep(packet.getObstacleExtrusionDistanceIfNotTooHighToStep());
      if (packet.getTooHighToStepDistance() != noValue)
         setTooHighToStepDistance(packet.getTooHighToStepDistance());
      if (packet.getHeightForMaxAvoidance() != noValue)
         setHeightForMaxAvoidance(packet.getHeightForMaxAvoidance());
      if (packet.getClusterResolution() != noValue)
         setClusterResolution(packet.getClusterResolution());
      if (packet.getExplorationDistanceFromStartGoal() != noValue)
         setExplorationDistanceFromStartGoal(packet.getExplorationDistanceFromStartGoal());
      if (packet.getPlanarRegionMinArea() != noValue)
         setPlanarRegionMinArea(packet.getPlanarRegionMinArea());
      if (packet.getPlanarRegionMinSize() > 0)
         setPlanarRegionMinSize((int) packet.getPlanarRegionMinSize());
      if (packet.getRegionOrthogonalAngle() != noValue)
         setRegionOrthogonalAngle(packet.getRegionOrthogonalAngle());
      if (packet.getSearchHostRegionEpsilon() != noValue)
         setSearchHostRegionEpsilon(packet.getSearchHostRegionEpsilon());
      if (packet.getCanDuckUnderHeight() != noValue)
         setCanDuckUnderHeight(packet.getCanDuckUnderHeight());
      if (packet.getCanEasilyStepOverHeight() != noValue)
         setCanEasilyStepOverHeight(packet.getCanEasilyStepOverHeight());
      if (packet.getLengthForLongInterRegionEdge() != noValue)
         setLengthForLongInterRegionEdge(packet.getLengthForLongInterRegionEdge());
      if (packet.getHeuristicWeight() != noValue)
         setHeuristicWeight(packet.getHeuristicWeight());
      if (packet.getDistanceWeight() != noValue)
         setDistanceWeight(packet.getDistanceWeight());
      if (packet.getElevationWeight() != noValue)
         setElevationWeight(packet.getElevationWeight());
      if (packet.getOccludedGoalEdgeWeight() != noValue)
         setOccludedGoalEdgeWeight(packet.getOccludedGoalEdgeWeight());
      if (packet.getWeightForInterRegionEdge() != noValue)
         setWeightForInterRegionEdge(packet.getWeightForInterRegionEdge());

      setReturnBestEffortSolution(packet.getReturnBestEffortSolution());
      setPerformPostProcessingNodeShifting(packet.getPerformPostProcessingNodeShifting());
      setIntroduceMidpointsInPostProcessing(packet.getIntroduceMidpointsInPostProcessing());
      setComputeOrientationsToAvoidObstacles(packet.getComputeOrientationsToAvoidObstacles());
      setOptimizeForNarrowPassage(packet.getOptimizeForNarrowPassage());
   }

}
