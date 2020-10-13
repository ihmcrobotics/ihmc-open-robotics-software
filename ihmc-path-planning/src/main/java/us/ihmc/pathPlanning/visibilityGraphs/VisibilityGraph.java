package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.*;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.InterRegionConnectionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.NavigableRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

public class VisibilityGraph
{
   private static final boolean createNavigableRegionNodesOnConstruction = false;
   //TODO: +++JerryPratt: Just get rid of createEdgesAroundClusterRing? Or store Nodes with clusters somehow?
   // Set createEdgesAroundClusterRing to true if at the beginning you want to create loops around the clusters.
   private static final boolean createEdgesAroundClusterRing = false;

   // Flag for whether to just connect the shortest interconnecting edge, or all of them.
   //TODO: Try this on for size for a while and if shortest edge seems like always the best way to go, remove the flag.
   protected static final boolean ONLY_USE_SHORTEST_INTER_CONNECTING_EDGE = true;
   private ArrayList<VisibilityGraphNavigableRegion> visibilityGraphNavigableRegions = new ArrayList<>();
   private final NavigableRegions navigableRegions;
   private final List<VisibilityGraphEdge> crossRegionEdges = new ArrayList<>();

   private VisibilityGraphNode startNode, goalNode;

   private final VisibilityGraphsParametersReadOnly parameters;
   private final InterRegionConnectionFilter interRegionConnectionFilter;
   private final InterRegionConnectionFilter preferredInterRegionConnectionFilter;
   private final InterRegionConnectionFilter preferredToNonPreferredInterRegionConnectionFilter;
   private final InterRegionConnectionFilter allPassFilter;

   public VisibilityGraph(NavigableRegions navigableRegions, InterRegionConnectionFilter interRegionConnectionFilter,
                          InterRegionConnectionFilter preferredInterRegionConnectionFilter,
                          InterRegionConnectionFilter preferredToNonPreferredInterRegionConnectionFilter,
                          VisibilityGraphsParametersReadOnly parameters)
   {
      this.navigableRegions = navigableRegions;
      this.interRegionConnectionFilter = interRegionConnectionFilter;
      this.preferredInterRegionConnectionFilter = preferredInterRegionConnectionFilter;
      this.preferredToNonPreferredInterRegionConnectionFilter = preferredToNonPreferredInterRegionConnectionFilter;
      this.parameters = parameters;

      allPassFilter = new InterRegionConnectionFilter()
      {
         @Override
         public double getMaximumInterRegionConnectionDistance()
         {
            return Double.POSITIVE_INFINITY;
         }

         @Override
         public boolean isConnectionValid(ConnectionPoint3D source, ConnectionPoint3D target)
         {
            return true;
         }
      };

      List<NavigableRegion> navigableRegionsList = navigableRegions.getNavigableRegionsList();

      for (NavigableRegion navigableRegion : navigableRegionsList)
      {
         VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = new VisibilityGraphNavigableRegion(navigableRegion, createEdgesAroundClusterRing);

         if (createNavigableRegionNodesOnConstruction)
            visibilityGraphNavigableRegion.createNavigableRegionNodes();

         visibilityGraphNavigableRegions.add(visibilityGraphNavigableRegion);
      }
   }

   public void fullyExpandVisibilityGraph()
   {
      double nonPreferredWeight = parameters.includePreferredExtrusions() ? parameters.getWeightForNonPreferredEdge() : 1.0;

      for (VisibilityGraphNavigableRegion visibilityGraphNavigableRegion : visibilityGraphNavigableRegions)
      {
         visibilityGraphNavigableRegion.createGraphBetweenInnerClusterRings(nonPreferredWeight);
      }

      // create inter-region connections
      for (int sourceIndex = 0; sourceIndex < visibilityGraphNavigableRegions.size(); sourceIndex++)
      {
         VisibilityGraphNavigableRegion sourceNavigableRegion = visibilityGraphNavigableRegions.get(sourceIndex);

         for (int targetIndex = sourceIndex + 1; targetIndex < visibilityGraphNavigableRegions.size(); targetIndex++)
         {
            VisibilityGraphNavigableRegion targetNavigableRegion = visibilityGraphNavigableRegions.get(targetIndex);
            createInterRegionVisibilityConnections(sourceNavigableRegion, targetNavigableRegion);
         }
      }

      // set edges have been determined
      for (VisibilityGraphNavigableRegion visibilityGraphNavigableRegion : visibilityGraphNavigableRegions)
      {
         List<VisibilityGraphNode> allNavigableNodes = visibilityGraphNavigableRegion.getAllNavigableNodes();
         List<VisibilityGraphNode> allPreferredNavigableNodes = visibilityGraphNavigableRegion.getAllPreferredNavigableNodes();
         for (VisibilityGraphNode node : allNavigableNodes)
            node.setEdgesHaveBeenDetermined(true);
         for (VisibilityGraphNode node : allPreferredNavigableNodes)
            node.setEdgesHaveBeenDetermined(true);
      }
   }

   public void computeInterEdgesWhenOnNoRegion(VisibilityGraphNode sourceNode, InterRegionConnectionFilter filter, double edgeWeight)
   {
      double nonPreferredWeight = parameters.includePreferredExtrusions() ? parameters.getWeightForNonPreferredEdge() : 1.0;

      for (VisibilityGraphNavigableRegion targetVisibilityGraphNavigableRegion : visibilityGraphNavigableRegions)
      {
         NavigableRegion targetNavigableRegion = targetVisibilityGraphNavigableRegion.getNavigableRegion();
         List<Cluster> targetObstacleClusters = targetNavigableRegion.getObstacleClusters();
         List<VisibilityGraphNode> allNavigableNodes = targetVisibilityGraphNavigableRegion.getAllNavigableNodes();
         List<VisibilityGraphNode> allPreferredNavigableNodes = targetVisibilityGraphNavigableRegion.getAllPreferredNavigableNodes();
         createVisibilityConnectionsWhenOnNoRegion(sourceNode, allNavigableNodes, navigableRegions.getNavigableRegionsList(),
                                                   targetObstacleClusters, crossRegionEdges, filter, nonPreferredWeight * edgeWeight,
                                                   parameters.getLengthForLongInterRegionEdge());
         createVisibilityConnectionsWhenOnNoRegion(sourceNode, allPreferredNavigableNodes, navigableRegions.getNavigableRegionsList(),
                                                   targetObstacleClusters, crossRegionEdges, filter, edgeWeight, parameters.getLengthForLongInterRegionEdge());
      }

      sourceNode.setEdgesHaveBeenDetermined(true);
   }

   public void computeInnerAndInterEdges(VisibilityGraphNode sourceNode)
   {
      double nonPreferredWeight = parameters.includePreferredExtrusions() ? parameters.getWeightForNonPreferredEdge() : 1.0;

      VisibilityGraphNavigableRegion sourceVisibilityGraphNavigableRegion = sourceNode.getVisibilityGraphNavigableRegion();
      sourceVisibilityGraphNavigableRegion.addInnerRegionEdgesFromSourceNode(sourceNode, nonPreferredWeight);

      computeInterEdges(sourceNode);

      sourceNode.setEdgesHaveBeenDetermined(true);
   }

   public void computeInterEdges(VisibilityGraphNode sourceNode)
   {
      VisibilityGraphNavigableRegion sourceVisibilityGraphNavigableRegion = sourceNode.getVisibilityGraphNavigableRegion();

      NavigableRegion sourceNavigableRegion = sourceVisibilityGraphNavigableRegion.getNavigableRegion();
      List<Cluster> sourceObstacleClusters = sourceNavigableRegion.getObstacleClusters();
      List<PlanarRegion> sourceObstacleRegions = sourceNavigableRegion.getObstacleRegions();

      List<VisibilityGraphEdge> interEdges = new ArrayList<>();

      double weightToPreferred = 1.0;
      if (parameters.includePreferredExtrusions() && !sourceNode.isPreferredNode())
      {
         weightToPreferred = parameters.getWeightForNonPreferredEdge();
      }

      for (VisibilityGraphNavigableRegion targetVisibilityGraphNavigableRegion : visibilityGraphNavigableRegions)
      {
         if (targetVisibilityGraphNavigableRegion == sourceVisibilityGraphNavigableRegion)
            continue;

         //TOOD: +++JerryPratt: Efficiently add Bounding Box check before going through all combinations. Perhaps have a cached bounding box reachable map?

         NavigableRegion targetNavigableRegion = targetVisibilityGraphNavigableRegion.getNavigableRegion();
         List<Cluster> targetObstacleClusters = targetNavigableRegion.getObstacleClusters();
         List<PlanarRegion> targetObstacleRegions = targetNavigableRegion.getObstacleRegions();

         List<VisibilityGraphNode> allNavigableNodes = targetVisibilityGraphNavigableRegion.getAllNavigableNodes();
         List<VisibilityGraphNode> allPreferredNavigableNodes = targetVisibilityGraphNavigableRegion.getAllPreferredNavigableNodes();

         InterRegionConnectionFilter toPreferredNodeFilter;
         InterRegionConnectionFilter toNonPreferredNodeFilter;
         if (sourceNode.isPreferredNode())
         {
            toPreferredNodeFilter = preferredInterRegionConnectionFilter;
            toNonPreferredNodeFilter = preferredToNonPreferredInterRegionConnectionFilter;
         }
         else
         {
            toPreferredNodeFilter = preferredToNonPreferredInterRegionConnectionFilter;
            toNonPreferredNodeFilter = interRegionConnectionFilter;
         }

         double nonPreferredWeight = parameters.includePreferredExtrusions() ? parameters.getWeightForNonPreferredEdge() : 1.0;

         createInterRegionVisibilityConnections(sourceNode, allPreferredNavigableNodes, sourceObstacleClusters, sourceObstacleRegions, targetObstacleClusters,
                                                targetObstacleRegions, toPreferredNodeFilter, interEdges, parameters.getLengthForLongInterRegionEdge(),
                                                weightToPreferred * parameters.getWeightForInterRegionEdge());

         createInterRegionVisibilityConnections(sourceNode, allNavigableNodes, sourceObstacleClusters, sourceObstacleRegions, targetObstacleClusters,
                                                targetObstacleRegions, toNonPreferredNodeFilter, interEdges, parameters.getLengthForLongInterRegionEdge(),
                                                nonPreferredWeight * parameters.getWeightForInterRegionEdge());
      }

      crossRegionEdges.addAll(interEdges);

      sourceNode.setEdgesHaveBeenDetermined(true);
   }

   public static void connectNodeToInnerRegionNodes(VisibilityGraphNode sourceNode, VisibilityGraphNavigableRegion visibilityGraphNavigableRegion,
                                                    VisibilityGraphNode nodeToAttachToIfInSameRegion, double nonPreferredWeight)
   {
      visibilityGraphNavigableRegion.addInnerRegionEdgesFromSourceNode(sourceNode, nonPreferredWeight);

      if (nodeToAttachToIfInSameRegion != null)
      {
         if ((sourceNode.getVisibilityGraphNavigableRegion() == visibilityGraphNavigableRegion) && (nodeToAttachToIfInSameRegion.getVisibilityGraphNavigableRegion() == visibilityGraphNavigableRegion))
         {
            visibilityGraphNavigableRegion.addInnerEdgeFromSourceToTargetNodeIfVisible(sourceNode, nodeToAttachToIfInSameRegion, 1.0);
         }
      }
      sourceNode.setEdgesHaveBeenDetermined(true);
   }

   public static VisibilityGraphNode createNode(Point3DReadOnly sourceInWorld, VisibilityGraphNavigableRegion visibilityGraphNavigableRegion,
                                                boolean isPreferredNode)
   {
      NavigableRegion navigableRegion = visibilityGraphNavigableRegion.getNavigableRegion();
      Point3D sourceInLocal3D = new Point3D(sourceInWorld);
      navigableRegion.transformFromWorldToLocal(sourceInLocal3D);
      Point2D sourceInLocal = new Point2D(sourceInLocal3D);

      Point3D projectedSourceInWorld = new Point3D(sourceInLocal);
      navigableRegion.transformFromLocalToWorld(projectedSourceInWorld);

      return new VisibilityGraphNode(projectedSourceInWorld, sourceInLocal, visibilityGraphNavigableRegion, isPreferredNode);
   }

   public static VisibilityGraphNode createNodeWithNoRegion(Point3DReadOnly sourceInWorld)
   {
      RigidBodyTransform sourceSpoofTransform = new RigidBodyTransform();
      sourceSpoofTransform.getTranslation().set(sourceInWorld);

      ConvexPolygon2D sourceSpoof = new ConvexPolygon2D();
      sourceSpoof.addVertex(0.0, 0.0);
      sourceSpoof.update();
      PlanarRegion sourceRegion = new PlanarRegion(sourceSpoofTransform, sourceSpoof);
      NavigableRegion navigableRegion = new NavigableRegion(sourceRegion, new Cluster(Cluster.ExtrusionSide.OUTSIDE, Cluster.ClusterType.POLYGON), new ArrayList<>());

      Point3D sourceInLocal3D = new Point3D(sourceInWorld);
      navigableRegion.transformFromWorldToLocal(sourceInLocal3D);
      Point2D sourceInLocal = new Point2D(sourceInLocal3D);

      Point3D projectedSourceInWorld = new Point3D(sourceInLocal);
      navigableRegion.transformFromLocalToWorld(projectedSourceInWorld);

      VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = new VisibilityGraphNavigableRegion(navigableRegion, createEdgesAroundClusterRing);
      return new VisibilityGraphNode(projectedSourceInWorld, sourceInLocal, visibilityGraphNavigableRegion, -1, true);
   }

   public void createInterRegionVisibilityConnections(VisibilityGraphNavigableRegion sourceNavigableRegion,
                                                      VisibilityGraphNavigableRegion targetNavigableRegion)
   {
      double nonPreferredWeight = parameters.includePreferredExtrusions() ? parameters.getWeightForNonPreferredEdge() : 1.0;

      createInterRegionVisibilityConnections(sourceNavigableRegion, targetNavigableRegion, interRegionConnectionFilter, preferredInterRegionConnectionFilter,
                                             preferredToNonPreferredInterRegionConnectionFilter, crossRegionEdges, parameters.getLengthForLongInterRegionEdge(),
                                             parameters.getWeightForInterRegionEdge(), nonPreferredWeight);
   }

   public VisibilityGraphNode getStartNode()
   {
      return startNode;
   }

   public VisibilityGraphNode getGoalNode()
   {
      return goalNode;
   }

   public HashSet<VisibilityGraphEdge> getStartEdges()
   {
      if (startNode == null)
         return null;
      return startNode.getEdges();
   }

   public HashSet<VisibilityGraphEdge> getGoalEdges()
   {
      if (goalNode == null)
         return null;
      return goalNode.getEdges();
   }

   public VisibilityGraphNode setStart(Point3DReadOnly sourceLocationInWorld, double ceilingHeight, double searchHostEpsilon)
   {
      VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = getVisibilityGraphNavigableRegionContainingThisPoint(sourceLocationInWorld, ceilingHeight,
                                                                                                                           searchHostEpsilon);
      if (visibilityGraphNavigableRegion == null)
      {
         startNode = createNodeWithNoRegion(sourceLocationInWorld);
         computeInterEdgesWhenOnNoRegion(startNode, allPassFilter, 1.0);

         if (startNode.getEdges().size() < 1)
         { // the start is contained within a navigable region, but is likely moving between regions because of an obstacle extrusion
            computeInterEdges(startNode);
         }
      }
      else
      {
         double nonPreferredWeight = parameters.includePreferredExtrusions() ? parameters.getWeightForNonPreferredEdge() : 1.0;

         startNode = createNode(sourceLocationInWorld, visibilityGraphNavigableRegion, true);
         connectNodeToInnerRegionNodes(startNode, visibilityGraphNavigableRegion, goalNode, nonPreferredWeight);
         computeInterEdges(startNode);
      }

      return startNode;
   }

   public VisibilityGraphNode setGoal(Point3DReadOnly sourceLocationInWorld, double ceilingHeight, double searchHostEpsilon)
   {
      VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = getVisibilityGraphNavigableRegionContainingThisPoint(sourceLocationInWorld, ceilingHeight,
                                                                                                                           searchHostEpsilon);
      if (visibilityGraphNavigableRegion == null)
      {
         goalNode = createNodeWithNoRegion(sourceLocationInWorld);
         computeInterEdgesWhenOnNoRegion(goalNode, allPassFilter, parameters.getOccludedGoalEdgeWeight());
      }
      else
      {
         double nonPreferredWeight = parameters.includePreferredExtrusions() ? parameters.getWeightForNonPreferredEdge() : 1.0;
         goalNode = createNode(sourceLocationInWorld, visibilityGraphNavigableRegion, true);
         connectNodeToInnerRegionNodes(goalNode, visibilityGraphNavigableRegion, startNode, nonPreferredWeight);
         computeInterEdges(goalNode);
      }

      return goalNode;
   }



   public VisibilityGraphNavigableRegion getVisibilityGraphNavigableRegionContainingThisPoint(Point3DReadOnly sourceLocationInWorld, double ceilingHeight, double searchHostEpsilon)
   {
      NavigableRegion hostNavigableRegion = NavigableRegionTools
            .getNavigableRegionContainingThisPoint(sourceLocationInWorld, navigableRegions, ceilingHeight, searchHostEpsilon);
      return getVisibilityGraphNavigableRegion(hostNavigableRegion);
   }

   private VisibilityGraphNavigableRegion getVisibilityGraphNavigableRegion(NavigableRegion navigableRegion)
   {
      if (navigableRegion == null)
         return null;

      for (VisibilityGraphNavigableRegion visibilityGraphNavigableRegion : visibilityGraphNavigableRegions)
      {
         if (visibilityGraphNavigableRegion.getNavigableRegion() == navigableRegion)
            return visibilityGraphNavigableRegion;
      }

      return null;
   }

   public ArrayList<VisibilityGraphNavigableRegion> getVisibilityGraphNavigableRegions()
   {
      return visibilityGraphNavigableRegions;
   }

   public List<VisibilityGraphEdge> getCrossRegionEdges()
   {
      return crossRegionEdges;
   }

   public static void createInterRegionVisibilityConnections(VisibilityGraphNavigableRegion sourceNavigableRegion,
                                                             VisibilityGraphNavigableRegion targetNavigableRegion,
                                                             InterRegionConnectionFilter interRegionConnectionFilter,
                                                             InterRegionConnectionFilter preferredInterRegionConnectionFilter,
                                                             InterRegionConnectionFilter preferredToNonPreferredInterRegionConnectionFilter,
                                                             List<VisibilityGraphEdge> edgesToPack, double lengthForLongInterRegionEdge,
                                                             double weightForInterRegionEdge, double nonPreferredWeight)
   {
      int sourceId = sourceNavigableRegion.getMapId();
      int targetId = targetNavigableRegion.getMapId();

      if (sourceId == targetId)
         return;

      PlanarRegion sourceHomePlanarRegion = sourceNavigableRegion.getNavigableRegion().getHomePlanarRegion();
      PlanarRegion targetHomePlanarRegion = targetNavigableRegion.getNavigableRegion().getHomePlanarRegion();

      BoundingBox3D sourceHomeRegionBoundingBox = sourceHomePlanarRegion.getBoundingBox3dInWorld();
      BoundingBox3D targetHomeRegionBoundingBox = targetHomePlanarRegion.getBoundingBox3dInWorld();

      // If the source and target regions are simply too far apart, then do not check their individual points.
      if (!sourceHomeRegionBoundingBox.intersectsEpsilon(targetHomeRegionBoundingBox, interRegionConnectionFilter.getMaximumInterRegionConnectionDistance()))
      {
         return;
      }

      List<Cluster> sourceObstacleClusters = sourceNavigableRegion.getNavigableRegion().getObstacleClusters();
      List<PlanarRegion> sourceObstacleRegions = sourceNavigableRegion.getNavigableRegion().getObstacleRegions();
      List<Cluster> targetObstacleClusters = targetNavigableRegion.getNavigableRegion().getObstacleClusters();
      List<PlanarRegion> targetObstacleRegions = targetNavigableRegion.getNavigableRegion().getObstacleRegions();

      List<VisibilityGraphNode> preferredSourceRegionNodes = sourceNavigableRegion.getAllPreferredNavigableNodes();
      List<VisibilityGraphNode> preferredTargetRegionNodes = targetNavigableRegion.getAllPreferredNavigableNodes();
      List<VisibilityGraphNode> sourceRegionNodes = sourceNavigableRegion.getAllNavigableNodes();
      List<VisibilityGraphNode> targetRegionNodes = targetNavigableRegion.getAllNavigableNodes();

      createInterRegionVisibilityConnections(preferredSourceRegionNodes, preferredTargetRegionNodes, sourceRegionNodes, targetRegionNodes,
                                             sourceObstacleClusters, sourceObstacleRegions, targetObstacleClusters, targetObstacleRegions,
                                             interRegionConnectionFilter, preferredInterRegionConnectionFilter,
                                             preferredToNonPreferredInterRegionConnectionFilter, edgesToPack,
                                             lengthForLongInterRegionEdge, weightForInterRegionEdge, nonPreferredWeight);
   }


   public static void createInterRegionVisibilityConnections(List<VisibilityGraphNode> preferredSourceNodeList,
                                                             List<VisibilityGraphNode> preferredTargetNodeList,
                                                             List<VisibilityGraphNode> sourceNodeList, List<VisibilityGraphNode> targetNodeList,
                                                             List<Cluster> sourceObstacleClusters, List<PlanarRegion> sourceObstacleRegions,
                                                             List<Cluster> targetObstacleClusters, List<PlanarRegion> targetObstacleRegions,
                                                             InterRegionConnectionFilter interRegionConnectionFilter,
                                                             InterRegionConnectionFilter preferredInterRegionConnectionFilter,
                                                             InterRegionConnectionFilter preferredToNonPreferredInterRegionConnectionFilter,
                                                             List<VisibilityGraphEdge> edgesToPack, double lengthForLongInterRegionEdge,
                                                             double weightForInterRegionEdge, double nonPreferredWeight)
   {
      // preferred to preferred
      for (VisibilityGraphNode sourceNode : preferredSourceNodeList)
      {
         createInterRegionVisibilityConnections(sourceNode, preferredTargetNodeList, sourceObstacleClusters, sourceObstacleRegions, targetObstacleClusters,
                                                targetObstacleRegions, preferredInterRegionConnectionFilter, edgesToPack, lengthForLongInterRegionEdge,
                                                weightForInterRegionEdge);
      }
      // non-preferred to preferred
      for (VisibilityGraphNode sourceNode : sourceNodeList)
      {
         createInterRegionVisibilityConnections(sourceNode, preferredTargetNodeList, sourceObstacleClusters, sourceObstacleRegions, targetObstacleClusters,
                                                targetObstacleRegions, preferredToNonPreferredInterRegionConnectionFilter, edgesToPack, lengthForLongInterRegionEdge,
                                                nonPreferredWeight * weightForInterRegionEdge);
      }
      // preferred to non-preferred
      for (VisibilityGraphNode sourceNode : preferredSourceNodeList)
      {
         createInterRegionVisibilityConnections(sourceNode, targetNodeList, sourceObstacleClusters, sourceObstacleRegions, targetObstacleClusters,
                                                targetObstacleRegions, preferredToNonPreferredInterRegionConnectionFilter, edgesToPack, lengthForLongInterRegionEdge,
                                                nonPreferredWeight * weightForInterRegionEdge);
      }
      // non-preferred to non-preferred
      for (VisibilityGraphNode sourceNode : sourceNodeList)
      {
         createInterRegionVisibilityConnections(sourceNode, targetNodeList, sourceObstacleClusters, sourceObstacleRegions, targetObstacleClusters,
                                                targetObstacleRegions, interRegionConnectionFilter, edgesToPack, lengthForLongInterRegionEdge,
                                                nonPreferredWeight * weightForInterRegionEdge);
      }
   }

   public static void createInterRegionVisibilityConnections(VisibilityGraphNode sourceNode, List<VisibilityGraphNode> targetNodeList,
                                                             List<Cluster> sourceObstacleClusters, List<PlanarRegion> sourceObstacleRegions,
                                                             List<Cluster> targetObstacleClusters, List<PlanarRegion> targetObstacleRegions,
                                                             InterRegionConnectionFilter filter, List<VisibilityGraphEdge> edgesToPack,
                                                             double lengthForLongInterRegionEdge, double weightForInterRegionEdge)
   {
      createInterRegionVisibilityConnections(sourceNode, targetNodeList, sourceObstacleClusters, sourceObstacleRegions, targetObstacleClusters,
                                             targetObstacleRegions, filter, edgesToPack, ONLY_USE_SHORTEST_INTER_CONNECTING_EDGE, lengthForLongInterRegionEdge,
                                             weightForInterRegionEdge);
   }

   public static void createInterRegionVisibilityConnections(VisibilityGraphNode sourceNode, List<VisibilityGraphNode> targetNodeList,
                                                             List<Cluster> sourceObstacleClusters, List<PlanarRegion> sourceObstacleRegions,
                                                             List<Cluster> targetObstacleClusters, List<PlanarRegion> targetObstacleRegions,
                                                             InterRegionConnectionFilter filter, List<VisibilityGraphEdge> edgesToPack,
                                                             boolean onlyUseShortestEdge, double lengthForLongInterRegionEdge, double weightForInterRegionEdge)
   {
      List<VisibilityGraphEdge> potentialEdges = new ArrayList<>();

      double lengthForLongEdgeSquared = MathTools.square(lengthForLongInterRegionEdge);

      if (VisibilityGraphNavigableRegion.ENABLE_EXPERIMENTAL_SPEEDUP)
      {
         targetNodeList.parallelStream().forEach(targetNode -> addInterEdgeIfVisible(sourceNode, targetNode, sourceObstacleClusters, sourceObstacleRegions,
                                                                                     targetObstacleClusters, targetObstacleRegions, filter,
                                                                                     lengthForLongEdgeSquared, potentialEdges));
      }
      else
      {
         for (VisibilityGraphNode targetNode : targetNodeList)
         {
            addInterEdgeIfVisible(sourceNode, targetNode, sourceObstacleClusters, sourceObstacleRegions, targetObstacleClusters, targetObstacleRegions,
                                  filter, lengthForLongEdgeSquared, potentialEdges);
         }
      }

      if (onlyUseShortestEdge)
      {
         VisibilityGraphEdge shortestEdgeXY = findShortestEdgeXY(sourceNode, potentialEdges);

         if (shortestEdgeXY != null)
         {
            shortestEdgeXY.setEdgeWeight(weightForInterRegionEdge);
            shortestEdgeXY.registerEnds();
            edgesToPack.add(shortestEdgeXY);
         }
      }
      else
      {
         for (VisibilityGraphEdge edge : potentialEdges)
         {
            edge.setEdgeWeight(weightForInterRegionEdge);
            edge.registerEnds();
         }
         edgesToPack.addAll(potentialEdges);
      }
   }

   private static void addInterEdgeIfVisible(VisibilityGraphNode sourceNode, VisibilityGraphNode targetNode, List<Cluster> sourceObstacleClusters,
                                             List<PlanarRegion> sourceObstacleRegions, List<Cluster> targetObstacleClusters,
                                             List<PlanarRegion> targetObstacleRegions, InterRegionConnectionFilter filter,
                                             double lengthForLongInterRegionEdgeSquared,  List<VisibilityGraphEdge> edgesToPack)
   {
      if (VisibilityTools.isInterRegionEdgeValid(sourceNode, targetNode, sourceObstacleClusters, sourceObstacleRegions, targetObstacleClusters,
                                                 targetObstacleRegions, filter, lengthForLongInterRegionEdgeSquared))
      {
         VisibilityGraphEdge edge;
         edge = new VisibilityGraphEdge(sourceNode, targetNode);
         edgesToPack.add(edge);
      }
   }



   public static void createVisibilityConnectionsWhenOnNoRegion(VisibilityGraphNode sourceNode, List<VisibilityGraphNode> allNavigableNodes,
                                                                List<NavigableRegion> allNavigableRegions, List<Cluster> targetObstacleClusters,
                                                                List<VisibilityGraphEdge> edgesToPack, InterRegionConnectionFilter filter, double edgeWeight,
                                                                double lengthForLongInterRegionEdge)
   {
      createVisibilityConnectionsWhenOnNoRegion(sourceNode, allNavigableNodes, allNavigableRegions, targetObstacleClusters, edgesToPack,
                                                ONLY_USE_SHORTEST_INTER_CONNECTING_EDGE, filter, edgeWeight, lengthForLongInterRegionEdge);
   }

   public static void createVisibilityConnectionsWhenOnNoRegion(VisibilityGraphNode sourceNode, List<VisibilityGraphNode> allNavigableNodes,
                                                                List<NavigableRegion> allNavigableRegions, List<Cluster> targetObstacleClusters,
                                                                List<VisibilityGraphEdge> edgesToPack, boolean useOnlyShortestEdge,
                                                                InterRegionConnectionFilter filter, double edgeWeight, double lengthForLongInterRegionEdge)
   {
      ConnectionPoint3D sourceInWorld = sourceNode.getPointInWorld();


      List<VisibilityGraphEdge> potentialEdges = new ArrayList<>();
      double lengthForLongEdgeSquared = MathTools.square(lengthForLongInterRegionEdge);

      for (VisibilityGraphNode targetNode : allNavigableNodes)
      {
         if (!filter.isConnectionValid(sourceNode.getPointInWorld(), targetNode.getPointInWorld()))
            continue;

         ConnectionPoint3D targetInWorld = targetNode.getPointInWorld();
         double xyDistanceSquared = sourceInWorld.distanceXYSquared(targetInWorld);
         if (xyDistanceSquared < lengthForLongEdgeSquared)
         {
            VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
            potentialEdges.add(edge);
         }
         else // Check if the edge is visible if it is a long one.
         {
            PlanarRegion targetHomeRegion = targetNode.getVisibilityGraphNavigableRegion().getNavigableRegion().getHomePlanarRegion();

            Point2DReadOnly targetInTargetLocal = targetNode.getPoint2DInLocal();

            Point3D sourceProjectedVerticallyOntoTarget = PlanarRegionTools.projectInZToPlanarRegion(sourceInWorld, targetHomeRegion);

            targetHomeRegion.transformFromWorldToLocal(sourceProjectedVerticallyOntoTarget);

               //TODO: +++JerryPratt: Inter-region connections and obstacles still needs some thought and some good unit tests.
            boolean checkForPreferredCollisions = sourceNode.isPreferredNode() && targetNode.isPreferredNode();
            boolean sourceIsVisibleThroughTargetObstacles = VisibilityTools.isPointVisibleToPointInSameRegion(targetObstacleClusters, targetInTargetLocal,
                                                                                                              new Point2D(sourceProjectedVerticallyOntoTarget),
                                                                                                              checkForPreferredCollisions);

            if (!sourceIsVisibleThroughTargetObstacles)
            {
               continue;
            }

            boolean sourceIsOnOuterEdge = true;
            for (NavigableRegion navigableRegion : allNavigableRegions)
            {
               Cluster cluster = navigableRegion.getHomeRegionCluster();
               RigidBodyTransform transformFromWorldToLocal = new RigidBodyTransform(cluster.getTransformToWorld());
               transformFromWorldToLocal.invert();

               // FIXME does this need to be vertically projected?
               Point3D targetInRegionLocal = PlanarRegionTools.projectInZToPlanarRegion(targetInWorld, targetHomeRegion);
               Point3D sourceInRegionLocal = PlanarRegionTools.projectInZToPlanarRegion(sourceInWorld, targetHomeRegion);

               transformFromWorldToLocal.transform(targetInRegionLocal);
               transformFromWorldToLocal.transform(sourceInRegionLocal);

               Point2D sourceInRegionLocal2D = new Point2D(sourceInRegionLocal);
               Point2D targetInRegionLocal2D = new Point2D(targetInRegionLocal);
               boolean isPointVisible = VisibilityTools.isPointVisibleInclusive(sourceInRegionLocal2D, targetInRegionLocal2D,
                                                                                cluster.getNavigableExtrusionsInLocal().getPoints(), cluster.isClosed());

               if (!isPointVisible)
               {
                  sourceIsOnOuterEdge = false;
                  break;
               }
            }

            if (sourceIsOnOuterEdge)
            {
               VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
               potentialEdges.add(edge);
            }
         }
      }

      // using an inflated unseen node edge weight encourages exploration of visible areas
      if (useOnlyShortestEdge)
      {
         VisibilityGraphEdge shortestEdgeXY = findShortestEdgeXY(sourceNode, potentialEdges);

         if (shortestEdgeXY != null)
         {
            shortestEdgeXY.setEdgeWeight(edgeWeight);
            shortestEdgeXY.registerEnds();

            edgesToPack.add(shortestEdgeXY);
         }
      }
      else
      {
         for (VisibilityGraphEdge edge : potentialEdges)
         {
            edge.setEdgeWeight(edgeWeight);
            edge.registerEnds();
         }
         edgesToPack.addAll(potentialEdges);
      }
   }

   private static VisibilityGraphEdge findShortestEdgeXY(VisibilityGraphNode sourceNode, List<VisibilityGraphEdge> potentialEdges)
   {
      VisibilityGraphEdge shortestEdgeXY = null;
      double shortestLengthXYSquared = Double.POSITIVE_INFINITY;

      for (VisibilityGraphEdge potentialEdge : potentialEdges)
      {
         VisibilityGraphNode targetNode = potentialEdge.getTargetNode();
         double distanceSquared = sourceNode.distanceXYSquared(targetNode);

         if (distanceSquared < shortestLengthXYSquared)
         {
            shortestEdgeXY = potentialEdge;
            shortestLengthXYSquared = distanceSquared;
         }
      }
      return shortestEdgeXY;
   }

   public VisibilityMapSolution createVisibilityMapSolution()
   {
      VisibilityMapSolution solution = new VisibilityMapSolution();

      solution.setNavigableRegions(navigableRegions);

      ArrayList<VisibilityMapWithNavigableRegion> visibilityMapsWithNavigableRegions = new ArrayList<>();

      for (VisibilityGraphNavigableRegion visibilityGraphNavigableRegion : visibilityGraphNavigableRegions)
      {

         NavigableRegion navigableRegion = visibilityGraphNavigableRegion.getNavigableRegion();
         HashSet<VisibilityGraphEdge> allEdges = visibilityGraphNavigableRegion.getAllEdges();

         VisibilityMapWithNavigableRegion visibilityMapWithNavigableRegion = new VisibilityMapWithNavigableRegion(navigableRegion);

         List<Connection> connections = createConnectionsFromEdges(allEdges);

         VisibilityMap visibilityMapInWorld = new VisibilityMap(connections);
         visibilityMapWithNavigableRegion.setVisibilityMapInWorld(visibilityMapInWorld);
         visibilityMapsWithNavigableRegions.add(visibilityMapWithNavigableRegion);
      }

      solution.setVisibilityMapsWithNavigableRegions(visibilityMapsWithNavigableRegions);

      InterRegionVisibilityMap interRegionVisibilityMap = new InterRegionVisibilityMap();

      for (VisibilityGraphEdge crossRegionEdge : crossRegionEdges)
      {
         Connection connection = new Connection(crossRegionEdge.getSourcePointInWorld(), crossRegionEdge.getTargetPointInWorld());
         interRegionVisibilityMap.addConnection(connection);
      }

      if (startNode != null)
      {
         List<Connection> startConnections = createConnectionsFromEdges(startNode.getEdges());
         SingleSourceVisibilityMap startMap = new SingleSourceVisibilityMap(startNode.getPointInWorld(), startNode.getRegionId(), startConnections);
         solution.setStartMap(startMap);
      }

      if (goalNode != null)
      {
         List<Connection> goalConnections = createConnectionsFromEdges(goalNode.getEdges());
         SingleSourceVisibilityMap goalMap = new SingleSourceVisibilityMap(goalNode.getPointInWorld(), goalNode.getRegionId(), goalConnections);
         solution.setGoalMap(goalMap);
      }

      solution.setInterRegionVisibilityMap(interRegionVisibilityMap);
      return solution;
   }

   private List<Connection> createConnectionsFromEdges(HashSet<VisibilityGraphEdge> edges)
   {
      List<Connection> connections = new ArrayList<>();

      for (VisibilityGraphEdge edge : edges)
      {
         if (edge == null)
         {
            // not sure why this is null sometimes
         }
         else
         {
            ConnectionPoint3D sourcePoint = edge.getSourcePointInWorld();
            ConnectionPoint3D targetPoint = edge.getTargetPointInWorld();
            connections.add(new Connection(sourcePoint, targetPoint));
         }
      }

      return connections;
   }

}
