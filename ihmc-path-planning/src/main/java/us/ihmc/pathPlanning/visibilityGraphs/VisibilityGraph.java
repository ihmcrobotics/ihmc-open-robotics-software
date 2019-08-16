package us.ihmc.pathPlanning.visibilityGraphs;

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
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.List;

public class VisibilityGraph
{
   // Flag for whether to just connect the shortest interconnecting edge, or all of them.
   //TODO: Try this on for size for a while and if shortest edge seems like always the best way to go, remove the flag.
   protected static final boolean ONLY_USE_SHORTEST_INTER_CONNECTING_EDGE = false;
   private ArrayList<VisibilityGraphNavigableRegion> visibilityGraphNavigableRegions = new ArrayList<>();
   private final NavigableRegions navigableRegions;
   private final List<VisibilityGraphEdge> crossRegionEdges = new ArrayList<>();

   private VisibilityGraphNode startNode, goalNode;

   private final VisibilityGraphsParametersReadOnly parameters;
   private final InterRegionConnectionFilter interRegionConnectionFilter;
   private final InterRegionConnectionFilter allPassFilter;

   public VisibilityGraph(NavigableRegions navigableRegions, InterRegionConnectionFilter interRegionConnectionFilter,
                          VisibilityGraphsParametersReadOnly parameters)
   {
      this.navigableRegions = navigableRegions;
      this.interRegionConnectionFilter = interRegionConnectionFilter;
      this.parameters = parameters;

      allPassFilter = new InterRegionConnectionFilter()
      {
         @Override
         public double getMaximumInterRegionConnetionDistance()
         {
            return Double.POSITIVE_INFINITY;
         }

         @Override
         public boolean isConnectionValid(ConnectionPoint3D source, ConnectionPoint3D target)
         {
            return true;
         }
      };

      List<NavigableRegion> navigableRegionsList = navigableRegions.getNaviableRegionsList();

      for (NavigableRegion navigableRegion : navigableRegionsList)
      {
         VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = new VisibilityGraphNavigableRegion(navigableRegion);

         //TODO: +++JerryPratt: Just get rid of createEdgesAroundClusterRing? Or store Nodes with clusters somehow?
         // Set createEdgesAroundClusterRing to true if at the beginning you want to create loops around the clusters.
         boolean createEdgesAroundClusterRing = false;

         visibilityGraphNavigableRegion.createNavigableRegionNodes(createEdgesAroundClusterRing);

         visibilityGraphNavigableRegions.add(visibilityGraphNavigableRegion);
      }
   }

   public void fullyExpandVisibilityGraph()
   {
      for (VisibilityGraphNavigableRegion visibilityGraphNavigableRegion : visibilityGraphNavigableRegions)
      {
         visibilityGraphNavigableRegion.createGraphBetweenInnerClusterRings();
      }

      for (int sourceIndex = 0; sourceIndex < visibilityGraphNavigableRegions.size(); sourceIndex++)
      {
         VisibilityGraphNavigableRegion sourceNavigableRegion = visibilityGraphNavigableRegions.get(sourceIndex);

         for (int targetIndex = sourceIndex + 1; targetIndex < visibilityGraphNavigableRegions.size(); targetIndex++)
         {
            VisibilityGraphNavigableRegion targetNavigableRegion = visibilityGraphNavigableRegions.get(targetIndex);
            createInterRegionVisibilityConnections(sourceNavigableRegion, targetNavigableRegion);
         }
      }

      for (VisibilityGraphNavigableRegion visibilityGraphNavigableRegion : visibilityGraphNavigableRegions)
      {
         List<VisibilityGraphNode> allNavigableNodes = visibilityGraphNavigableRegion.getAllNavigableNodes();
         for (VisibilityGraphNode node : allNavigableNodes)
         {
            node.setEdgesHaveBeenDetermined(true);
         }
      }
   }

   public void computeInterEdgesWhenOnNoRegion(VisibilityGraphNode sourceNode, InterRegionConnectionFilter filter, double edgeWeight)
   {
      for (VisibilityGraphNavigableRegion targetVisibilityGraphNavigableRegion : visibilityGraphNavigableRegions)
      {
         NavigableRegion targetNavigableRegion = targetVisibilityGraphNavigableRegion.getNavigableRegion();
         List<Cluster> targetObstacleClusters = targetNavigableRegion.getObstacleClusters();
         List<VisibilityGraphNode> allNavigableNodes = targetVisibilityGraphNavigableRegion.getAllNavigableNodes();
         createVisibilityConnectionsWhenOnNoRegion(sourceNode, allNavigableNodes, navigableRegions.getNaviableRegionsList(),
                                                   targetObstacleClusters, crossRegionEdges, filter, edgeWeight, parameters.getLengthForLongInterRegionEdge());
      }

      sourceNode.setEdgesHaveBeenDetermined(true);
   }

   public void computeInnerAndInterEdges(VisibilityGraphNode sourceNode)
   {
      VisibilityGraphNavigableRegion sourceVisibilityGraphNavigableRegion = sourceNode.getVisibilityGraphNavigableRegion();
      sourceVisibilityGraphNavigableRegion.addInnerRegionEdgesFromSourceNode(sourceNode);

      computeInterEdges(sourceNode);

      sourceNode.setEdgesHaveBeenDetermined(true);
   }

   public void computeInterEdges(VisibilityGraphNode sourceNode)
   {
      VisibilityGraphNavigableRegion sourceVisibilityGraphNavigableRegion = sourceNode.getVisibilityGraphNavigableRegion();

      NavigableRegion sourceNavigableRegion = sourceVisibilityGraphNavigableRegion.getNavigableRegion();
      List<Cluster> sourceObstacleClusters = sourceNavigableRegion.getObstacleClusters();

      List<VisibilityGraphEdge> interEdges = new ArrayList<>();
      for (VisibilityGraphNavigableRegion targetVisibilityGraphNavigableRegion : visibilityGraphNavigableRegions)
      {
         if (targetVisibilityGraphNavigableRegion == sourceVisibilityGraphNavigableRegion)
            continue;

         //TOOD: +++JerryPratt: Efficiently add Bounding Box check before going through all combinations. Perhaps have a cached bounding box reachable map?

         NavigableRegion targetNavigableRegion = targetVisibilityGraphNavigableRegion.getNavigableRegion();
         List<Cluster> targetObstacleClusters = targetNavigableRegion.getObstacleClusters();

         List<VisibilityGraphNode> allNavigableNodes = targetVisibilityGraphNavigableRegion.getAllNavigableNodes();
         createInterRegionVisibilityConnections(sourceNode, allNavigableNodes, sourceObstacleClusters, targetObstacleClusters, interRegionConnectionFilter,
                                                interEdges, parameters.getLengthForLongInterRegionEdge(), parameters.getWeightForInterRegionEdge());
      }

      crossRegionEdges.addAll(interEdges);

      sourceNode.setEdgesHaveBeenDetermined(true);
   }

   public static void connectNodeToInnerRegionNodes(VisibilityGraphNode sourceNode, VisibilityGraphNavigableRegion visibilityGraphNavigableRegion,
                                                    VisibilityGraphNode nodeToAttachToIfInSameRegion)
   {
      visibilityGraphNavigableRegion.addInnerRegionEdgesFromSourceNode(sourceNode);

      if (nodeToAttachToIfInSameRegion != null)
      {
         if ((sourceNode.getVisibilityGraphNavigableRegion() == visibilityGraphNavigableRegion) && (nodeToAttachToIfInSameRegion.getVisibilityGraphNavigableRegion() == visibilityGraphNavigableRegion))
         {
            visibilityGraphNavigableRegion.addInnerEdgeFromSourceToTargetNodeIfVisible(sourceNode, nodeToAttachToIfInSameRegion);
         }
      }
      sourceNode.setEdgesHaveBeenDetermined(true);
   }

   public static VisibilityGraphNode createNode(Point3DReadOnly sourceInWorld, VisibilityGraphNavigableRegion visibilityGraphNavigableRegion)
   {
      NavigableRegion navigableRegion = visibilityGraphNavigableRegion.getNavigableRegion();
      Point3D sourceInLocal3D = new Point3D(sourceInWorld);
      navigableRegion.transformFromWorldToLocal(sourceInLocal3D);
      Point2D sourceInLocal = new Point2D(sourceInLocal3D);

      Point3D projectedSourceInWorld = new Point3D(sourceInLocal);
      navigableRegion.transformFromLocalToWorld(projectedSourceInWorld);

      return new VisibilityGraphNode(projectedSourceInWorld, sourceInLocal, visibilityGraphNavigableRegion);
   }

   public static VisibilityGraphNode createNodeWithNoRegion(Point3DReadOnly sourceInWorld)
   {
      RigidBodyTransform sourceSpoofTransform = new RigidBodyTransform();
      sourceSpoofTransform.setTranslation(sourceInWorld);

      ConvexPolygon2D sourceSpoof = new ConvexPolygon2D();
      sourceSpoof.addVertex(0.0, 0.0);
      sourceSpoof.update();
      PlanarRegion sourceRegion = new PlanarRegion(sourceSpoofTransform, sourceSpoof);
      NavigableRegion navigableRegion = new NavigableRegion(sourceRegion);

      Point3D sourceInLocal3D = new Point3D(sourceInWorld);
      navigableRegion.transformFromWorldToLocal(sourceInLocal3D);
      Point2D sourceInLocal = new Point2D(sourceInLocal3D);

      Point3D projectedSourceInWorld = new Point3D(sourceInLocal);
      navigableRegion.transformFromLocalToWorld(projectedSourceInWorld);

      VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = new VisibilityGraphNavigableRegion(navigableRegion);
      return new VisibilityGraphNode(projectedSourceInWorld, sourceInLocal, visibilityGraphNavigableRegion, -1);
   }

   public void createInterRegionVisibilityConnections(VisibilityGraphNavigableRegion sourceNavigableRegion,
                                                      VisibilityGraphNavigableRegion targetNavigableRegion)
   {
      createInterRegionVisibilityConnections(sourceNavigableRegion, targetNavigableRegion, interRegionConnectionFilter, crossRegionEdges,
                                             parameters.getLengthForLongInterRegionEdge(), parameters.getWeightForInterRegionEdge());
   }

   public VisibilityGraphNode getStartNode()
   {
      return startNode;
   }

   public VisibilityGraphNode getGoalNode()
   {
      return goalNode;
   }

   public List<VisibilityGraphEdge> getStartEdges()
   {
      if (startNode == null)
         return null;
      return startNode.getEdges();
   }

   public List<VisibilityGraphEdge> getGoalEdges()
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
      }
      else
      {
         startNode = createNode(sourceLocationInWorld, visibilityGraphNavigableRegion);
         connectNodeToInnerRegionNodes(startNode, visibilityGraphNavigableRegion, goalNode);
      }

      if (startNode.getEdges().size() < 1)
      { // the start is contained within a navigable region, but is likely moving between regions because of an obstacle extrusion
         computeInterEdges(startNode);
      }

      if (startNode.getEdges().size() == 0)
         throw new RuntimeException("We have a problem.");

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
         goalNode = createNode(sourceLocationInWorld, visibilityGraphNavigableRegion);
         connectNodeToInnerRegionNodes(goalNode, visibilityGraphNavigableRegion, startNode);
      }

      return goalNode;
   }



   private VisibilityGraphNavigableRegion getVisibilityGraphNavigableRegionContainingThisPoint(Point3DReadOnly sourceLocationInWorld, double ceilingHeight, double searchHostEpsilon)
   {
      NavigableRegion hostNavigableRegion = NavigableRegionTools
            .getNavigableRegionContainingThisPoint(sourceLocationInWorld, navigableRegions, ceilingHeight, searchHostEpsilon);
      VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = getVisibilityGraphNavigableRegion(hostNavigableRegion);
      return visibilityGraphNavigableRegion;
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
                                                             VisibilityGraphNavigableRegion targetNavigableRegion, InterRegionConnectionFilter filter,
                                                             List<VisibilityGraphEdge> edgesToPack, double lengthForLongInterRegionEdge,
                                                             double weightForInterRegionEdge)
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
      if (!sourceHomeRegionBoundingBox.intersectsEpsilon(targetHomeRegionBoundingBox, filter.getMaximumInterRegionConnetionDistance()))
      {
         return;
      }

      List<Cluster> sourceObstacleClusters = sourceNavigableRegion.getNavigableRegion().getObstacleClusters();
      List<Cluster> targetObstacleClusters = targetNavigableRegion.getNavigableRegion().getObstacleClusters();

      List<VisibilityGraphNode> sourceRegionNodes = sourceNavigableRegion.getAllNavigableNodes();
      List<VisibilityGraphNode> targetRegionNodes = targetNavigableRegion.getAllNavigableNodes();

      createInterRegionVisibilityConnections(sourceRegionNodes, targetRegionNodes, sourceObstacleClusters, targetObstacleClusters, filter, edgesToPack,
                                             lengthForLongInterRegionEdge, weightForInterRegionEdge);
   }


   public static void createInterRegionVisibilityConnections(List<VisibilityGraphNode> sourceNodeList, List<VisibilityGraphNode> targetNodeList,
                                                             List<Cluster> sourceObstacleClusters, List<Cluster> targetObstacleClusters,
                                                             InterRegionConnectionFilter filter, List<VisibilityGraphEdge> edgesToPack,
                                                             double lengthForLongInterRegionEdge, double weightForInterRegionEdge)
   {
      for (VisibilityGraphNode sourceNode : sourceNodeList)
      {
         createInterRegionVisibilityConnections(sourceNode, targetNodeList, sourceObstacleClusters, targetObstacleClusters, filter, edgesToPack,
                                                lengthForLongInterRegionEdge, weightForInterRegionEdge);
      }
   }

   public static void createInterRegionVisibilityConnections(VisibilityGraphNode sourceNode, List<VisibilityGraphNode> targetNodeList,
                                                             List<Cluster> sourceObstacleClusters, List<Cluster> targetObstacleClusters,
                                                             InterRegionConnectionFilter filter, List<VisibilityGraphEdge> edgesToPack,
                                                             double lengthForLongInterRegionEdge, double weightForInterRegionEdge)
   {
      createInterRegionVisibilityConnections(sourceNode, targetNodeList, sourceObstacleClusters, targetObstacleClusters, filter, edgesToPack,
                                             ONLY_USE_SHORTEST_INTER_CONNECTING_EDGE, lengthForLongInterRegionEdge, weightForInterRegionEdge);
   }

   public static void createInterRegionVisibilityConnections(VisibilityGraphNode sourceNode, List<VisibilityGraphNode> targetNodeList,
                                                             List<Cluster> sourceObstacleClusters, List<Cluster> targetObstacleClusters,
                                                             InterRegionConnectionFilter filter, List<VisibilityGraphEdge> edgesToPack,
                                                             boolean onlyUseShortestEdge, double lengthForLongInterRegionEdge,  double weightForInterRegionEdge)
   {
      PlanarRegion sourceHomeRegion = sourceNode.getVisibilityGraphNavigableRegion().getNavigableRegion().getHomePlanarRegion();
      ConnectionPoint3D sourceInWorld = sourceNode.getPointInWorld();
      Point2DReadOnly sourceInSourceLocal = sourceNode.getPoint2DInLocal();

      RigidBodyTransform transformFromWorldToSource = new RigidBodyTransform();
      if (sourceHomeRegion != null)
      {
         sourceHomeRegion.getTransformToWorld(transformFromWorldToSource);
         transformFromWorldToSource.invert();
      }

      RigidBodyTransform transformFromWorldToTarget = new RigidBodyTransform();

      List<VisibilityGraphEdge> potentialEdges = new ArrayList<>();

      for (VisibilityGraphNode targetNode : targetNodeList)
      {
         ConnectionPoint3D targetInWorld = targetNode.getPointInWorld();
         if (filter.isConnectionValid(sourceInWorld, targetInWorld))
         {
            //TODO: +++++++JerryPratt: xyDistance check is a hack to allow connections through keep out regions enough to make them, but not enough to go through walls...
            double xyDistance = sourceInWorld.distanceXY(targetInWorld);
            if (xyDistance < lengthForLongInterRegionEdge)
            {
               VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
               potentialEdges.add(edge);
            }
            else // Check if the edge is visible if it is a long one.
            {
               PlanarRegion targetHomeRegion = targetNode.getVisibilityGraphNavigableRegion().getNavigableRegion().getHomePlanarRegion();
               targetHomeRegion.getTransformToWorld(transformFromWorldToTarget);
               transformFromWorldToTarget.invert();

               Point2DReadOnly targetInTargetLocal = targetNode.getPoint2DInLocal();

               boolean targetIsVisibleThroughSourceObstacles = true;
               if (sourceHomeRegion != null)
               {
                  Point3D targetProjectedVerticallyOntoSource = PlanarRegionTools.projectInZToPlanarRegion(targetInWorld, sourceHomeRegion);
                  transformFromWorldToSource.transform(targetProjectedVerticallyOntoSource);
                  Point2D targetInSourceLocal = new Point2D(targetProjectedVerticallyOntoSource);
                  targetIsVisibleThroughSourceObstacles = VisibilityTools.isPointVisibleForStaticMaps(sourceObstacleClusters, sourceInSourceLocal,
                                                                                                              targetInSourceLocal);
               }

               Point3D sourceProjectedVerticallyOntoTarget = PlanarRegionTools.projectInZToPlanarRegion(sourceInWorld, targetHomeRegion);

               transformFromWorldToTarget.transform(sourceProjectedVerticallyOntoTarget);

               Point2D sourceInTargetLocal = new Point2D(sourceProjectedVerticallyOntoTarget);

               //TODO: +++JerryPratt: Inter-region connections and obstacles still needs some thought and some good unit tests.
               boolean sourceIsVisibleThroughTargetObstacles = VisibilityTools.isPointVisibleForStaticMaps(targetObstacleClusters, targetInTargetLocal,
                                                                                                           sourceInTargetLocal);

               if ((targetIsVisibleThroughSourceObstacles && sourceIsVisibleThroughTargetObstacles))
               {
                  VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
                  edge.setEdgeWeight(weightForInterRegionEdge);
                  potentialEdges.add(edge);
               }
            }
         }
      }

      if (onlyUseShortestEdge)
      {
         VisibilityGraphEdge shortestEdgeXY = findShortestEdgeXY(sourceNode, potentialEdges);

         if (shortestEdgeXY != null)
         {
            shortestEdgeXY.registerEdgeWithNodes();
            edgesToPack.add(shortestEdgeXY);
         }
      }
      else
      {
         for (VisibilityGraphEdge edge : potentialEdges)
         {
            edge.registerEdgeWithNodes();
         }
         edgesToPack.addAll(potentialEdges);
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

      for (VisibilityGraphNode targetNode : allNavigableNodes)
      {
         if (!filter.isConnectionValid(sourceNode.getPointInWorld(), targetNode.getPointInWorld()))
            continue;

         ConnectionPoint3D targetInWorld = targetNode.getPointInWorld();
         double xyDistance = sourceInWorld.distanceXY(targetInWorld);
         if (xyDistance < lengthForLongInterRegionEdge)
         {
            VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
            potentialEdges.add(edge);
         }
         else // Check if the edge is visible if it is a long one.
         {
//            PlanarRegion sourceHomeRegion = sourceNode.getVisibilityGraphNavigableRegion().getNavigableRegion().getHomePlanarRegion();
            PlanarRegion targetHomeRegion = targetNode.getVisibilityGraphNavigableRegion().getNavigableRegion().getHomePlanarRegion();

            Point2DReadOnly targetInTargetLocal = targetNode.getPoint2DInLocal();
//            Point2DReadOnly sourceInSourceLocal = sourceNode.getPoint2DInLocal();

            Point3D sourceProjectedVerticallyOntoTarget = PlanarRegionTools.projectInZToPlanarRegion(sourceInWorld, targetHomeRegion);
//            Point3D targetProjectedVerticallyOntoSource = PlanarRegionTools.projectInZToPlanarRegion(targetInWorld, targetHomeRegion);

            targetHomeRegion.transformFromWorldToLocal(sourceProjectedVerticallyOntoTarget);
//            sourceHomeRegion.transformFromWorldToLocal(targetProjectedVerticallyOntoSource);


               //TODO: +++JerryPratt: Inter-region connections and obstacles still needs some thought and some good unit tests.
            boolean sourceIsVisibleThroughTargetObstacles = VisibilityTools.isPointVisibleForStaticMaps(targetObstacleClusters, targetInTargetLocal,
                                                                                                        new Point2D(sourceProjectedVerticallyOntoTarget));

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
                                                                                cluster.getNavigableExtrusionsInLocal(), cluster.isClosed());

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
            shortestEdgeXY.registerEdgeWithNodes();
            shortestEdgeXY.setEdgeWeight(edgeWeight);
            edgesToPack.add(shortestEdgeXY);
         }
      }
      else
      {
         for (VisibilityGraphEdge edge : potentialEdges)
         {
            edge.registerEdgeWithNodes();
            edge.setEdgeWeight(edgeWeight);
         }
         edgesToPack.addAll(potentialEdges);
      }

      edgesToPack.addAll(potentialEdges);
   }

   private static VisibilityGraphEdge findShortestEdgeXY(VisibilityGraphNode sourceNode, List<VisibilityGraphEdge> potentialEdges)
   {
      VisibilityGraphEdge shortestEdgeXY = null;
      double shortestLengthXY = Double.POSITIVE_INFINITY;

      for (VisibilityGraphEdge potentialEdge : potentialEdges)
      {
         VisibilityGraphNode targetNode = potentialEdge.getTargetNode();
         double distance = sourceNode.distanceXY(targetNode);

         if (distance < shortestLengthXY)
         {
            shortestEdgeXY = potentialEdge;
            shortestLengthXY = distance;
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
         List<VisibilityGraphEdge> allEdges = visibilityGraphNavigableRegion.getAllEdges();

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

   private List<Connection> createConnectionsFromEdges(List<VisibilityGraphEdge> edges)
   {
      List<Connection> connections = new ArrayList<>();

      for (VisibilityGraphEdge edge : edges)
      {
         ConnectionPoint3D sourcePoint = edge.getSourcePointInWorld();
         ConnectionPoint3D targetPoint = edge.getTargetPointInWorld();
         connections.add(new Connection(sourcePoint, targetPoint));
      }
      return connections;
   }

}
