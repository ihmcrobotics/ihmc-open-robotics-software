package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import org.apache.commons.lang3.mutable.MutableBoolean;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.rules.interfaces.IteratorSelectionRule;
import us.ihmc.jOctoMap.tools.OcTreeNearestNeighborTools;
import us.ihmc.jOctoMap.tools.OcTreeNearestNeighborTools.NeighborActionRule;
import us.ihmc.robotEnvironmentAwareness.exception.PlanarRegionSegmentationException;
import us.ihmc.robotics.geometry.PlanarRegion;

public class PlanarRegionSegmentationCalculator
{
   private final Random random = new Random(234324L);

   private final Set<NormalOcTreeNode> allRegionNodes = new HashSet<>();
   private List<PlanarRegionSegmentationNodeData> regionsNodeData = new ArrayList<>();
   private final List<NormalOcTreeNode> nodesWithoutRegion = new ArrayList<>();

   private PlanarRegionSegmentationParameters parameters;
   private SurfaceNormalFilterParameters surfaceNormalFilterParameters;
   private OcTreeBoundingBoxInterface boundingBox;

   public void compute(NormalOcTreeNode root)
   {
      allRegionNodes.clear();

      regionsNodeData.parallelStream().forEach(region -> removeBadNodesFromRegion(boundingBox, parameters, region));
      regionsNodeData = regionsNodeData.parallelStream().filter(region -> !region.isEmpty()).collect(Collectors.toList());
      regionsNodeData.forEach(region -> region.nodeStream().forEach(allRegionNodes::add));
      regionsNodeData.forEach(region -> growPlanarRegion(root, region, boundingBox, parameters));
      regionsNodeData = regionsNodeData.stream().filter(region -> region.getNumberOfNodes() > parameters.getMinRegionSize()).collect(Collectors.toList());

      Set<NormalOcTreeNode> nodeSet = new HashSet<>();
      nodeSet.clear();
      new OcTreeIterable<>(root, leafInBoundingBoxWithNormalSetRule(boundingBox)).forEach(nodeSet::add);
      nodeSet.removeAll(allRegionNodes);

      nodesWithoutRegion.clear();
      nodesWithoutRegion.addAll(nodeSet);

      regionsNodeData.addAll(searchNewPlanarRegions(root, boundingBox, parameters, random));
      regionsNodeData.parallelStream().forEach(PlanarRegionSegmentationNodeData::recomputeNormalAndOrigin);
      regionsNodeData.parallelStream().forEach(PlanarRegionSegmentationCalculator::flipNormalOfOutliers);
      regionsNodeData = regionsNodeData.parallelStream().filter(region -> !isRegionSparse(region)).collect(Collectors.toList());

      regionsNodeData = mergePlanarRegionsIfPossible(root, regionsNodeData, parameters);
   }

   public boolean isRegionSparse(PlanarRegionSegmentationNodeData region)
   {
      Vector3D standardDeviationPrincipalValues = region.getStandardDeviationPrincipalValues();
      if (standardDeviationPrincipalValues.getZ() > parameters.getMaxStandardDeviation())
         return true;
      double density = region.getNumberOfNodes() / PolygonizerTools.computeEllipsoidVolume(standardDeviationPrincipalValues);
      return density < parameters.getMinVolumicDensity();
   }

   public void removeDeadNodes()
   {
      regionsNodeData.stream().forEach(region -> removeDeadNodesFromRegion(region));
   }

   public List<PlanarRegionSegmentationNodeData> getSegmentationNodeData()
   {
      return regionsNodeData;
   }

   public List<PlanarRegionSegmentationRawData> getSegmentationRawData()
   {
      return regionsNodeData.stream().map(PlanarRegionSegmentationRawData::new).collect(Collectors.toList());
   }

   public void clear()
   {
      regionsNodeData.clear();
   }

   private IteratorSelectionRule<NormalOcTreeNode> leafInBoundingBoxWithNormalSetRule(OcTreeBoundingBoxInterface boundingBox)
   {
      IteratorSelectionRule<NormalOcTreeNode> isNormalSetRule = (node, maxDepth) -> node.isNormalSet();
      return OcTreeIteratorFactory.multipleRule(OcTreeIteratorFactory.leavesInsideBoundingBoxOnly(boundingBox), isNormalSetRule);
   }

   public static void flipNormalOfOutliers(PlanarRegionSegmentationNodeData region)
   {
      Vector3D regionNormal = region.getNormal();
      int numberOfNormalsFlipped = (int) region.nodeParallelStream().filter(node -> isNodeNormalFlipped(node, regionNormal)).count();
      int numberOfNormalsNotFlipped = region.getNumberOfNodes() - numberOfNormalsFlipped;

      // The majority of the nodes are flipped => flip the region normal
      if (numberOfNormalsFlipped > numberOfNormalsNotFlipped)
         regionNormal.negate();

      // Flip the nodes that upside down
      region.nodeParallelStream().filter(node -> isNodeNormalFlipped(node, regionNormal)).forEach(NormalOcTreeNode::negateNormal);
   }

   private static boolean isNodeNormalFlipped(NormalOcTreeNode node, Vector3D referenceNormal)
   {
      return node.getNormalX() * referenceNormal.getX() + node.getNormalY() * referenceNormal.getY() + node.getNormalZ() * referenceNormal.getZ() < 0.0;
   }

   public static List<PlanarRegionSegmentationNodeData> mergePlanarRegionsIfPossible(NormalOcTreeNode root, List<PlanarRegionSegmentationNodeData> inputRegions,
                                                                                     PlanarRegionSegmentationParameters parameters)
   {
      List<PlanarRegionSegmentationNodeData> mergedRegions = new ArrayList<>();
      while (!inputRegions.isEmpty())
      {
         PlanarRegionSegmentationNodeData candidateForMergeOtherRegions = inputRegions.get(0);
         Map<Boolean, List<PlanarRegionSegmentationNodeData>> mergeableAndNonMergeableGroups = inputRegions.subList(1, inputRegions.size()).parallelStream()
                                                                                                           // Group each region according to the result of areRegionsMergeable.
                                                                                                           .collect(Collectors.groupingBy(other -> areRegionsMergeable(root,
                                                                                                                                                                       candidateForMergeOtherRegions,
                                                                                                                                                                       other,
                                                                                                                                                                       parameters)));

         // Merge all the mergeable regions onto the candidate.
         mergeableAndNonMergeableGroups.getOrDefault(true, Collections.emptyList()).forEach(candidateForMergeOtherRegions::addNodesFromOtherRegion);
         // All non mergeable regions, used for the next iteration.
         inputRegions = mergeableAndNonMergeableGroups.getOrDefault(false, Collections.emptyList());
         // We're done with candidate, put it in the output list.
         mergedRegions.add(candidateForMergeOtherRegions);
      }
      return mergedRegions;
   }

   public static boolean areRegionsMergeable(NormalOcTreeNode root, PlanarRegionSegmentationNodeData currentRegion,
                                             PlanarRegionSegmentationNodeData potentialRegionToMerge, PlanarRegionSegmentationParameters parameters)
   {
      if (currentRegion == potentialRegionToMerge)
         throw new PlanarRegionSegmentationException("Problem Houston.");

      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();

      if (currentRegion.absoluteOrthogonalDistance(potentialRegionToMerge.getOrigin()) > maxDistanceFromPlane)
         return false;

      double dotThreshold = Math.cos(parameters.getMaxAngleFromPlane());

      if (currentRegion.absoluteDot(potentialRegionToMerge) < dotThreshold)
         return false;

      double searchRadius = parameters.getSearchRadius();
      double searchRadiusSquared = searchRadius * searchRadius;

      if (currentRegion.distanceSquaredFromOtherRegionBoundingBox(potentialRegionToMerge) > searchRadiusSquared)
         return false;

      PlanarRegionSegmentationNodeData regionToNavigate;
      PlanarRegionSegmentationNodeData otherRegion;

      if (potentialRegionToMerge.getNumberOfNodes() < currentRegion.getNumberOfNodes())
      {
         regionToNavigate = potentialRegionToMerge;
         otherRegion = currentRegion;
      }
      else
      {
         regionToNavigate = currentRegion;
         otherRegion = potentialRegionToMerge;
      }

      return regionToNavigate.nodeStream().filter(node -> otherRegion.distanceFromBoundingBox(node) < searchRadiusSquared)
                             .filter(node -> isNodeInOtherRegionNeighborhood(root, node, otherRegion, searchRadius)).findFirst().isPresent();
   }

   public static boolean isNodeInOtherRegionNeighborhood(NormalOcTreeNode root, NormalOcTreeNode nodeFromOneRegion,
                                                         PlanarRegionSegmentationNodeData otherRegion, double searchRadius)
   {
      MutableBoolean foundNeighborFromOtherRegion = new MutableBoolean(false);

      NeighborActionRule<NormalOcTreeNode> actionRule = new NeighborActionRule<NormalOcTreeNode>()
      {
         @Override
         public void doActionOnNeighbor(NormalOcTreeNode node)
         {
            if (otherRegion.contains(node))
               foundNeighborFromOtherRegion.setTrue();
         }

         @Override
         public boolean earlyAbort()
         {
            return foundNeighborFromOtherRegion.booleanValue();
         }
      };

      OcTreeNearestNeighborTools.findRadiusNeighbors(root, nodeFromOneRegion, searchRadius, actionRule);
      return foundNeighborFromOtherRegion.booleanValue();
   }

   public List<PlanarRegionSegmentationNodeData> searchNewPlanarRegions(NormalOcTreeNode root, OcTreeBoundingBoxInterface boundingBox,
                                                                        PlanarRegionSegmentationParameters parameters, Random random)
   {
      List<PlanarRegionSegmentationNodeData> newRegions = new ArrayList<>();

      float minNormalQuality = (float) parameters.getMinNormalQuality();

      for (NormalOcTreeNode node : nodesWithoutRegion)
      {
         if (node.getNormalAverageDeviation() > minNormalQuality)
            continue;

         int regionId = PlanarRegion.NO_REGION_ID;
         while (regionId == PlanarRegion.NO_REGION_ID)
            regionId = random.nextInt(Integer.MAX_VALUE);
         PlanarRegionSegmentationNodeData region = createNewOcTreeNodePlanarRegion(root, node, regionId, boundingBox, parameters);

         if (region.getNumberOfNodes() > parameters.getMinRegionSize())
            newRegions.add(region);
      }

      return newRegions;
   }

   public PlanarRegionSegmentationNodeData createNewOcTreeNodePlanarRegion(NormalOcTreeNode root, NormalOcTreeNode seedNode, int regionId,
                                                                           OcTreeBoundingBoxInterface boundingBox,
                                                                           PlanarRegionSegmentationParameters parameters)
   {
      PlanarRegionSegmentationNodeData newRegion = new PlanarRegionSegmentationNodeData(regionId);
      newRegion.addNode(seedNode);
      growPlanarRegion(root, newRegion, boundingBox, parameters);
      return newRegion;
   }

   public void growPlanarRegion(NormalOcTreeNode root, PlanarRegionSegmentationNodeData ocTreeNodePlanarRegion, OcTreeBoundingBoxInterface boundingBox,
                                PlanarRegionSegmentationParameters parameters)
   {
      Vector3D cameraPosition = new Vector3D();
      double searchRadius = parameters.getSearchRadius();

      Deque<NormalOcTreeNode> nodesToExplore = new ArrayDeque<>();
      Set<NormalOcTreeNode> newSetToExplore = new HashSet<>();

      NeighborActionRule<NormalOcTreeNode> extendSearchRule = neighborNode -> recordCandidatesForRegion(neighborNode, ocTreeNodePlanarRegion, newSetToExplore,
                                                                                                        boundingBox, parameters);
      if (surfaceNormalFilterParameters.isUseSurfaceNormalFilter())
      {
         ocTreeNodePlanarRegion.nodeStream() // TODO This should be in parallel, but the previous lambda makes threads share data which is no good.
                               .filter(node -> isNodeInBoundingBox(node, boundingBox)
                                     && isNodeSurfaceNormalInBoundary(node, cameraPosition, surfaceNormalFilterParameters.getSurfaceNormalUpperBound(),
                                                                      surfaceNormalFilterParameters.getSurfaceNormalLowerBound()))
                               .forEach(regionNode -> OcTreeNearestNeighborTools.findRadiusNeighbors(root, regionNode, searchRadius, extendSearchRule));
      }
      else
      {
         ocTreeNodePlanarRegion.nodeStream() // TODO This should be in parallel, but the previous lambda makes threads share data which is no good.
                               .filter(node -> isNodeInBoundingBox(node, boundingBox))
                               .forEach(regionNode -> OcTreeNearestNeighborTools.findRadiusNeighbors(root, regionNode, searchRadius, extendSearchRule));
      }
      nodesToExplore.addAll(newSetToExplore);

      while (!nodesToExplore.isEmpty())
      {
         NormalOcTreeNode currentNode = nodesToExplore.poll();
         if (!ocTreeNodePlanarRegion.addNode(currentNode)) // TODO This updates the region normal based on the average of the nodes' normals, can very likely be improved.
            continue;
         allRegionNodes.add(currentNode);
         newSetToExplore.clear();
         OcTreeNearestNeighborTools.findRadiusNeighbors(root, currentNode, searchRadius, extendSearchRule);
         nodesToExplore.addAll(newSetToExplore);
      }
   }

   public void recordCandidatesForRegion(NormalOcTreeNode neighborNode, PlanarRegionSegmentationNodeData region, Set<NormalOcTreeNode> newSetToExplore,
                                         OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters)
   {
      if (allRegionNodes.contains(neighborNode))
         return;
      if (!isNodeInBoundingBox(neighborNode, boundingBox))
         return;
      if (!isNodePartOfRegion(neighborNode, region, parameters.getMaxDistanceFromPlane(), Math.cos(parameters.getMaxAngleFromPlane())))
         return;
      if (!neighborNode.isNormalSet() || !neighborNode.isHitLocationSet())
         return;

      newSetToExplore.add(neighborNode);
   }

   private static void removeBadNodesFromRegion(OcTreeBoundingBoxInterface boundingBox, PlanarRegionSegmentationParameters parameters,
                                                PlanarRegionSegmentationNodeData region)
   {
      List<NormalOcTreeNode> nodesToRemove = region.nodeStream().collect(Collectors.groupingBy(node -> isBadNode(node, region, boundingBox, parameters)))
                                                   .getOrDefault(true, Collections.emptyList());

      region.removeNodesAndUpdate(nodesToRemove);
   }

   private static void removeDeadNodesFromRegion(PlanarRegionSegmentationNodeData region)
   {
      List<NormalOcTreeNode> nodesToRemove = region.nodeStream().collect(Collectors.groupingBy(node -> isNodeDead(node))).getOrDefault(true,
                                                                                                                                       Collections.emptyList());

      region.removeNodesAndUpdate(nodesToRemove);
   }

   private static boolean isNodeInBoundingBox(NormalOcTreeNode node, OcTreeBoundingBoxInterface boundingBox)
   {
      return boundingBox == null || boundingBox.isInBoundingBox(node.getX(), node.getY(), node.getZ());
   }

   private static boolean isBadNode(NormalOcTreeNode node, PlanarRegionSegmentationNodeData region, OcTreeBoundingBoxInterface boundingBox,
                                    PlanarRegionSegmentationParameters parameters)
   {
      if (isNodeDead(node))
         return true;

      double maxDistanceFromPlane = parameters.getMaxDistanceFromPlane();
      double dotThreshold = Math.cos(parameters.getMaxAngleFromPlane());
      return isNodeInBoundingBox(node, boundingBox) && !isNodePartOfRegion(node, region, maxDistanceFromPlane, dotThreshold);
   }

   private static boolean isNodeDead(NormalOcTreeNode node)
   {
      return !node.isNormalSet();
   }

   public static boolean isNodePartOfRegion(NormalOcTreeNode node, PlanarRegionSegmentationNodeData region, double maxDistanceFromPlane, double dotThreshold)
   {
      double absoluteOrthogonalDistance = region.absoluteOrthogonalDistance(node);
      if (absoluteOrthogonalDistance > maxDistanceFromPlane)
         return false;

      double absoluteDot = region.absoluteDotWithNodeNormal(node);
      return absoluteDot > dotThreshold;
   }

   private static boolean isNodeSurfaceNormalInBoundary(NormalOcTreeNode node, Vector3D cameraPosition, double upperBound, double lowerBound)
   {
      Vector3D cameraToNode = new Vector3D(node.getHitLocationCopy());
      cameraToNode.add(-cameraPosition.getX(), -cameraPosition.getY(), -cameraPosition.getZ());
      Vector3D surfaceNormal = node.getNormalCopy();

      cameraToNode.normalize();

      double dotValue = cameraToNode.dot(surfaceNormal);

      boolean isOutOfSight = lowerBound <= dotValue && dotValue < upperBound;
      return !isOutOfSight;
   }

   public void setParameters(PlanarRegionSegmentationParameters parameters)
   {
      this.parameters = parameters;
   }

   public void setSurfaceNormalFilterParameters(SurfaceNormalFilterParameters parameters)
   {
      this.surfaceNormalFilterParameters = parameters;
   }

   public void setBoundingBox(OcTreeBoundingBoxInterface boundingBox)
   {
      this.boundingBox = boundingBox;
   }
}
