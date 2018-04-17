package us.ihmc.robotEnvironmentAwareness.planarRegion;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.geometry.PointMean;
import us.ihmc.robotEnvironmentAwareness.geometry.VectorMean;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class PlanarRegionSegmentationNodeData implements Iterable<NormalOcTreeNode>
{
   private int id = PlanarRegion.NO_REGION_ID;

   private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
   private final VectorMean normal = new VectorMean();
   private final PointMean point = new PointMean();
   private final Vector3D standardDeviationPrincipalValues = new Vector3D();
   private final Vector3D temporaryVector = new Vector3D();

   private final Point3D min = new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private final Point3D max = new Point3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

   private final List<NormalOcTreeNode> nodes = new ArrayList<>();
   private final Set<NormalOcTreeNode> nodeSet = new HashSet<>();

   public PlanarRegionSegmentationNodeData(int id)
   {
      this.id = id;
   }

   public PlanarRegionSegmentationNodeData(int id, Collection<NormalOcTreeNode> nodes)
   {
      this(id);
      addNodes(nodes);
   }

   public boolean addNode(NormalOcTreeNode node)
   {
      boolean isRegionModified = nodeSet.add(node);
      if (isRegionModified)
      {
         updateNormalAndOriginOnly(node);
         nodes.add(node);
         updateBoundingBoxWithNewNode(node);
      }
      return isRegionModified;
   }

   public boolean addNodes(Collection<NormalOcTreeNode> nodes)
   {
      return nodes.stream().filter(this::addNode).findFirst().isPresent();
   }

   public boolean addNodesFromOtherRegion(PlanarRegionSegmentationNodeData other)
   {
      return other.nodeStream().filter(this::addNode).findFirst().isPresent();
   }

   public boolean contains(NormalOcTreeNode node)
   {
      return nodeSet.contains(node);
   }

   public void removeNodesAndUpdate(Collection<NormalOcTreeNode> nodesToRemove)
   {
      boolean containsAtLeastOne = nodesToRemove.parallelStream().filter(nodeSet::contains).findFirst().isPresent();

      if (containsAtLeastOne)
      {
         nodes.removeAll(nodesToRemove);
         recomputeNormalAndOrigin();
         nodesToRemove.stream()
                      .filter(nodeSet::remove)
                      .forEach(this::updateBoundingBoxAfterRemovingNode);
      }
   }

   public void recomputeNormalAndOrigin()
   {
      pca.clear();
      nodes.stream().forEach(node -> pca.addPoint(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ()));
      pca.compute();

      Point3D mean = new Point3D();
      pca.getMean(mean);

      point.clear();
      point.update(mean, getNumberOfNodes());

      Vector3D thirdVector = new Vector3D();
      pca.getThirdVector(thirdVector);
      pca.getStandardDeviation(standardDeviationPrincipalValues);

      if (thirdVector.dot(normal) < 0.0)
         thirdVector.negate();
      normal.clear();
      normal.update(thirdVector, getNumberOfNodes());
   }

   private void updateNormalAndOriginOnly(NormalOcTreeNode node)
   {
      node.getNormal(temporaryVector);
      // TODO Review and possibly improve dealing with normal flips.
      if (getNumberOfNodes() >= 1 && temporaryVector.dot(normal) < 0.0)
         temporaryVector.negate();
      normal.update(temporaryVector);
      point.update(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ());
   }

   public boolean isInsideBoundingBox(NormalOcTreeNode node)
   {
      double nodeX = node.getX();
      if (nodeX < min.getX() || nodeX > max.getX())
         return false;

      double nodeY = node.getY();
      if (nodeY < min.getY() || nodeY > max.getY()) 
         return false;

      double nodeZ = node.getZ();
      if (nodeZ < min.getZ() || nodeZ > max.getZ()) 
         return false;

      return true;
   }

   public double distanceFromBoundingBox(NormalOcTreeNode node)
   {
      return Math.sqrt(distanceSquaredFromBoundingBox(node));
   }

   public double distanceSquaredFromBoundingBox(NormalOcTreeNode node)
   {
      if (isInsideBoundingBox(node))
         return 0.0;

      double nodeX = node.getX();
      double nodeY = node.getY();
      double nodeZ = node.getZ();

      double dx = max(min.getX() - nodeX, 0.0, nodeX - max.getX());
      double dy = max(min.getY() - nodeY, 0.0, nodeY - max.getY());
      double dz = max(min.getZ() - nodeZ, 0.0, nodeZ - max.getZ());
      return dx * dx + dy * dy + dz * dz;
   }

   public double distanceFromOtherRegionBoundingBox(PlanarRegionSegmentationNodeData other)
   {
      return Math.sqrt(distanceSquaredFromOtherRegionBoundingBox(other));
   }

   public double distanceSquaredFromOtherRegionBoundingBox(PlanarRegionSegmentationNodeData other)
   {
      double dx = max(min.getX() - other.max.getX(), 0.0, other.min.getX() - max.getX());
      double dy = max(min.getY() - other.max.getY(), 0.0, other.min.getY() - max.getY());
      double dz = max(min.getZ() - other.max.getZ(), 0.0, other.min.getZ() - max.getZ());
      return dx * dx + dy * dy + dz * dz;
   }

   private static double max(double a, double b, double c)
   {
      if (a > b)
         return a > c ? a : c;
      else
         return b > c ? b : c;
   }

   private void updateBoundingBoxWithNewNode(NormalOcTreeNode newNode)
   {
      double nodeX = newNode.getX();
      if (nodeX < min.getX())
         min.setX(nodeX);
      else if (nodeX > max.getX())
         max.setX(nodeX);

      double nodeY = newNode.getY();
      if (nodeY < min.getY())
         min.setY(nodeY);
      else if (nodeY > max.getY()) 
         max.setY(nodeY);

      double nodeZ = newNode.getZ();
      if (nodeZ < min.getZ())
         min.setZ(nodeZ);
      else if (nodeZ > max.getZ()) 
         max.setZ(nodeZ);
   }

   private void updateBoundingBoxAfterRemovingNode(NormalOcTreeNode removedNode)
   {
      if (nodes.isEmpty())
      {
         min.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
         max.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      }

      double nodeX = removedNode.getX();
      double nodeY = removedNode.getY();
      double nodeZ = removedNode.getZ();
      double epsilon = 1.0e-3;

      if (Math.abs(nodeX - min.getX()) < epsilon)
         min.setX(findMin((node1, node2) -> node1.getX() < node2.getX() ? -1 : 1).getX());
      else if (Math.abs(nodeX - max.getX()) < epsilon)
         max.setX(findMax((node1, node2) -> node1.getX() < node2.getX() ? -1 : 1).getX());

      if (Math.abs(nodeY - min.getY()) < epsilon)
         min.setY(findMin((node1, node2) -> node1.getY() < node2.getY() ? -1 : 1).getY());
      else if (Math.abs(nodeY - max.getY()) < epsilon)
         max.setY(findMax((node1, node2) -> node1.getY() < node2.getY() ? -1 : 1).getY());
      
      if (Math.abs(nodeZ - min.getZ()) < epsilon)
         min.setZ(findMin((node1, node2) -> node1.getZ() < node2.getZ() ? -1 : 1).getZ());
      else if (Math.abs(nodeZ - max.getZ()) < epsilon)
         max.setZ(findMax((node1, node2) -> node1.getZ() < node2.getZ() ? -1 : 1).getZ());
   }

   private NormalOcTreeNode findMin(Comparator<NormalOcTreeNode> nodeComparator)
   {
      return nodes.parallelStream().min(nodeComparator).get();
   }

   private NormalOcTreeNode findMax(Comparator<NormalOcTreeNode> nodeComparator)
   {
      return nodes.parallelStream().max(nodeComparator).get();
   }

   public int getId()
   {
      return id;
   }

   public void getPoint(int index, Point3D pointToPack)
   {
      nodes.get(index).getHitLocation(pointToPack);
   }

   public NormalOcTreeNode getNode(int index)
   {
      return nodes.get(index);
   }

   public double orthogonalDistance(Point3D point)
   {
      temporaryVector.set(point);
      temporaryVector.sub(this.point);
      return temporaryVector.dot(normal);
   }

   public double absoluteOrthogonalDistance(Point3D point)
   {
      return Math.abs(orthogonalDistance(point));
   }

   public double orhtogonalDistance(NormalOcTreeNode node)
   {
      temporaryVector.set(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ());
      temporaryVector.sub(point);
      return temporaryVector.dot(normal);
   }

   public double absoluteOrthogonalDistance(NormalOcTreeNode node)
   {
      return Math.abs(orhtogonalDistance(node));
   }

   public double angle(Vector3D normal)
   {
      return this.normal.angle(normal);
   }

   public double absoluteAngle(Vector3D normal)
   {
      return Math.abs(angle(normal));
   }

   public double dot(Vector3D normal)
   {
      return this.normal.dot(normal);
   }

   public double absoluteDot(Vector3D normal)
   {
      return Math.abs(dot(normal));
   }

   public double dot(PlanarRegionSegmentationNodeData other)
   {
      return dot(other.normal);
   }

   public double absoluteDot(PlanarRegionSegmentationNodeData other)
   {
      return Math.abs(dot(other));
   }

   public double dotWithNodeNormal(NormalOcTreeNode node)
   {
      return normal.getX() * node.getNormalX() + normal.getY() * node.getNormalY() + normal.getZ() * node.getNormalZ();
   }

   public double absoluteDotWithNodeNormal(NormalOcTreeNode node)
   {
      return Math.abs(dotWithNodeNormal(node));
   }

   public Vector3D getNormal()
   {
      return normal;
   }

   public Point3D getOrigin()
   {
      return point;
   }

   public Vector3D getStandardDeviationPrincipalValues()
   {
      return standardDeviationPrincipalValues;
   }

   public boolean isEmpty()
   {
      return nodes.isEmpty();
   }

   public int getNumberOfNodes()
   {
      return nodes.size();
   }

   public Stream<NormalOcTreeNode> nodeStream()
   {
      return nodes.stream();
   }

   public Stream<NormalOcTreeNode> nodeParallelStream()
   {
      return nodes.parallelStream();
   }

   @Override
   public Iterator<NormalOcTreeNode> iterator()
   {
      return nodes.iterator();
   }

   @Override
   public String toString()
   {
      String ret = "Region ID: " + id;
      ret += ", origin: " + point + ", normal: " + normal;
      ret += ", size: " + getNumberOfNodes();
      return ret;
   }
}
