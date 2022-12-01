package us.ihmc.perception.mapping;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMerger;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class PlanarRegionGraph
{
   private final PlanarRegionNode root = new PlanarRegionNode(null);

   public void addEdge(PlanarRegion parent, PlanarRegion child)
   {
      boolean containsParent = root.recursivelyContains(parent.getRegionId());
      boolean containsChild = root.recursivelyContains(child.getRegionId());
      if (!containsParent && !containsChild)
      {
         PlanarRegionNode parentNode = new PlanarRegionNode(parent);
         PlanarRegionNode childNode = new PlanarRegionNode(child);
         root.addChildNode(parentNode);
         parentNode.addChildNode(childNode);
      }
      else if (containsParent && containsChild)
      {
         PlanarRegionNode parentNode = root.recursivelyGet(parent.getRegionId());
         PlanarRegionNode childNode = root.recursivelyGet(child.getRegionId());
         makeRootOfBranch(childNode);
         parentNode.addChildNode(childNode);
      }
      else if (containsParent)
      {
         PlanarRegionNode parentNode = root.recursivelyGet(parent.getRegionId());
         parentNode.addChildNode(new PlanarRegionNode(child));
      }
      else
      {
         PlanarRegionNode childNode = root.recursivelyGet(child.getRegionId());
         childNode.addChildNode(new PlanarRegionNode(parent));
      }
   }

   public void addRootOfBranch(PlanarRegion rootOfBranch)
   {
      root.addChildNode(new PlanarRegionNode(rootOfBranch));
   }

   public void collapseGraphByMerging(double updateTowardsChildAlpha)
   {
      root.recursivelyMergeInChildren(updateTowardsChildAlpha);
   }

   public PlanarRegionsList getAsPlanarRegionsList()
   {
      return root.collectAsPlanarRegionsList();
   }


   private static boolean mergeRegionIntoParent(PlanarRegion parentRegion, PlanarRegion childRegion, double updateTowardsChildAlpha)
   {
      childRegion.setRegionId(parentRegion.getRegionId());

      // Update Map Region Normal and Origin
      UnitVector3DReadOnly mapNormal = parentRegion.getNormal();
      Point3DReadOnly mapOrigin = parentRegion.getPoint();

      UnitVector3DReadOnly regionNormal = childRegion.getNormal();
      Point3DReadOnly regionOrigin = childRegion.getPoint();

      Vector3D futureNormal = new Vector3D();
      futureNormal.interpolate(mapNormal, regionNormal, updateTowardsChildAlpha);

      double futureHeightZ = EuclidCoreTools.interpolate(mapOrigin.getZ(), regionOrigin.getZ(), updateTowardsChildAlpha);

      Vector3D normalVector = new Vector3D(mapNormal);
      Vector3D axis = new Vector3D();
      axis.cross(normalVector, futureNormal);
      double angle = normalVector.angle(futureNormal);

      Point3D futureOrigin = new Point3D(mapOrigin.getX(), mapOrigin.getY(), futureHeightZ);
      AxisAngle rotationToFutureRegion = new AxisAngle(axis, angle);
      Vector3D translationToFutureRegion = new Vector3D();
      translationToFutureRegion.sub(futureOrigin, mapOrigin);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.appendOrientation(rotationToFutureRegion);
      transform.appendTranslation(translationToFutureRegion);

      parentRegion.applyTransform(transform);

      // Update Map Region Hull
      //List<Point2D> concaveHullMapRegionVertices = mapRegion.getConcaveHull();
      //List<Point2D> concaveHullRegionVertices = region.getConcaveHull();
      //List<Point2D> mergedConcaveHull = ConcaveHullMerger.mergeConcaveHulls(concaveHullMapRegionVertices, concaveHullRegionVertices, null);
      //ArrayList<ConvexPolygon2D> newPolygonsFromConcaveHull = new ArrayList<>();
      //ConcaveHullDecomposition.recursiveApproximateDecomposition(mergedConcaveHull, 0.001, newPolygonsFromConcaveHull);
      //
      //RigidBodyTransform transformToWorld = new RigidBodyTransform();
      //mapRegion.getTransformToWorld(transformToWorld);
      //
      //PlanarRegion planarRegion = new PlanarRegion(transformToWorld, mergedConcaveHull, newPolygonsFromConcaveHull);
      //planarRegion.setRegionId(mapRegion.getRegionId());
      //
      //mapRegion.set(planarRegion);

      ArrayList<PlanarRegion> mergedRegion = ConcaveHullMerger.mergePlanarRegions(parentRegion, childRegion, 1.0f, null);

      if (mergedRegion != null)
      {
         if (mergedRegion.size() > 0)
         {
            parentRegion.set(mergedRegion.get(0));
            return true;
         }
         else
         {
            return false;
         }
      }

      throw new RuntimeException("Shoot");
   }


   private static void makeRootOfBranch(PlanarRegionNode node)
   {
      PlanarRegionNode parentNode = node.getParentNode();
      node.setParentNode(null);
      while (parentNode != null && !parentNode.isRoot())
      {
         node.addChildNode(parentNode);
         parentNode.removeChildNode(node);
         node = parentNode;
         parentNode = parentNode.getParentNode();
      }
   }

   private static class PlanarRegionNode
   {
      private final PlanarRegion planarRegion;
      private final List<PlanarRegionNode> childNodes = new ArrayList<>();

      private PlanarRegionNode parentNode = null;

      public PlanarRegionNode(PlanarRegion planarRegion)
      {
         this.planarRegion = planarRegion;
      }

      public boolean isRoot()
      {
         return planarRegion == null;
      }

      public int getNodeId()
      {
         if (!isRoot())
            return planarRegion.getRegionId();
         return -1;
      }

      public void addChildNode(PlanarRegionNode childNode)
      {
         childNodes.add(childNode);
         childNode.setParentNode(parentNode);
      }

      public void setParentNode(PlanarRegionNode parentNode)
      {
         this.parentNode = parentNode;
      }

      public int getNumberOfChildren()
      {
         return childNodes.size();
      }

      public PlanarRegionNode getChildNode(int childNodeNumber)
      {
         return childNodes.get(childNodeNumber);
      }

      public void removeChildNode(PlanarRegionNode childNode)
      {
         this.childNodes.remove(childNode);
      }

      public PlanarRegionNode getParentNode()
      {
         return parentNode;
      }

      public boolean recursivelyContains(PlanarRegionNode other)
      {
         return recursivelyContains(other.getNodeId());
      }

      public boolean recursivelyContains(int nodeId)
      {
         if (getNodeId() == nodeId)
            return true;

         for (PlanarRegionNode child : childNodes)
         {
            if (child.recursivelyContains(nodeId))
               return true;
         }

         return false;
      }

      public PlanarRegionNode recursivelyGet(int nodeId)
      {
         if (getNodeId() == nodeId)
            return this;

         for (PlanarRegionNode child : childNodes)
         {
            PlanarRegionNode node = child.recursivelyGet(nodeId);
            if (node != null)
               return node;
         }

         return null;
      }

      public boolean equals(PlanarRegionNode other)
      {
         return getNodeId() == other.getNodeId();
      }

      public void recursivelyMergeInChildren(double updateTowardsChildAlpha)
      {
         childNodes.forEach(node -> node.recursivelyMergeInChildren(updateTowardsChildAlpha));

         if (planarRegion == null)
            return;

         int childIdx = 0;
         while (childIdx < childNodes.size())
         {
            if (mergeRegionIntoParent(planarRegion, childNodes.get(childIdx).planarRegion, updateTowardsChildAlpha))
            {
               childNodes.remove(childIdx);
            }
            else
            {
               childIdx++;
            }
         }
      }

      public PlanarRegionsList collectAsPlanarRegionsList()
      {
         PlanarRegionsList list = new PlanarRegionsList();

         if (planarRegion != null)
            list.addPlanarRegion(planarRegion);

         childNodes.forEach(child -> list.addPlanarRegionsList(child.collectAsPlanarRegionsList()));

         return list;
      }
   }
}
