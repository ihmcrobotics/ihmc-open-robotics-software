package us.ihmc.perception.mapping;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMTools;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMerger;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class PlanarRegionGraph
{
   private final PlanarRegionNode root = new PlanarRegionNode(null);

   public void addEdge(PlanarRegion parent, PlanarRegion child)
   {
      if (parent.getRegionId() == child.getRegionId())
         throw new RuntimeException("Can't add the child region with the same Id as the parent.");

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
         makeRoot(childNode);
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



   private static void makeRoot(PlanarRegionNode node)
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

      if (parentNode != null)
         parentNode.removeChildNode(node);
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
         return parentNode == null;
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
         childNode.setParentNode(this);
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
         for (int i = 0; i < childNodes.size(); i++)
            childNodes.get(i).recursivelyMergeInChildren(updateTowardsChildAlpha);

         if (planarRegion == null)
            return;

         int childIdx = 0;
         boolean changed = false;
         do
         {
            changed = false;
            while (childIdx < childNodes.size())
            {
               PlanarRegionNode childNode = childNodes.get(childIdx);
               if (PlanarRegionSLAMTools.mergeRegionIntoParent(planarRegion, childNode.planarRegion, updateTowardsChildAlpha))
               {
                  changed = true;
                  childNodes.remove(childIdx);
                  // inherit the remaining children of the child
                  for (int i = 0; i < childNode.getNumberOfChildren(); i++)
                     addChildNode(childNode.getChildNode(childIdx));
               }
               else
               {
                  childIdx++;
               }
            }
         }
         while (changed);
      }

      public PlanarRegionsList collectAsPlanarRegionsList()
      {
         PlanarRegionsList list = new PlanarRegionsList();

         if (planarRegion != null)
            list.addPlanarRegion(planarRegion);

         for (int i = 0; i < childNodes.size(); i++)
            list.addPlanarRegionsList(childNodes.get(i).collectAsPlanarRegionsList());

         return list;
      }
   }
}
