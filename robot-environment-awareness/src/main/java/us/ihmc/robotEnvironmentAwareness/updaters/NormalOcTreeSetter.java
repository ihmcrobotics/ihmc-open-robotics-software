package us.ihmc.robotEnvironmentAwareness.updaters;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxInterface;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.node.baseImplementation.AbstractOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;

import java.util.*;

public class NormalOcTreeSetter
{
   public static void updateOcTree(NormalOcTree ocTreeToUpdate, NormalOcTree incomingOcTree)
   {
      List<NormalOcTreeNode> allDestinationNodes = new ArrayList<>();

      OcTreeBoundingBoxInterface boundingBox = incomingOcTree.getBoundingBox();
      Iterator<NormalOcTreeNode> destinationIterator = OcTreeIteratorFactory.createLeafBoundingBoxIteratable(ocTreeToUpdate.getRoot(), boundingBox).iterator();
      Iterator<NormalOcTreeNode> sourceIterator = OcTreeIteratorFactory.createLeafBoundingBoxIteratable(incomingOcTree.getRoot(), boundingBox).iterator();

      // get all the already existing nodes
      while (destinationIterator.hasNext())
         allDestinationNodes.add(destinationIterator.next());

      // iterate over the nodes that already exist, and override their values with the incoming ones.
      List<NormalOcTreeNode> incomingNodesToAdd = new ArrayList<>();
      while (sourceIterator.hasNext())
      {
         NormalOcTreeNode incomingNode = sourceIterator.next();
         int destinationIndex = indexOf(allDestinationNodes, incomingNode);

         if (destinationIndex > 0)
         {
            allDestinationNodes.remove(destinationIndex).copyData(incomingNode);
         }
         else
         {
            incomingNodesToAdd.add(incomingNode);
         }
      }

      // remove all the unused nodes, as those no longer exist
      for (NormalOcTreeNode nodeToDelete : allDestinationNodes)
      {
         ocTreeToUpdate.deleteNode(nodeToDelete.getKeyCopy());
      }

      // add all the new nodes
      PointCloud pointCloud = new PointCloud();
      incomingNodesToAdd.forEach(node -> pointCloud.add(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ()));
      Scan scan = new Scan(new Point3D(), pointCloud);
      Set<NormalOcTreeNode> addedNodes = new HashSet<>();
      ocTreeToUpdate.setMaximumInsertRange(0.0);
      ocTreeToUpdate.setMinimumInsertRange(0.0);
      ocTreeToUpdate.insertScan(scan, false, addedNodes, null);
      for (NormalOcTreeNode addedNode : addedNodes)
      {
         int index = indexOf(incomingNodesToAdd, addedNode);
         addedNode.copyData(incomingNodesToAdd.remove(index));
      }
   }

   private static int indexOf(List<NormalOcTreeNode> source, NormalOcTreeNode node)
   {
      int index;
      if (node == null)
      {
         for (index = 0; index < source.size(); ++index)
         {
            if (source.get(index) == null)
            {
               return index;
            }
         }
      }
      else
      {
         for (index = 0; index < source.size(); ++index)
         {
            if (nodeEquals(node, source.get(index)))
            {
               return index;
            }
         }
      }

      return -1;
   }

   private static boolean nodeEquals(AbstractOcTreeNode<NormalOcTreeNode> nodeA, AbstractOcTreeNode<NormalOcTreeNode> nodeB)
   {
      if (nodeA.getKey0() != nodeB.getKey0())
         return false;

      if (nodeA.getKey1() != nodeB.getKey1())
         return false;

      return nodeA.getKey2() == nodeB.getKey2();
   }
}
