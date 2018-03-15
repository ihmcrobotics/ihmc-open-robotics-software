package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class CentroidalMotionPlanner
{
   /**
    * The reference frame in which the planner operates. All submodules are frame agnostic, 
    * so all quantities must be converted to this frame before running the planner
    */
   private final ReferenceFrame plannerFrame = ReferenceFrame.getWorldFrame();
   private final double robotMass;
   private final double Ixx;
   private final double Iyy;
   private final double Izz;
   private final double deltaTMin;

   private final RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = new RecycledLinkedListBuilder<>(CentroidalMotionNode.class);
   private final OptimizationControlModuleHelper helper;
   private final CentroidalZAxisOptimizationControlModule heightControlModule;

   public CentroidalMotionPlanner(double robotMass, double nominalIxx, double nominalIyy, double nominalIzz, double deltaTMin)
   {
      this.robotMass = robotMass;
      this.Ixx = nominalIxx;
      this.Iyy = nominalIyy;
      this.Izz = nominalIzz;
      this.deltaTMin = deltaTMin;
      this.helper = new OptimizationControlModuleHelper(0.0, 0.0, -9.81, robotMass);
      heightControlModule = new CentroidalZAxisOptimizationControlModule(robotMass, helper);
   }

   /**
    * @param dataNode
    */
   public boolean sumbitNode(CentroidalMotionNode nodeToAdd)
   {
      double nodeTime = nodeToAdd.getTime();
      if (!Double.isFinite(nodeTime))
         return false;

      if (nodeList.getSize() == 0)
         nodeList.getOrCreateFirstEntry().element.set(nodeToAdd);
      else if (nodeList.getFirstEntry().element.getTime() > nodeTime)
         nodeList.insertAtBeginning().element.set(nodeToAdd);
      else if (nodeList.getLastEntry().element.getTime() < nodeTime)
         nodeList.insertAtEnd().element.set(nodeToAdd);
      else
      {
         RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> existingNode = getFirstNodeAfter(nodeTime);
         nodeList.insertBefore(existingNode).element.set(nodeToAdd);
      }
      return true;
   }

   public RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> getFirstNodeAfter(double time)
   {
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> node = nodeList.getFirstEntry();
      while (node != null)
      {
         if (node.element.getTime() > time)
            break;
         node = node.getNext();
      }
      return node;
   }

   public void compute()
   {
      processNodes();
      heightControlModule.submitNodeList(nodeList);
   }

   private void processNodes()
   {
      // Add any pre-processing on the nodes here. Typically should be clipping invalid inputs and merging nodes within an epsilon
      mergeNodesWithinEpsilon(deltaTMin);
   }

   private void mergeNodesWithinEpsilon(double timeEpsilon)
   {
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> node = nodeList.getFirstEntry();
      double nodeTime = node.element.getTime();
      while(node != null)
      {
         RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> candidateNode = node.getNext();
         if(candidateNode == null)
            break;
         if(nodeTime + timeEpsilon > candidateNode.element.getTime())
         {
            mergeNodes(node.element, candidateNode.element);
            nodeList.remove(candidateNode);
         }
         else
            node = node.getNext();
      }
   }
   
   public void mergeNodes(CentroidalMotionNode nodeToKeep, CentroidalMotionNode nodeToDiscard)
   {
      // TODO implement this
   }

   public RecycledLinkedListBuilder<CentroidalMotionNode> getNodeList()
   {
      return nodeList;
   }
}