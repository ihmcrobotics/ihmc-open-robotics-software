package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import java.util.List;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.RecycledLinkedListBuilder.RecycledLinkedListEntry;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;

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
   private final ForceTrajectory forceTrajectory;
   
   private final FrameTrajectory3D tempTrajectory;
   private final FrameVector3D tempVector1 = new FrameVector3D(), tempVector2 = new FrameVector3D();
   private final FrameVector3D tempVector3 = new FrameVector3D(), tempVector4 = new FrameVector3D();

   public CentroidalMotionPlanner(CentroidalMotionPlannerParameters parameters)
   {
      this.robotMass = parameters.getRobotMass();
      this.Ixx = parameters.getNominalIxx();
      this.Iyy = parameters.getNominalIyy();
      this.Izz = parameters.getNominalIzz();
      this.deltaTMin = parameters.getDeltaTMin();
      this.helper = new OptimizationControlModuleHelper(parameters);
      this.heightControlModule = new CentroidalZAxisOptimizationControlModule(robotMass, helper, parameters);
      this.forceTrajectory = new ForceTrajectory(100, OptimizationControlModuleHelper.forceCoefficients);
      
      this.tempTrajectory = new FrameTrajectory3D(OptimizationControlModuleHelper.forceCoefficients, plannerFrame);
      reset();
   }
   
   public void reset()
   {
      nodeList.clear();
      helper.reset();
      heightControlModule.reset();
      forceTrajectory.reset();
   }

   /**
    * @param dataNode
    */
   public boolean submitNode(CentroidalMotionNode nodeToAdd)
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
      heightControlModule.compute();
   }

   private void processNodes()
   {
      // Add any pre-processing on the nodes here. Typically should be clipping invalid inputs and merging nodes within an epsilon
      mergeNodesWithinEpsilon(deltaTMin);
      helper.processNodeList(nodeList);
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
   
   private void mergeNodes(CentroidalMotionNode nodeToKeep, CentroidalMotionNode nodeToDiscard)
   {
      // TODO implement this
   }

   public RecycledLinkedListBuilder<CentroidalMotionNode> getNodeList()
   {
      return nodeList;
   }

   public ForceTrajectory getForceProfile()
   {
      packForceTrajectory();
      return forceTrajectory;
   }
   
   private void packForceTrajectory()
   {
      forceTrajectory.reset();
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry = nodeList.getFirstEntry();
      while(entry.getNext() != null)
      {
         RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> nextEntry = entry.getNext();
         double t0 = entry.element.getTime();
         double tF = nextEntry.element.getTime();
         entry.element.getForce(tempVector1);
         entry.element.getForceRate(tempVector1);
         nextEntry.element.getForce(tempVector2);
         nextEntry.element.getForceRate(tempVector2);
         tempTrajectory.setCubic(t0, tF, tempVector1, tempVector2, tempVector3, tempVector4);
         forceTrajectory.set(tempTrajectory);
      }
   }
}