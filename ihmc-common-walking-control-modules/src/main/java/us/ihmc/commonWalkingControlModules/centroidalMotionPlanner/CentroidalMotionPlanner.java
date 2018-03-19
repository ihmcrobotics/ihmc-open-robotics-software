package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;

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
   private final CentroidalXYAxisOptimizationControlModule transversePlaneControlModule;
   private final ForceTrajectory forceTrajectory;

   private final FrameTrajectory3D tempTrajectory;
   private final FrameVector3D tempInitialForce = new FrameVector3D(), tempInitialForceRate = new FrameVector3D();
   private final FrameVector3D tempFinalForce = new FrameVector3D(), tempFinalForceRate = new FrameVector3D();

   public CentroidalMotionPlanner(CentroidalMotionPlannerParameters parameters)
   {
      this.robotMass = parameters.getRobotMass();
      this.Ixx = parameters.getNominalIxx();
      this.Iyy = parameters.getNominalIyy();
      this.Izz = parameters.getNominalIzz();
      this.deltaTMin = parameters.getDeltaTMin();
      this.helper = new OptimizationControlModuleHelper(parameters);
      this.heightControlModule = new CentroidalZAxisOptimizationControlModule(helper, parameters);
      this.transversePlaneControlModule = new CentroidalXYAxisOptimizationControlModule(helper, parameters);
      this.forceTrajectory = new ForceTrajectory(100, OptimizationControlModuleHelper.forceCoefficients);

      this.tempTrajectory = new FrameTrajectory3D(OptimizationControlModuleHelper.forceCoefficients, plannerFrame);
      reset();
   }

   public void reset()
   {
      nodeList.clear();
      helper.reset();
      heightControlModule.reset();
      transversePlaneControlModule.reset();
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

   private RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> getFirstNodeAfter(double time)
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
      mergeNodesWithinEpsilon(deltaTMin);
      helper.processNodeList(nodeList);
      heightControlModule.compute();
      transversePlaneControlModule.compute();
      helper.processDecisionVariables();
   }

   private void mergeNodesWithinEpsilon(double timeEpsilon)
   {
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> node = nodeList.getFirstEntry();
      double nodeTime = node.element.getTime();
      while (node != null)
      {
         RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> candidateNode = node.getNext();
         if (candidateNode == null)
            break;
         if (nodeTime + timeEpsilon > candidateNode.element.getTime())
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
      DenseMatrix64F[] forceValues = helper.getOptimizedForceValues();
      DenseMatrix64F[] forceRateValues = helper.getOptimizedForceRateValues();
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry = nodeList.getFirstEntry();
      tempInitialForce.set(plannerFrame, forceValues[0].get(0, 0), forceValues[1].get(0, 0), forceValues[2].get(0, 0));
      tempInitialForceRate.set(plannerFrame, forceRateValues[0].get(0, 0), forceRateValues[1].get(0, 0), forceRateValues[2].get(0, 0));
      for (int index = 1; entry.getNext() != null; index++)
      {
         RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> nextEntry = entry.getNext();
         double t0 = entry.element.getTime();
         double tF = nextEntry.element.getTime();
         tempFinalForce.set(plannerFrame, forceValues[0].get(index, 0), forceValues[1].get(index, 0), forceValues[2].get(index, 0));
         tempFinalForceRate.set(plannerFrame, forceRateValues[0].get(index, 0), forceRateValues[1].get(index, 0), forceRateValues[2].get(index, 0));

         tempTrajectory.setCubic(t0, tF, tempInitialForce, tempInitialForceRate, tempFinalForce, tempFinalForceRate);
         forceTrajectory.set(tempTrajectory);

         tempInitialForce.set(tempFinalForce);
         tempInitialForceRate.set(tempFinalForceRate);
         entry = nextEntry;
      }
   }
}