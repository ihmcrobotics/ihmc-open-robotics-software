package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.trajectories.ForceTrajectory;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.trajectories.PositionTrajectory;
import us.ihmc.commonWalkingControlModules.controlModules.flight.WholeBodyMotionPlanner;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

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
   private final LinearControlModuleHelper helper;
   private final CentroidalZAxisOptimizationControlModule heightControlModule;
   private final CentroidalXYAxisOptimizationControlModule transversePlaneControlModule;
   private final ForceTrajectory forceTrajectory;
   private final PositionTrajectory positionTrajectory;

   private final FrameTrajectory3D tempTrajectory;
   private final FrameVector3D tempInitialForce = new FrameVector3D(), tempInitialForceRate = new FrameVector3D();
   private final FrameVector3D tempFinalForce = new FrameVector3D(), tempFinalForceRate = new FrameVector3D();
   private final FrameVector3D tempInitialVelocity = new FrameVector3D(), tempFinalVelocity = new FrameVector3D();
   private final FramePoint3D tempInitialPosition = new FramePoint3D(), tempFinalPosition = new FramePoint3D();

   private final YoInteger yoNumberOfNodesSubmitted;

   public CentroidalMotionPlanner(CentroidalMotionPlannerParameters parameters, YoVariableRegistry registry)
   {
      this.robotMass = parameters.getRobotMass();
      this.Ixx = parameters.getNominalIxx();
      this.Iyy = parameters.getNominalIyy();
      this.Izz = parameters.getNominalIzz();
      this.deltaTMin = parameters.getDeltaTMin();
      this.helper = new LinearControlModuleHelper(parameters);
      this.heightControlModule = new CentroidalZAxisOptimizationControlModule(helper, parameters);
      this.transversePlaneControlModule = new CentroidalXYAxisOptimizationControlModule(helper, parameters);
      this.forceTrajectory = new ForceTrajectory(WholeBodyMotionPlanner.maxNumberOfSegments, LinearControlModuleHelper.forceCoefficients);
      this.positionTrajectory = new PositionTrajectory(WholeBodyMotionPlanner.maxNumberOfSegments, LinearControlModuleHelper.positionCoefficients);

      this.yoNumberOfNodesSubmitted = new YoInteger(getClass().getSimpleName() + "", registry);
      this.tempTrajectory = new FrameTrajectory3D(LinearControlModuleHelper.forceCoefficients, plannerFrame);
      reset();
   }

   public void reset()
   {
      nodeList.clear();
      helper.reset();
      heightControlModule.reset();
      transversePlaneControlModule.reset();
      forceTrajectory.reset();
      yoNumberOfNodesSubmitted.set(0);
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
      yoNumberOfNodesSubmitted.increment();
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
      if (nodeList.size() == 0)
         return;
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

   public PositionTrajectory getCoMTrajectory()
   {
      packCoMTrajectory();
      return positionTrajectory;
   }

   private void packCoMTrajectory()
   {
      positionTrajectory.reset();
      DenseMatrix64F[] forceValues = helper.getOptimizedForceValues();
      DenseMatrix64F[] velocityValues = helper.getOptimizedForceValues();
      DenseMatrix64F[] positionValues = helper.getOptimizedForceRateValues();
      double robotMassInverse = 1.0 / robotMass;
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry = nodeList.getFirstEntry();
      tempInitialForce.set(plannerFrame, forceValues[0].get(0, 0) * robotMassInverse, forceValues[1].get(0, 0) * robotMassInverse, forceValues[2].get(0, 0) * robotMassInverse);
      tempInitialVelocity.set(plannerFrame, velocityValues[0].get(0, 0), velocityValues[1].get(0, 0), velocityValues[2].get(0, 0));
      tempInitialPosition.set(plannerFrame, positionValues[0].get(0, 0), positionValues[1].get(0, 0), positionValues[2].get(0, 0));
      for (int index = 1; entry.getNext() != null; index++)
      {
         RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> nextEntry = entry.getNext();
         double t0 = entry.element.getTime();
         double tF = nextEntry.element.getTime();
         tempFinalForce.set(plannerFrame, forceValues[0].get(index, 0) * robotMassInverse, forceValues[1].get(index, 0) * robotMassInverse, forceValues[2].get(index, 0) * robotMassInverse);
         tempFinalVelocity.set(plannerFrame, velocityValues[0].get(index, 0), velocityValues[1].get(index, 0), velocityValues[2].get(index, 0));
         tempFinalPosition.set(plannerFrame, positionValues[0].get(index, 0), positionValues[1].get(index, 0), positionValues[2].get(index, 0));

         tempTrajectory.setQuintic(t0, tF, tempInitialPosition, tempInitialVelocity, tempInitialForce, tempFinalPosition, tempFinalVelocity, tempFinalForce);
         positionTrajectory.set(tempTrajectory);

         tempInitialForce.set(tempFinalForce);
         tempInitialVelocity.set(tempFinalVelocity);
         tempInitialPosition.set(tempFinalPosition);
         entry = nextEntry;
      }
      
      
   }
}