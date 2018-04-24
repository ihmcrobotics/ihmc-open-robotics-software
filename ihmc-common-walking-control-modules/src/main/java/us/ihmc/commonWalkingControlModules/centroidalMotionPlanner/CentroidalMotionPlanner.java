package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.trajectories.ForceTrajectory;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.trajectories.PositionTrajectory;
import us.ihmc.commonWalkingControlModules.controlModules.flight.WholeBodyMotionPlanner;
import us.ihmc.commons.PrintTools;
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
   private final ControlModuleHelper helper;
   private final CentroidalZAxisOptimizationControlModule heightControlModule;
   private final CentroidalTwistOptimizationControlModule twistControlModule;
   private final CentroidalXYAxisOptimizationControlModule transversePlaneControlModule;
   private final ForceTrajectory forceTrajectory;
   private final ForceTrajectory torqueTrajectory;
   private final PositionTrajectory positionTrajectory;
   private final PositionTrajectory orientationTrajectory;

   private final FrameTrajectory3D tempTrajectory;
   private final FrameVector3D tempInitialValue = new FrameVector3D(), tempInitialValueRate = new FrameVector3D();
   private final FrameVector3D tempFinalValue = new FrameVector3D(), tempFinalValueRate = new FrameVector3D();
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
      this.helper = new ControlModuleHelper(parameters);
      this.heightControlModule = new CentroidalZAxisOptimizationControlModule(helper, parameters, registry);
      this.twistControlModule = new CentroidalTwistOptimizationControlModule(helper, parameters, registry);
      this.transversePlaneControlModule = new CentroidalXYAxisOptimizationControlModule(helper, parameters);
      this.forceTrajectory = new ForceTrajectory(WholeBodyMotionPlanner.maxNumberOfSegments, ControlModuleHelper.forceCoefficients);
      this.positionTrajectory = new PositionTrajectory(WholeBodyMotionPlanner.maxNumberOfSegments, ControlModuleHelper.positionCoefficients);

      this.torqueTrajectory = new ForceTrajectory(WholeBodyMotionPlanner.maxNumberOfSegments, ControlModuleHelper.forceCoefficients);
      this.orientationTrajectory = new PositionTrajectory(WholeBodyMotionPlanner.maxNumberOfSegments, ControlModuleHelper.positionCoefficients);

      this.yoNumberOfNodesSubmitted = new YoInteger("NumberOfNodes", registry);
      this.tempTrajectory = new FrameTrajectory3D(ControlModuleHelper.positionCoefficients, plannerFrame);
      reset();
   }

   public void reset()
   {
      nodeList.clear();
      resetControlModules();
      resetTrajectories();
      yoNumberOfNodesSubmitted.set(0);
   }

   private void resetControlModules()
   {
      helper.reset();
      heightControlModule.reset();
      twistControlModule.reset();
      transversePlaneControlModule.reset();
   }

   private void resetTrajectories()
   {
      forceTrajectory.reset();
      positionTrajectory.reset();
      torqueTrajectory.reset();
      orientationTrajectory.reset();
   }

   /**
    * @param dataNode
    */
   public boolean submitNode(CentroidalMotionNode nodeToAdd)
   {
      double nodeTime = nodeToAdd.getTime();
      if (!Double.isFinite(nodeTime))
         return false;
      PrintTools.debug(nodeToAdd.toStringAngular());
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
      PrintTools.debug("Height Control Module: ");
      heightControlModule.compute();
      PrintTools.debug("Twist Control Module: ");
      twistControlModule.compute();
      PrintTools.debug("Transverse Control Module: ");
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
      tempInitialValue.set(plannerFrame, forceValues[0].get(0, 0), forceValues[1].get(0, 0), forceValues[2].get(0, 0));
      tempInitialValueRate.set(plannerFrame, forceRateValues[0].get(0, 0), forceRateValues[1].get(0, 0), forceRateValues[2].get(0, 0));
      for (int index = 1; entry.getNext() != null; index++)
      {
         RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> nextEntry = entry.getNext();
         double t0 = entry.element.getTime();
         double tF = nextEntry.element.getTime();
         tempFinalValue.set(plannerFrame, forceValues[0].get(index, 0), forceValues[1].get(index, 0), forceValues[2].get(index, 0));
         tempFinalValueRate.set(plannerFrame, forceRateValues[0].get(index, 0), forceRateValues[1].get(index, 0), forceRateValues[2].get(index, 0));

         tempTrajectory.setCubic(t0, tF, tempInitialValue, tempInitialValueRate, tempFinalValue, tempFinalValueRate);
         forceTrajectory.set(tempTrajectory);

         tempInitialValue.set(tempFinalValue);
         tempInitialValueRate.set(tempFinalValueRate);
         entry = nextEntry;
      }
   }

   public PositionTrajectory getPositionTrajectory()
   {
      packPositionTrajectory();
      return positionTrajectory;
   }

   private void packPositionTrajectory()
   {
      positionTrajectory.reset();
      DenseMatrix64F[] forceValues = helper.getOptimizedForceValues();
      DenseMatrix64F[] velocityValues = helper.getOptimizedVelocityValues();
      DenseMatrix64F[] positionValues = helper.getOptimizedPositionValues();
      double robotMassInverse = 1.0 / robotMass;
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry = nodeList.getFirstEntry();
      tempInitialValue.set(plannerFrame, forceValues[0].get(0, 0) * robotMassInverse, forceValues[1].get(0, 0) * robotMassInverse,
                           forceValues[2].get(0, 0) * robotMassInverse);
      tempInitialVelocity.set(plannerFrame, velocityValues[0].get(0, 0), velocityValues[1].get(0, 0), velocityValues[2].get(0, 0));
      tempInitialPosition.set(plannerFrame, positionValues[0].get(0, 0), positionValues[1].get(0, 0), positionValues[2].get(0, 0));
      for (int index = 1; entry.getNext() != null; index++)
      {
         RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> nextEntry = entry.getNext();
         double t0 = entry.element.getTime();
         double tF = nextEntry.element.getTime();
         tempFinalValue.set(plannerFrame, forceValues[0].get(index, 0) * robotMassInverse, forceValues[1].get(index, 0) * robotMassInverse,
                            forceValues[2].get(index, 0) * robotMassInverse);
         tempFinalVelocity.set(plannerFrame, velocityValues[0].get(index, 0), velocityValues[1].get(index, 0), velocityValues[2].get(index, 0));
         tempFinalPosition.set(plannerFrame, positionValues[0].get(index, 0), positionValues[1].get(index, 0), positionValues[2].get(index, 0));

         tempTrajectory.setQuintic(t0, tF, tempInitialPosition, tempInitialVelocity, tempInitialValue, tempFinalPosition, tempFinalVelocity, tempFinalValue);
         positionTrajectory.set(tempTrajectory);

         tempInitialValue.set(tempFinalValue);
         tempInitialVelocity.set(tempFinalVelocity);
         tempInitialPosition.set(tempFinalPosition);
         entry = nextEntry;
      }
   }

   public ForceTrajectory getTorqueProfile()
   {
      packTorqueTrajectory();
      return torqueTrajectory;
   }

   private void packTorqueTrajectory()
   {
      torqueTrajectory.reset();
      DenseMatrix64F torqueValues = helper.getOptimizedYawTorqueValues();
      DenseMatrix64F torqueRateValues = helper.getOptimizedYawTorqueRateValues();

      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry = nodeList.getFirstEntry();
      tempInitialValue.set(plannerFrame, 0.0, 0.0, torqueValues.get(0, 0));
      tempInitialValueRate.set(plannerFrame, 0.0, 0.0, torqueRateValues.get(0, 0));
      for (int index = 1; entry.getNext() != null; index++)
      {
         RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> nextEntry = entry.getNext();
         double t0 = entry.element.getTime();
         double tF = nextEntry.element.getTime();
         tempFinalValue.set(plannerFrame, 0.0, 0.0, torqueValues.get(index, 0));
         tempFinalValueRate.set(plannerFrame, 0.0, 0.0, torqueRateValues.get(index, 0));

         tempTrajectory.setCubic(t0, tF, tempInitialValue, tempInitialValueRate, tempFinalValue, tempFinalValueRate);
         torqueTrajectory.set(tempTrajectory);

         tempInitialValue.set(tempFinalValue);
         tempInitialValueRate.set(tempFinalValueRate);
         entry = nextEntry;
      }
   }

   public PositionTrajectory getOrientationTrajectory()
   {
      packOrientationTrajectory();
      return orientationTrajectory;
   }

   private void packOrientationTrajectory()
   {
      orientationTrajectory.reset();
      DenseMatrix64F torqueValues = helper.getOptimizedYawTorqueValues();
      DenseMatrix64F yawRateValues = helper.getOptimizedYawRateValues();
      DenseMatrix64F yawValues = helper.getOptimizedYawValues();
      double robotZInertiaInverse = 1.0 / Izz;
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> entry = nodeList.getFirstEntry();
      tempInitialValue.set(plannerFrame, 0.0, 0.0, torqueValues.get(0, 0) * robotZInertiaInverse);
      tempInitialVelocity.set(plannerFrame, 0.0, 0.0, yawRateValues.get(0, 0));
      tempInitialPosition.set(plannerFrame, 0.0, 0.0, yawValues.get(0, 0));
      for (int index = 1; entry.getNext() != null; index++)
      {
         RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> nextEntry = entry.getNext();
         double t0 = entry.element.getTime();
         double tF = nextEntry.element.getTime();
         tempFinalValue.set(plannerFrame, 0.0, 0.0, torqueValues.get(index, 0) * robotZInertiaInverse);
         tempFinalVelocity.set(plannerFrame, 0.0, 0.0, yawRateValues.get(index, 0));
         tempFinalPosition.set(plannerFrame, 0.0, 0.0, yawValues.get(index, 0));

         tempTrajectory.setQuintic(t0, tF, tempInitialPosition, tempInitialVelocity, tempInitialValue, tempFinalPosition, tempFinalVelocity, tempFinalValue);
         orientationTrajectory.set(tempTrajectory);

         tempInitialValue.set(tempFinalValue);
         tempInitialVelocity.set(tempFinalVelocity);
         tempInitialPosition.set(tempFinalPosition);
         entry = nextEntry;
      }
   }
}