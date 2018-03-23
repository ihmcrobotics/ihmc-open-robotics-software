package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;
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

   private static final Axis x = Axis.X;
   private static final Axis y = Axis.Y;
   private static final Axis z = Axis.Z;

   private final RecycledLinkedListBuilder<CentroidalMotionNode> nodeList = new RecycledLinkedListBuilder<>(CentroidalMotionNode.class);
   private final RecycledLinkedListBuilder<CentroidalMotionSupportPolygon> supportPolygonList = new RecycledLinkedListBuilder<>(CentroidalMotionSupportPolygon.class);
   private final LinearControlModuleHelper linearHelper;
   private final AngularControlModuleHelper angularHelper;
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
      this.linearHelper = new LinearControlModuleHelper(parameters);
      this.angularHelper = new AngularControlModuleHelper(parameters);
      this.heightControlModule = new CentroidalZAxisOptimizationControlModule(linearHelper, parameters);
      this.transversePlaneControlModule = new CentroidalXYAxisOptimizationControlModule(linearHelper, angularHelper, parameters);
      this.forceTrajectory = new ForceTrajectory(100, LinearControlModuleHelper.forceCoefficients);

      this.tempTrajectory = new FrameTrajectory3D(LinearControlModuleHelper.forceCoefficients, plannerFrame);
      reset();
   }

   public void reset()
   {
      nodeList.clear();
      linearHelper.reset();
      angularHelper.reset();
      heightControlModule.reset();
      transversePlaneControlModule.reset();
      forceTrajectory.reset();
   }

   public boolean submitSupportPolygon(CentroidalMotionSupportPolygon supportPolygonToAdd)
   {
      if (supportPolygonToAdd.containsNaN())
         return false;
      else if (supportPolygonToAdd.getStartTime() >= supportPolygonToAdd.getEndTime())
         return false;

      // Node is confirmed to be a valid node
      double startTime = supportPolygonToAdd.getStartTime();
      double endTime = supportPolygonToAdd.getEndTime();

      if (supportPolygonList.getSize() == 0)
         supportPolygonList.getOrCreateFirstEntry().element.set(supportPolygonToAdd);
      else if (supportPolygonList.getFirstEntry().element.getStartTime() >= endTime)
         supportPolygonList.insertAtBeginning().element.set(supportPolygonToAdd);
      else if (supportPolygonList.getLastEntry().element.getEndTime() <= startTime)
         supportPolygonList.insertAtEnd().element.set(supportPolygonToAdd);
      else
      {
         RecycledLinkedListBuilder<CentroidalMotionSupportPolygon>.RecycledLinkedListEntry<CentroidalMotionSupportPolygon> previousPolygon = getPolygonBefore(startTime, endTime);
         if(previousPolygon == null)
            return false;
         else 
            supportPolygonList.insertAfter(previousPolygon).element.set(supportPolygonToAdd);
      }
      return true;
   }
   
   private RecycledLinkedListBuilder<CentroidalMotionSupportPolygon>.RecycledLinkedListEntry<CentroidalMotionSupportPolygon> getPolygonBefore(double startTime, double endTime)
   {
      RecycledLinkedListBuilder<CentroidalMotionSupportPolygon>.RecycledLinkedListEntry<CentroidalMotionSupportPolygon> supportPolygon = supportPolygonList.getFirstEntry();
      for (int i = 0; i < supportPolygonList.getSize(); i++)
      {
         if (supportPolygon.element.getEndTime() <= startTime)
         {
            if(supportPolygon.getNext() == null)
               return supportPolygon;
            else if (supportPolygon.getNext().element.getStartTime() >= endTime)
               return supportPolygon;
            else  // There is an overlap between the times and the existing polygons
               return null;
         }
         supportPolygon = supportPolygon.getNext();
      }
      return null;
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
      linearHelper.processNodeList(nodeList);
      heightControlModule.compute();
      DenseMatrix64F zForceValues = linearHelper.getOptimizedForceValues(z);
      DenseMatrix64F zForceRateValues = linearHelper.getOptimizedForceRateValues(z);
      DenseMatrix64F zVelocityValues = linearHelper.getOptimizedVelocityValues(z);
      DenseMatrix64F zPositionValues = linearHelper.getOptimizedPositionValues(z);
      DenseMatrix64F deltaTMatrix = linearHelper.getDeltaTMatrix();
      angularHelper.setZForceValues(zForceValues, zForceRateValues, zVelocityValues, zPositionValues, deltaTMatrix);
      DenseMatrix64F yPositionCoefficientMatrix = linearHelper.getPositionCoefficientMatrix(y);
      DenseMatrix64F yPositionBiasMatrix = linearHelper.getPositionBiasMatrix(y);
      DenseMatrix64F yVelocityCoefficientMatrix = linearHelper.getVelocityCoefficientMatrix(y);
      DenseMatrix64F yVelocityBiasMatrix = linearHelper.getVelocityBiasMatrix(y);
      DenseMatrix64F yForceCoefficientMatrix = linearHelper.getForceCoefficientMatrix(y);
      DenseMatrix64F yForceBiasMatrix = linearHelper.getForceBiasMatrix(y);
      DenseMatrix64F yForceRateCoefficientMatrix = linearHelper.getForceRateCoefficientMatrix(y);
      DenseMatrix64F yForceRateBiasMatrix = linearHelper.getForceRateBiasMatrix(y);
      angularHelper.computeXTorqueCoefficientsInTermsOfYDecisionVariables(yPositionCoefficientMatrix, yPositionBiasMatrix, yVelocityCoefficientMatrix,
                                                                          yVelocityBiasMatrix, yForceCoefficientMatrix, yForceBiasMatrix,
                                                                          yForceRateCoefficientMatrix, yForceRateBiasMatrix, deltaTMatrix);
      DenseMatrix64F xPositionCoefficientMatrix = linearHelper.getPositionCoefficientMatrix(x);
      DenseMatrix64F xPositionBiasMatrix = linearHelper.getPositionBiasMatrix(x);
      DenseMatrix64F xVelocityCoefficientMatrix = linearHelper.getVelocityCoefficientMatrix(x);
      DenseMatrix64F xVelocityBiasMatrix = linearHelper.getVelocityBiasMatrix(x);
      DenseMatrix64F xForceCoefficientMatrix = linearHelper.getForceCoefficientMatrix(x);
      DenseMatrix64F xForceBiasMatrix = linearHelper.getForceBiasMatrix(x);
      DenseMatrix64F xForceRateCoefficientMatrix = linearHelper.getForceRateCoefficientMatrix(x);
      DenseMatrix64F xForceRateBiasMatrix = linearHelper.getForceRateBiasMatrix(x);
      angularHelper.computeYTorqueCoefficientsInTermsOfXDecisionVariables(xPositionCoefficientMatrix, xPositionBiasMatrix, xVelocityCoefficientMatrix,
                                                                          xVelocityBiasMatrix, xForceCoefficientMatrix, xForceBiasMatrix,
                                                                          xForceRateCoefficientMatrix, xForceRateBiasMatrix, deltaTMatrix);
      angularHelper.computeCoPPointConstraints(nodeList, supportPolygonList);
      transversePlaneControlModule.compute();
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
      DenseMatrix64F[] forceValues = linearHelper.getOptimizedForceValues();
      DenseMatrix64F[] forceRateValues = linearHelper.getOptimizedForceRateValues();
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