package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.trajectories.Trajectory;

/**
 * Optimized linear motion along the Z axis to obtain a feasible CoM height trajectory
 * This basically involves trading off between force, velocity and position objectives to obtain a desired motion
 * @author Apoorv S
 *
 */
public class CentroidalZAxisOptimizationControlModule
{
   // Planner parameters
   private final double mass;

   // Planner runtime variables
   private final OptimizationControlModuleHelper helper;

   private final DenseMatrix64F solverInput_H;
   private final DenseMatrix64F solverInput_f;
   private final DenseMatrix64F solverInput_lb;
   private final DenseMatrix64F solverInput_ub;
   private final DenseMatrix64F solverInput_Ain;
   private final DenseMatrix64F solverInput_bin;
   private final DenseMatrix64F solverInput_Aeq;
   private final DenseMatrix64F solverInput_beq;
   private int numberOfDecisionVariables;
   private int numberOfObjectives;
   private int numberOfEqualityConstraints;
   private int numberOfInequalityConstraints;
   private int numberOfNodes;
   private DenseMatrix64F deltaT;

   // Variables to store results for runtime
   public final Trajectory heightTrajectory;
   public final Trajectory linearVelocityProfile;
   public final Trajectory forceProfile;

   public Vector3D gravity;

   public CentroidalZAxisOptimizationControlModule(double robotMass, OptimizationControlModuleHelper helper)
   {
      this.mass = robotMass;
      this.helper = helper;
      
      // Initialize the variables to store the optimization results
      heightTrajectory = new Trajectory(OptimizationControlModuleHelper.positionCoefficients);
      linearVelocityProfile = new Trajectory(OptimizationControlModuleHelper.velocityCoefficients);
      forceProfile = new Trajectory(OptimizationControlModuleHelper.forceCoefficients);

      // Initialize the QP matrices
      solverInput_H = new DenseMatrix64F(OptimizationControlModuleHelper.defaultNumberOfNodes * 2, OptimizationControlModuleHelper.defaultNumberOfNodes * 2);
      solverInput_f = new DenseMatrix64F(OptimizationControlModuleHelper.defaultNumberOfNodes * 2, 1);
      solverInput_lb = new DenseMatrix64F(OptimizationControlModuleHelper.defaultNumberOfNodes * 2, 1);
      solverInput_ub = new DenseMatrix64F(OptimizationControlModuleHelper.defaultNumberOfNodes * 2, 1);
      solverInput_Aeq = new DenseMatrix64F(OptimizationControlModuleHelper.defaultNumberOfNodes, OptimizationControlModuleHelper.defaultNumberOfNodes * 2);
      solverInput_beq = new DenseMatrix64F(OptimizationControlModuleHelper.defaultNumberOfNodes, 1);
      solverInput_Ain = new DenseMatrix64F(OptimizationControlModuleHelper.defaultNumberOfNodes, OptimizationControlModuleHelper.defaultNumberOfNodes * 2);
      solverInput_bin = new DenseMatrix64F(OptimizationControlModuleHelper.defaultNumberOfNodes, 1);
      reset();
   }

   public void reset()
   {
      numberOfNodes = 0;
      numberOfDecisionVariables = 0;
      numberOfObjectives = 0;
      numberOfEqualityConstraints = 0;
      numberOfInequalityConstraints = 0;

      resetTrajectories();
      shapeQPMatrices();
   }

   private void resetTrajectories()
   {
      heightTrajectory.reset();
      linearVelocityProfile.reset();
      forceProfile.reset();
   }

   private void shapeQPMatrices()
   {
      solverInput_H.reshape(numberOfDecisionVariables, numberOfDecisionVariables);
      solverInput_f.reshape(numberOfDecisionVariables, 1);
      solverInput_lb.reshape(numberOfDecisionVariables, 1);
      solverInput_ub.reshape(numberOfDecisionVariables, 1);
      solverInput_Aeq.reshape(numberOfEqualityConstraints, numberOfDecisionVariables);
      solverInput_beq.reshape(numberOfEqualityConstraints, 1);
      solverInput_Ain.reshape(numberOfInequalityConstraints, numberOfDecisionVariables);
      solverInput_bin.reshape(numberOfInequalityConstraints, 1);
   }

   public void submitNodeList(RecycledLinkedListBuilder<CentroidalMotionNode> nodeList)
   {
      numberOfNodes = nodeList.getSize();
      if (numberOfNodes < 2)
         throw new RuntimeException("Cannot create trajectories with just one node");
      setDeltaT(nodeList);
      getNumberOfDecisionVariables(nodeList);
   }

   private void setDeltaT(RecycledLinkedListBuilder<CentroidalMotionNode> nodeList)
   {
      deltaT.reshape(numberOfNodes - 1, 1);
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> node = nodeList.getFirstEntry();
      for (int i = 0; node.getNext() != null; i++)
      {
         CentroidalMotionNode nextNode = node.getNext().element;
         CentroidalMotionNode currentNode = node.element;
         deltaT.set(i, 0, nextNode.getTime() - currentNode.getTime());
         node = node.getNext();
      }
   }

   private void getNumberOfDecisionVariables(RecycledLinkedListBuilder<CentroidalMotionNode> nodeList)
   {
      numberOfDecisionVariables = 0;
      RecycledLinkedListBuilder<CentroidalMotionNode>.RecycledLinkedListEntry<CentroidalMotionNode> node = nodeList.getFirstEntry();
      if (node.element.getZForceConstraintType() == EffortVariableConstraintType.OBJECTIVE)
         numberOfDecisionVariables += 2;
      for (int i = 0; node.getNext() != null; i++)
      {
         CentroidalMotionNode nextNode = node.getNext().element;
         if (nextNode.getZForceConstraintType() == EffortVariableConstraintType.OBJECTIVE)
            numberOfDecisionVariables += 2;
         node = node.getNext();
      }
   }

   public Trajectory getForceProfile()
   {
      return forceProfile;
   }

   public Trajectory getHeightTrajectory()
   {
      return heightTrajectory;
   }

   public Trajectory getLinearVelocityProfile()
   {
      return linearVelocityProfile;
   }
}