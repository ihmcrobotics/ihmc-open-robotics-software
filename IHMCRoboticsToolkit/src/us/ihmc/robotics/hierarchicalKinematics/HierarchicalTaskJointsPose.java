package us.ihmc.robotics.hierarchicalKinematics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.dataStructures.Vector64F;

public class HierarchicalTaskJointsPose extends HierarchicalTask {

   private Vector64F      goal;
   private DenseMatrix64F coupled_pose;
   final private int numJoints;

   public HierarchicalTaskJointsPose(String name, ForwardKinematicSolver _solver) {
      super(name, _solver, _solver.getNumberOfJoints());
      
      this.numJoints = _solver.getNumberOfJoints();
      
      goal = new Vector64F(numJoints);

      coupled_pose = new DenseMatrix64F(numJoints, numJoints );
      coupled_pose.zero();

      Jacobian.zero();
      for (int i=0; i<numJoints; i++)
      {
         Jacobian.set(i,i, 1); // diagonal
      }     
   }

   public void setCoupledJointWeights(DenseMatrix64F coupled)
   {
      if( coupled.getNumRows() != numJoints || coupled.getNumCols() != numJoints)
      {
         throw new IllegalArgumentException("dimension of vector goal is wrong");
      }
      coupled_pose.set(coupled );
   }

   @Override
   public void setTarget(Vector64F _goal) {
      if( goal.getNumRows() != numJoints)
      {
         throw new IllegalArgumentException("dimension of vector goal is wrong");
      }
      goal.set(_goal);
   }
   
  /* @Override
   public void setWeightError(Vector64F errorWeights)
   {
      weights_error = errorWeights;
   }*/

   @Override
   public Vector64F getTarget() {
      return goal;
   }

   @Override
   public Vector64F getCurrent()
   {
      return forward_kinematic_solver.getLastQ();
   }

   @Override
   protected Vector64F getErrorImpl() 
   {
      // goal + ( coupled_pose*fk->getLastQ() ) -  fk->getLastQ();	

      Vector64F err = new Vector64F( getNumJoints() );
      err.zero();

      Vector64F lastQ       = new Vector64F( forward_kinematic_solver.getLastQ() );
      Vector64F coupled_err = new Vector64F( getNumJoints() );

      CommonOps.mult( coupled_pose, lastQ, coupled_err);

      for (int i=0; i < getNumJoints(); i++ )
      {
         err.set(i, goal.get(i) + coupled_err.get(i) - lastQ.get(i) );
      }

      return err;
   }
   @Override
   public DenseMatrix64F computeJacobian()
   {
      return Jacobian;
   }
}



