package us.ihmc.robotics.hierarchicalKinematics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;
import us.ihmc.robotics.dataStructures.Vector64F;

/**
 * The abstract class HierarchicalTask is the base for any task assigned to HierarchicalKinematicSolver.
 * <p>
 * Considering that very often the solve can found infinite solutions, you can influence 
 * the way the solution is calculated using "weights" expressed either in the task space or joint space. 
 */
public abstract class HierarchicalTask {

   protected double  joint_dQ_eps;
   protected double  maximum_task_space_error;
   protected boolean enabled;

   protected String name = new String();

   final protected Vector64F       weights_J;
   final protected DenseMatrix64F  weight_matrix_T;
   final protected DenseMatrix64F  Jacobian;
   final protected DenseMatrix64F  tempJ;
   final protected ForwardKinematicSolver   forward_kinematic_solver;
   protected HierarchicalTask      parentTask;

   final int taskSpaceDimension;
   final private  Vector64F weightedError;

   public int getNumJoints() 
   {
      return forward_kinematic_solver.getNumberOfJoints();
   }

   public int getDimensionsOfTaskSpace()
   {
      return Jacobian.getNumRows();
   }

   public String getName()
   {
      return name;
   }
   
   

   /**
    * @return the forward_kinematic_solver
    */
   public ForwardKinematicSolver getForwardSolver()
   {
      return forward_kinematic_solver;
   }

   /**
    * Instantiates a new hierarchical task.
    *
    * @param _name the name of the task (needed for logging purposes).
    * @param _solver forward kinematic solver that is used to compute jacobians.
    * @param taskSpaceDimension the number of dimension in task space. Pose of example is 6, position is 3.
    */
   public HierarchicalTask(String _name, ForwardKinematicSolver _solver, int taskSpaceDimension)
   {
      this.name = _name;
      this.taskSpaceDimension = taskSpaceDimension;

      joint_dQ_eps = 0.01;
      maximum_task_space_error = Double.MAX_VALUE;
      enabled = true;
      forward_kinematic_solver = _solver; 

      weights_J = new Vector64F( _solver.getNumberOfJoints() );
      weights_J.setOnes();

      weight_matrix_T = new DenseMatrix64F(taskSpaceDimension, taskSpaceDimension);
      CommonOps.setIdentity( weight_matrix_T );

      Jacobian  = new DenseMatrix64F( taskSpaceDimension, this.getNumJoints()  );
      tempJ     = new DenseMatrix64F( taskSpaceDimension, this.getNumJoints()  );
      
      weightedError = new Vector64F( taskSpaceDimension );
   }

   /**
    * Specify the minimum error magnitude to consider the task "done".
    *
    * @param eps_ the new error tolerance.
    */
   public void setErrorTolerance(double eps_) {
      joint_dQ_eps = eps_;
   }

   public double getErrorTolearance() {
      return joint_dQ_eps;
   }

   /**
    * Calculate the error and check that it is less than errorTollerange.
    *
    * @return true, if is error less than tolerance.
    */
   public boolean isErrorLessThanTolerance()
   {
      Vector64F error = getError();
      double    tolerance = getErrorTolearance();
      return (error.norm() < tolerance);
   }

   /**
    * Set the maximum magnitude of the error in task space.
    * <p>
    * Note: it MUST be higher than errotTolerance, otherwise the solver will never converge.
    *
    * @param max_error the new clamping value for task space error.
    */
   public void setClampingValueForTaskSpaceError(double max_error) {
      maximum_task_space_error = max_error;
   }

   public Vector64F getWeightsJointSpace() {
      return weights_J;
   }

   public DenseMatrix64F getWeightMatrixTaskSpace() {
      return weight_matrix_T;
   }


   /**
    * Sets the weights joint space. It is used to influence the behaviour of the solver.
    * <p>
    * The input vector is used to enable or (partially) disable a joint.
    * 1 means "fully enabled" and 0 is "fully disabled". 
    * 
    * @param _weights the new weights joint space
    */
   public void setWeightsJointSpace(Vector64F _weights)  {
      if( _weights.getNumElements() != Jacobian.getNumCols() )
      {
         throw new IllegalArgumentException("vector with wrong size passed");
      }
      
      for (int i = 0; i< _weights.getNumElements(); i++ )
      {
         if (_weights.get(i) < -0.01 ){
            throw new IllegalArgumentException("Weight cant be less than 0.0");
         }
      }
      
      weights_J.set(_weights);
   }

   /**
    * Sets the weights in task space. It is used to influence the behaviour of the solver.
    * <p>
    * For example, suppose that you have a 6D pose task; your taskSpaceWeight vector will have 6 elements,
    * were 1 means "enabled" and 0 is "disabled. 
    * Setting to 0 the three values of the vector that refer to the rotation, will force the solver to
    * control only the position.
    * 
    * @param _weights the new weights joint space
    */
   public void setWeightsTaskSpace(Vector64F _weights) 
   {
      if( _weights.getNumElements() != Jacobian.getNumRows() )
      {
         throw new IllegalArgumentException("vector with wrong size passed");
      }
      weight_matrix_T.zero();
      for (int i = 0; i< _weights.getNumElements(); i++ )
      {
         double val = _weights.get(i);    

         if (_weights.get(i) < -0.01 ){
            throw new IllegalArgumentException("Weight cant be less than 0.0");
         }

         weight_matrix_T.set( i,i,val);
      }
   }

   /**
    * Sets setWeightsTaskSpace(Vector64F ...). Use it if you know what you are doing.
    *
    * @param weight_matrix the new weights task space
    */
   public void setWeightsTaskSpace(DenseMatrix64F weight_matrix) 
   {
      if( weight_matrix.getNumCols() != Jacobian.getNumRows() ||
            weight_matrix.getNumCols() != weight_matrix.getNumRows()  )
      {
         throw new IllegalArgumentException("matrix/vector with wrong size passed");
      }
      weight_matrix_T.set( weight_matrix );
   }

   /**
    * Compute and return the error, i.e. the difference between target and current state of the task.
    * 
    * @return error vector.
    */
   public Vector64F getError() 
   {
      // we are probably applying these weight twice... no big deal if
      // weight_matrix_T contains only 0s and 1s
      
      Vector64F tempError = new Vector64F( getErrorImpl() );    
      CommonOps.mult( weight_matrix_T, tempError, weightedError );  

      if( maximum_task_space_error > 0.0001)
      {
         double err_norm = NormOps.normF(weightedError);
         if( err_norm > maximum_task_space_error)
         {
            CommonOps.scale( maximum_task_space_error/err_norm, weightedError);
         }
      }
      return weightedError;
   }

   /**
    * Quickly Enable or disable a task without modifying its internal state/configuratiom.
    *
    * @param enable true to enable.
    */
   public void setEnabled(boolean enable) {
      enabled = enable;
   }

   public boolean isEnabled() {
      return enabled;
   }

   /**
    * Sets the target, i.e. a specific state in task space.
    *
    * @param _goal the new target.
    */
   abstract public void setTarget(Vector64F _goal);

   abstract public Vector64F getTarget();

   /**
    * Gets the current state of the task. 
    *
    * @return the current state. To be compared with the target one.
    */
   abstract public Vector64F getCurrent();

   /**
    * Each task must implement this method to compute the error, i.e the difference between
    * target task and current one. The error must be consistent with the jacobian returned by 
    * computeJacobian().
    *
    * @return the error impl
    */
   abstract protected Vector64F getErrorImpl();

   /**
    * Compute jacobian. Used to converge to (isErrorLessThanTolerance() == true)
    *
    * @return the jacobian matrix.
    */
   abstract public DenseMatrix64F computeJacobian();


}
