package us.ihmc.robotics.hierarchicalKinematics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;
import us.ihmc.convexOptimization.qpOASES.BooleanType;
import us.ihmc.convexOptimization.qpOASES.MessageHandling;
import us.ihmc.convexOptimization.qpOASES.PrintLevel;
import us.ihmc.convexOptimization.qpOASES.SQProblem;
import us.ihmc.convexOptimization.qpOASES.VisibilityStatus;
import us.ihmc.convexOptimization.qpOASES.qpOASES;
import us.ihmc.convexOptimization.qpOASES.returnValue;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.Vector64F;

import java.util.ArrayList;
import java.util.HashMap;

/*
import us.ihmc.utilities.qpSolver.qpOASES.*;
 */

/**
 * The HierarchicalKinematicSolver find a solution that satisfies a hierarchy of given tasks.
 * <p>
 * Usage: 
 * <p>
 * 1) First you need to use the method addTask() to provide the instances of the tasks that need to be
 *    performed. Remember that the order in which this method is called IS important. Tasks that are added first 
 *    have higher priority.
 * <p>
 * 2) You can switch the priority of two tasks at runtime using the method swapPriorityOfTwoTasks().
 * <p>
 * 3) For each of you tasks, use the method HierarchicalTask.setTarget() to define your goal. You can
 *    also change the weights of the task to tune its behavior. See the class HierarchicalTask for
 *    more details.
 * 4) Call the method solve() to get a new robot configuration. Note that a robot configuration is
 *    completely defined by a set of joint angles.
 * <p>
 * NOTE: when you create a HierarchicalTask, you must pass an instance of ForwardKinematicSolver. 
 * Such instance must be obtained using getForwardSolver().
 */
public class HierarchicalKinematicSolver {

   final private RobotModel model;
   final private ForwardKinematicSolver fk;
   private int     maxiter;
   private double  eps_joints;
   private int     max_constraints_from_hierarchy;
   private int     verbosityLevel = 0;

   final private DenseMatrix64F constraintMatrix  = new DenseMatrix64F(1, 1);
   final private Vector64F      constraintLowerBound = new Vector64F(1);
   final private Vector64F      constraintUpperBound = new Vector64F(1);

   final public  CollisionAvoidanceConstraint  collisionAvoidance;
   final private ArrayList<HierarchicalTask> taskList = new ArrayList<HierarchicalTask>();
   final private ArrayList<Vector64F> deltaQ_list         = new ArrayList<Vector64F>();
   final private us.ihmc.convexOptimization.qpOASES.Options solver_options = new us.ihmc.convexOptimization.qpOASES.Options();

   private Vector64F guessed_dq;

   public HierarchicalKinematicSolver(RobotModel model_, int maxiter_, double eps_joints_) {
      max_constraints_from_hierarchy = 0;
      eps_joints = eps_joints_;
      maxiter    = maxiter_;
      model      = model_;
      fk         = new ForwardKinematicSolver(model);
      guessed_dq = new Vector64F(fk.getNumberOfJoints());
      guessed_dq.zero();

      collisionAvoidance = new CollisionAvoidanceConstraint( fk, model );

      //disable info display
      qpOASES.getGlobalMessageHandler().setInfoVisibilityStatus(  VisibilityStatus.VS_HIDDEN );

      fk.updateKinematics( guessed_dq ); //warmup

      solver_options.setPrintLevel(PrintLevel.PL_NONE);
      solver_options.setEnableRegularisation(BooleanType.BT_TRUE);
      solver_options.setEpsRegularisation(0.1); 
      solver_options.setNumRegularisationSteps(5);
      solver_options.setTerminationTolerance(1e-4);
      solver_options.setBoundTolerance(1e-4);
   }

   public ForwardKinematicSolver getForwardSolver() 
   { 
      return fk;
   }

   public RobotModel getRobotModel() 
   { 
      return model;
   }

   public HashMap<String, Integer> createListOfActiveJoints()
   {
      HashMap<String, Integer> jointlist = new HashMap<String, Integer>();
      for (int i=0; i< model.getNrOfJoints(); i++ )
      {
         jointlist.put( model.getActiveJointName(i), i) ;
      }
      return jointlist;
   }

   /**
    * Swap the priority of two tasks only if needed. 
    * <p>
    * The tasks passed as first argument will have an higher priority than the second one.
    *
    * @param highestPriorityTask the highest priority task.
    * @param lowestPriorityTask  the lowest priority task.
    */
   public void changeRelativePriorityOfTwoTasks(HierarchicalTask highestPriorityTask, HierarchicalTask lowestPriorityTask)
   {
      int index1 = taskList.indexOf( highestPriorityTask );
      int index2 = taskList.indexOf( lowestPriorityTask );

      if( index2 < index1 ) // lowest number = highest priority
      {
         taskList.set( index2, highestPriorityTask );
         taskList.set( index1, lowestPriorityTask );
      }
   }

   /**
    * Adds a task to the solver. The first tasks added have higher priority in the hierarchy.
    * 
    * <p>
    * Important: it will fails if the task doesn't use the proper instance of ForwardKinematicSolver.
    *
    * @param t the task.
    */
   public void addTask(HierarchicalTask t) {

      int N = t.getNumJoints();
      int C = t.getDimensionsOfTaskSpace();

      taskList.add(t);
      deltaQ_list.add(new Vector64F(N));

      max_constraints_from_hierarchy += C; // increase for the next one
   }

   /**
    * Sets the verbosity of the log displayed by the method solve().
    * <p>
    * 0: no info is displayed.
    * 1: a resume of the computation.
    * 2: very verbose output.
    *
    * @param verbosityLevel the new verbosity level.
    */
   public void setVerbosityLevel(int verbosityLevel) 
   {
      if( verbosityLevel < 0 ) verbosityLevel = 0;
      if( verbosityLevel > 2 ) verbosityLevel = 2;
      this.verbosityLevel = verbosityLevel;
   }

   private void blockCopyInto(DenseMatrix64F destination, 
         int offset_row,
         int offset_col, 
         DenseMatrix64F source) 
   {
      if (offset_row < 0 || offset_col < 0
            || (offset_col + source.getNumCols()) > destination.getNumCols()
            || (offset_row + source.getNumRows()) > destination.getNumRows()) 
      {
         throw new IndexOutOfBoundsException();
      }
      for (int c = 0; c < source.getNumCols(); c++) {
         for (int r = 0; r < source.getNumRows(); r++) {
            destination.set(r + offset_row, c + offset_col, source.get(r, c));
         }
      }
   }


   private DenseMatrix64F buildReducedConstraintMatrix(int maxNumberOfConstraints, int numActiveJoints, boolean[] activeJoints )
   {
      //   System.out.println("--------\n" + composite_J);

      int numJoints = activeJoints.length;

      int numActiveConstraints = maxNumberOfConstraints;
      boolean[] activeConstraints = new  boolean[maxNumberOfConstraints];

      for (int row = 0; row < maxNumberOfConstraints; row++ )
      {
         activeConstraints[row] = true;
         for (int col = 0; col < numJoints; col++ )
         {
            if( activeJoints[col] == false && ! MathTools.epsilonEquals( constraintMatrix.get(row,col), 0.0, 0.0001  ))
            {
               activeConstraints[row] = false;
               numActiveConstraints--;
               break;
            }
         }
      }

      DenseMatrix64F A = new DenseMatrix64F( numActiveConstraints, numActiveJoints ); 

      if( numActiveConstraints > 0 )
      {
         int row = 0;
         for (int r = 0; r < maxNumberOfConstraints ; r++ )
         {
            if( activeConstraints[r] )
            {
               int col = 0;
               for( int c=0; c< numJoints; c++ ) 
               {
                  if( activeJoints[c] ){
                     A.set( row,col, constraintMatrix.get(r,c) );
                     col++;
                  }
               }
               row++;
            }
         }
      }
      //  System.out.println("-----\n" + A);
      return A;
   }

   public int solve(Vector64F q_init, Vector64F q_out, boolean continueUntilPostureConverged)  throws Exception 
   {
      boolean verbose = false;
      boolean atLeastOneSolutionsFound = false;

      if( verbosityLevel == 2 ) verbose = true;

      int numJoints = q_init.getNumRows();

      //ultra conservative large size, just in case.
      // note that resize actually happens only the first time this method is executed.
      int max_constraints = max_constraints_from_hierarchy + collisionAvoidance.getNumConstraints();
      constraintMatrix.reshape( max_constraints, numJoints);
      constraintLowerBound.reshape( max_constraints, 1);
      constraintUpperBound.reshape( max_constraints, 1);

      Vector64F q = new Vector64F(q_init);
      q_out.set(q_init);

      Vector64F last_succesfull_qout = new Vector64F(q_init);

      //--------------------------------------------------------------------------
      // Clamp the values of the vector q to avoid a problem that we might have 
      // when q_init is not initialized correctly.
      for (int i = 0; i < q.getNumElements(); i++) 
      {
         double min =  model.q_min(i);
         double max =  model.q_max(i);

         if( min > max)
         {
            throw new RuntimeException("a minimum joint limit is larger than the maximum one. Check the URDF file");
         }

         if (q.get(i) <min)
            q.set(i, min);
         else if (q.get(i) > max)
            q.set(i, max);
      }
      //--------------------------------------------------------------------------
      // Update (only once) the state of the robot.
      // This is needed to compute up to date jacobians (each of the tasks is supposed to have a reference to fk). 
      fk.updateKinematics(q);

      Vector64F jointWeight = new Vector64F( model.getNrOfJoints() );
      jointWeight.setZero();
      //--------------------------------------------------------------------------
      // iterate until a solution is found or a major error is detected.
      for (int iter = 0; iter < maxiter; iter++) 
      {

         // add constraints based on collision detection.
         int total_constraints = collisionAvoidance.getNumConstraints();

         try{
            if( collisionAvoidance.isEnabled())
            {
               // the top of the composite matrix and vector is filled with 
               collisionAvoidance.computeConstraints(constraintMatrix, constraintUpperBound);         

               if (verbose) System.out.print("DISTANCES: ");

               for (int i=0; i<total_constraints; i++ )
               {
                  //very large number as lower limit
                  constraintLowerBound.set(i, Double.NEGATIVE_INFINITY); 
                  if (verbose) System.out.format("%.3f / ", constraintUpperBound.get(i)  );
               }
               if (verbose) System.out.println();
            }
         }
         catch (Exception e){
            e.printStackTrace();
         }

         if (verbose)
            System.out.println("\n--------------------------------\n");

         // The joint weights are copied from each of the tasks.
         // At each step the number of active joints can only increase, it can not decrease.
         jointWeight.setZero();   
         boolean [] activeJoints = new boolean [jointWeight.getNumElements()];

         // Hierarchy loop:
         // Start solving the QP problem for tasks with higher priority.
         // At the end of the iteration, the solution is used to build a set of constraints to be used 
         // by the next task in the list.
         // This means that the next task should not change the results obtained by the previous task.
         for (int t = 0; t < taskList.size(); t++) 
         {
            HierarchicalTask task = taskList.get(t);
            
            if (verbose)
               System.out.println("----------- task " + iter + "/" + t + " ----"+ task.getName() + "-------\n");
            
            int numActiveJoints = updateJointWeightsAndActiveJoints(task, jointWeight, activeJoints);

            Vector64F deltaQ = deltaQ_list.get(t);

            // use the previous result if available
            if( t == 0 )
               deltaQ.zero();
            else
               deltaQ.set( deltaQ_list.get(t-1) );

            // note: disable is all the weights are 0
            if (task.isEnabled() && task.getWeightsJointSpace().norm() > this.eps_joints ) 
            {
               DenseMatrix64F JW = buildWeightedJacobian(task, jointWeight);

               Vector64F err = new Vector64F(task.getError());
               double err_norm = NormOps.normF(err);

               if (verbose) {
                  System.out.println(">> q:   \t" + q);
                  System.out.println("target: " + task.getTarget());
                  System.out.println("current: " + task.getCurrent());
                  System.out.format("err: [%.3f]", err_norm );
                  System.out.println( err );
               } 	

               Vector64F.throwIfContainsNAN( q );
               Vector64F.throwIfContainsNAN( task.getTarget() );
               Vector64F.throwIfContainsNAN( task.getCurrent() );
               Vector64F.throwIfContainsNAN( err );

               boolean is_joint_pose = (task instanceof HierarchicalTaskJointsPose);

               boolean disableTask = false;

               if( !(is_joint_pose || task.isErrorLessThanTolerance() == false || iter == 0  ) )
               {
                  disableTask = true;
               }

               if (disableTask == false) 
               {
                  DenseMatrix64F reducedConstraintMatrix = buildReducedConstraintMatrix( total_constraints, numActiveJoints, activeJoints );

                  returnValue res = returnValue.RET_ERROR_UNDEFINED;

                  //-----------------------------------
                  // To considerably speed up computation we reduce the complexity of the problem:
                  // We remove all the column from JW which are related to a disable joint.
                  // The dimension of the Hessian is reduced from [numJoints, numJoints] to 
                  // [numActiveJoints, numActiveJoints].
                  // Similarly we remove the rows of the contraintMatrix that affect only disable joints,
                  // and column related to disabled joints.

                  SQProblem solver = new SQProblem( numActiveJoints, reducedConstraintMatrix.getNumRows() );
                  solver.setPrintLevel(PrintLevel.PL_NONE);                  
                  solver.setOptions( solver_options );    
                  int nWSR = 100;       


                  DenseMatrix64F reduced_JW  = new DenseMatrix64F( JW.getNumRows(), numActiveJoints );
                  for (int row = 0; row < JW.getNumRows(); row++ )
                  {
                     int col = 0;
                     for( int c=0; c< JW.getNumCols(); c++ ) {
                        if( activeJoints[c] ){
                           reduced_JW.set( row,col, JW.get(row,c) );
                           col++;
                        }
                     }
                  }

                  DenseMatrix64F reducedHessian = new DenseMatrix64F(numActiveJoints, numActiveJoints );
                  CommonOps.multTransA(reduced_JW, reduced_JW, reducedHessian);

                  Vector64F reducedG = new Vector64F( numActiveJoints );
                  CommonOps.multTransA(-1.0, reduced_JW, err, reducedG);

                  Vector64F reducedLowerBound   = new Vector64F(numActiveJoints);
                  Vector64F reducedUpperBound   = new Vector64F(numActiveJoints);

                  // the lower and upper bounds are the distance from joint limits.
                  {
                     int row = 0;
                     for( int r=0; r< numJoints; r++ ) 
                     {
                        if( activeJoints[r] ){
                           reducedLowerBound.set(row, model.q_min().get(r) - q.get(r) );
                           reducedUpperBound.set(row, model.q_max().get(r) - q.get(r) );
                           row++;
                        }
                     }
                  }

                  guessed_dq.setZero();

                  res =  qpOASES.initAndSolve(solver, reducedHessian.data, reducedG.data, 
                        reducedConstraintMatrix.data,
                        reducedLowerBound.data, reducedUpperBound.data, 
                        constraintLowerBound.data, constraintUpperBound.data,
                        nWSR, guessed_dq.data );

                  //-----------------------------------

                  if (res != returnValue.SUCCESSFUL_RETURN)
                  {
                     if(verbosityLevel>0)
                     {
                        System.out.format("\n>>>> iteration %d: task %d failed with code %d:  %s <<<<<\n\n" ,iter, t, res.swigValue(),
                              MessageHandling.getErrorCodeMessage(res) );
                     }
                     return -2;
                  }
                  else { // qpOASES.initAndSolve 
                     
                     Vector64F reducedDeltaQ = new Vector64F( numActiveJoints );
                     solver.getPrimalSolution( reducedDeltaQ.data );

                     // map the reduced solution to the full vector
                     {
                        int row = 0;
                        for( int r=0; r< deltaQ.getNumElements(); r++ ) 
                        {
                           if( activeJoints[r] ){
                              deltaQ.set(r, reducedDeltaQ.get(row) );
                              row++;
                           }
                           else{
                              deltaQ.set(r, 0.0 );
                           }
                        }
                     }

                     if (verbose)
                        System.out.println(">> q:   \t" + q);

                     CommonOps.add(q, deltaQ, q_out);

                     // use this solution as next quessed_q
                     guessed_dq.set( deltaQ );

                     if (verbose){
                        System.out.println(">> del: \t" + deltaQ);
                        System.out.println(">> qout:\t" + q_out);

                        Vector64F delta   = new Vector64F( JW.getNumRows() );
                        Vector64F new_err = new Vector64F( JW.getNumRows() );
                        CommonOps.mult(JW, deltaQ, delta );
                        CommonOps.subtract( err, delta, new_err );
                        System.out.format(">> new_err A: [%.3f] %s\n", new_err.norm(), new_err );
                     }
                  }
               }

               if (verbose) {
                  System.out.println("\nERROR BEFORE");

                  for (int tt = 0; tt<=t; tt++) {
                     HierarchicalTask TT = taskList.get(tt);
                     if (TT.isEnabled() ) {  System.out.format("Error T: [%.4f] %s\n", TT.getError().norm(),  TT.getError() );  }
                  }
               }

               if (verbose) {
                  if( disableTask )
                  {
                     System.out.println("\nERROR AFTER is the SAME: task was disabled");
                  }
                  else { 
                     fk.updateKinematics(q_out);
                     System.out.println("\nERROR AFTER ITERATION");

                     for (int tt = 0; tt<=t; tt++) {
                        HierarchicalTask TT = taskList.get(tt);
                        if (TT.isEnabled() ) {  System.out.format("Error T: [%.4f] %s\n", TT.getError().norm(),  TT.getError() );  }
                     }
                  }
               }

               // append constraints for the NEXT task
               if ( t < taskList.size() - 1)
               {
                  // store the jacobian in composite_J and
                  // J*dQ in composite_lower_b and composite_upper_b
                  blockCopyInto(constraintMatrix, total_constraints, 0, JW);

                  Vector64F Aq = new Vector64F(JW.getNumRows());
                  CommonOps.mult(JW, deltaQ, Aq);

                  blockCopyInto(constraintLowerBound, total_constraints, 0, Aq);
                  blockCopyInto(constraintUpperBound, total_constraints, 0, Aq);

                  total_constraints += task.getDimensionsOfTaskSpace();
               } 
            } // end of enabled
         } // end of for t

         // /----------------------------------
         boolean errorsBelowLimit = true;

         int lastTask = taskList.size()-1;
         boolean hasConverged = ( deltaQ_list.get( lastTask ).norm() < this.eps_joints );

         // compute the forward kinematic here. In this way the next iteration will have an up to date robot state
         fk.updateKinematics(q_out);

         for (int t = 0; t < taskList.size(); t++) {
            HierarchicalTask T = taskList.get(t);
            if (T.isEnabled() )
            {
               if(T.isErrorLessThanTolerance() == false) {
                  errorsBelowLimit = false;
               }
            }
         }

         // boolean hasCollision = collisionAvoidance.hasCollision();
         boolean hasCollision = false;

         if ( !hasCollision && errorsBelowLimit) // acceptable solution
         {
            atLeastOneSolutionsFound = true;
            last_succesfull_qout.set( q_out );

            if( verbosityLevel == 1 ) {
                  printStatus("succeded");
            }
            return iter;
         }

         if( hasConverged && ( hasCollision || !errorsBelowLimit) )
         {
            if( verbosityLevel == 1 ) {
               printStatus("failed soon");
               return -1;
            }
         }

         q.set(q_out);

      } // end of for iter

      if( atLeastOneSolutionsFound  )
      {      
         q_out.set( last_succesfull_qout );

         if( verbosityLevel == 1 ) {
            printStatus("succeded at LAST");
         }
         return maxiter;
      }


      if( verbosityLevel == 1 )  {
         printStatus("failed");
      }
      return -1;
   }

   // the weight used in cascade should be the max of all the previous weights.
   // In other words, don't disable a joint used in a previous task.
   private int updateJointWeightsAndActiveJoints(HierarchicalTask task, Vector64F jointWeight, boolean[] activeJoints)
   {
      int numActiveJoints = 0;

      if( task.isEnabled() )
      {
         for (int j=0; j< jointWeight.getNumRows(); j++ )
         {
            double jval = task.getWeightsJointSpace().get(j);
            if( jval < 0 ) jval = 0;

            if( jval > jointWeight.get(j))
            {
               jointWeight.set(j, jval);
            }
            if( (jointWeight.get(j) > eps_joints ) )
            {
               activeJoints[j] = true;
               numActiveJoints++;
            }
            else{
               activeJoints[j] = false;
            }                 
         }
      }
      return numActiveJoints;
   }

   private DenseMatrix64F buildWeightedJacobian(HierarchicalTask task, Vector64F jointWeight)
   {
      DenseMatrix64F J       = task.computeJacobian() ;    
      DenseMatrix64F JW      = new DenseMatrix64F( J.getNumRows(), J.getNumCols() );
      DenseMatrix64F temp_JW = new DenseMatrix64F( J.getNumRows(), J.getNumCols() );

      for (int c=0; c < J.getNumCols(); c++ )
      {
         for (int r=0; r < J.getNumRows(); r++ )
         {
            double new_value =  J.get(r,c) * jointWeight.get(c);
            temp_JW.set(r,c, new_value);
         }
      }
      CommonOps.mult( task.getWeightMatrixTaskSpace(), temp_JW, JW);
      return JW;
   }

   void printStatus(String message)
   {
      System.out.format("-------- HierarchicalInverseKinematic " + message + " -----------\n");

      for (int t = 0; t < taskList.size(); t++) 
      {
         HierarchicalTask T = taskList.get(t);
         if( T.isEnabled())
         {
            if(T.isErrorLessThanTolerance() == false) 
            {
               System.out.format("[Task %d]: FAILED : %s\n", t, T.getName()); 
            }
            else{
               System.out.format("[Task %d]: PASSED : %s\n", t, T.getName()); 
            }
            System.out.format("\t Target \t\t %s\n", T.getTarget() );
            System.out.format("\t Error  [%.4f]\t %s\n\n", T.getError().norm(),  T.getError() );
         }
         else{
            System.out.format("[Task %d]: DISABLED : %s\n\n", t, T.getName()); 
         }
      }
      if( collisionAvoidance.hasCollision() )
      {
         System.out.format("FAILURE:  Model collides :( \n\n" ); 
      }
      System.out.format("--------------------------------------------------------------\n\n");
   }

   public int getNumberOfJoints()
   {
      return model.getNrOfJoints();
   }

}
