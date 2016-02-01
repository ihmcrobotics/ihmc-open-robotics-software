package us.ihmc.robotics.hierarchicalKinematics;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.dataStructures.Vector64F;

import javax.vecmath.Vector3d;

public class HierarchicalTaskCOM extends HierarchicalTask {

   private Vector64F  goal;
   private Vector64F  goal_bracket;

   final private Vector64F COM = new Vector64F(3);
   final private Vector3d tempVect3d = new Vector3d();
   final Vector64F tempErr = new Vector64F( 3 );
   
   public HierarchicalTaskCOM(String name, ForwardKinematicSolver _solver) {
      super(name, _solver, 3);

      goal = new Vector64F( 3 );
      goal_bracket = new Vector64F( 3 );

      // weight_matrix_T is an identity matrix. disable the Z
      weight_matrix_T.set(2,2, 0);

   }
   
   public void setTargetBracket(Vector64F bracket) 
   {
      goal_bracket.set(bracket);
      for (int i=0; i<3; i++)
      {
         if( goal_bracket.get(i) < 0 ) 
            goal_bracket.set(i, -goal_bracket.get(i));
      }
   }

   @Override
   public void setTarget(Vector64F _goal) {
      goal.set(_goal);
   }

   @Override
   public Vector64F getTarget() {
      return goal;
   }

   @Override
   protected Vector64F getErrorImpl() 
   {
      CommonOps.subtract( getTarget() , getCurrent(), tempErr );
      for (int i=0; i<3; i++)
      {
         double err_i = tempErr.get(i);
         double err_b = goal_bracket.get(i);
         if( err_i > -err_b &&  err_i < err_b) 
         {
            tempErr.set(i, 0);
         }
      }
      return tempErr;
   }
   @Override
   public DenseMatrix64F computeJacobian()
   {
      forward_kinematic_solver.computeJacobianCOM( Jacobian );
      return Jacobian;
   }

   
   @Override
   public Vector64F getCurrent()
   {
      forward_kinematic_solver.getCOM(tempVect3d) ; 
      
      COM.set(0, tempVect3d.x);
      COM.set(1, tempVect3d.y);
      COM.set(2, tempVect3d.z);
      
      return COM;
   }

}



