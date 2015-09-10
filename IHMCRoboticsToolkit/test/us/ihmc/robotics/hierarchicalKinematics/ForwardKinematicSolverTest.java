package us.ihmc.robotics.hierarchicalKinematics;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.Vector64F;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import static org.junit.Assert.assertTrue;

public class ForwardKinematicSolverTest
{
   private RobotModel model;
   private ForwardKinematicSolver forward_solver;
   
   private DenseMatrix64F jacobianEndEffector;
   private DenseMatrix64F jacobianWristPlusOffset;
   private DenseMatrix64F jacobianManualCalculation;
   final private boolean doprint = true;
   

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test() 
   {
      HikSupport.loadLibrary();
      model =  new RobotModel();
      model.loadURDF(getClass().getResource("test_collision.urdf").getPath(), false);
      forward_solver = new ForwardKinematicSolver(model);
      
      Vector64F Q = new Vector64F( model.getNrOfJoints() );
      Q.setZero();
      forward_solver.updateKinematics( Q );
      
      int larm =  model.getBodyId( "r_larm" );
      int hand =  model.getBodyId( "r_hand" );
      
      Vector3d offset =  new Vector3d();
      offset.set(0,0,0);
      
      jacobianEndEffector       = new DenseMatrix64F(6, model.getNrOfJoints() );
      jacobianWristPlusOffset   = new DenseMatrix64F(6, model.getNrOfJoints() );
      jacobianManualCalculation = new DenseMatrix64F(3, model.getNrOfJoints() );
      
      forward_solver.computeJacobian(hand, offset, jacobianEndEffector);   
      System.out.println( jacobianEndEffector  );
      
      offset.setY( -0.31 );   
      forward_solver.computeJacobian(larm, offset, jacobianWristPlusOffset);
      System.out.println( ( jacobianWristPlusOffset ) );
      
      
      Quat4d quat = new Quat4d();
      Vector3d posA  = new Vector3d();
      forward_solver.getBodyPose(hand, quat, posA);
      
      double EPS = 0.0002;
      // obtain the upper part of the jacobian using finite differentiation.
      // i.e. small deltas to calculate each of the columns of the matrix
      for(int j=0; j<model.getNrOfJoints(); j++ )
      {
         Q.setZero();
         Q.set(j, EPS);
         
         Vector3d posB  = new Vector3d();
         forward_solver.updateKinematics( Q );
         forward_solver.getBodyPose(hand, quat, posB);
         
         jacobianManualCalculation.set(0,j, (posB.x - posA.x)/EPS);
         jacobianManualCalculation.set(1,j, (posB.y - posA.y)/EPS);
         jacobianManualCalculation.set(2,j, (posB.z - posA.z)/EPS);
      }
      
      System.out.println( ( jacobianManualCalculation ) );
      
      for (int r=0; r< jacobianEndEffector.getNumRows(); r++)
      {
         for (int c=0; c < (jacobianEndEffector.getNumCols() - (hand-larm)); c++)
         {
            double a = jacobianEndEffector.get(r, c);
            double b = jacobianWristPlusOffset.get(r, c);
            if( Math.abs(a+b)> 0.0001)
            {
               double errorPercent =  100*(a-b)/(a+b) ;
               if( doprint )
                  System.out.format("A: %.1f\n",  errorPercent );
               assertTrue( errorPercent < 1 && errorPercent > -1 );
            }
         }
      }
      
      for (int r=0; r< jacobianManualCalculation.getNumRows(); r++)
      {
         for (int c=0; c< jacobianManualCalculation.getNumCols(); c++)
         {
            double a = jacobianEndEffector.get(r, c);
            double b = jacobianManualCalculation.get(r, c);
            if( Math.abs(a+b)> 0.001)
            {
               double errorPercent =  100*(a-b)/(a+b) ;
               if( doprint )
                  System.out.format("B: %.1f\n",  errorPercent );
               assertTrue( errorPercent < 1 && errorPercent > -1 );
            }
         }
      }  
   }
   
   
  /* public void test1()  throws Exception
   {
      NativeLibraryLoader.loadLibrary("us.ihmc.robotics.hierarchicalKinematics", "hik_java");

      model =  new RobotModel();
      model.loadURDF( getClass().getResource("atlas_v4_wb.urdf").getPath() , true);

      forward_solver = new ForwardKinematicSolver(model);

      VectorXd Q = new VectorXd( model.getNrOfJoints() );
      Q.setZero();
      forward_solver.updateKinematics( Q );

      String[] names = new  String[] {
            "back_bkz",
            "back_bky",
            "back_bkx",
            "l_arm_shz",
            "l_arm_shx",
            "l_arm_ely",
            "l_arm_elx",
            "l_arm_wry",
            "l_arm_wrx",
            "r_arm_shz",
            "r_arm_shx",
            "r_arm_ely",
            "r_arm_elx",
            "r_arm_wry",
            "r_arm_wrx",
            "l_leg_hpz",
            "l_leg_hpx",
            "l_leg_hpy",
            "l_leg_kny",
            "l_leg_aky",
            "l_leg_akx",
            "r_leg_hpz",
            "r_leg_hpx",
            "r_leg_hpy",
            "r_leg_kny",
            "r_leg_aky",
            "r_leg_akx"
      };

      VectorXd quat = new VectorXd(4);
      VectorXd pos = new Vector3d();

      for (String name : names)
      {
         int jointIndex = model.getJointIndexByName(name);
         int bodyId = model.getChildBodyOfJoint( jointIndex );
         forward_solver.getBodyPose( bodyId, quat, pos);

         pos.set( 0, pos.get(0) + 0.0);
         pos.set( 1, pos.get(1) -0.1115);
         pos.set( 2, pos.get(2) -0.862);

         System.out.println( name + "( " + jointIndex + ") : " + pos);
      }  
   }*/
}
