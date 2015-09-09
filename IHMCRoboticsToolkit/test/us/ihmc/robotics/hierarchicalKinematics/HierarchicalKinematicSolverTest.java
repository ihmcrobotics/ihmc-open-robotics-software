package us.ihmc.robotics.hierarchicalKinematics;

import org.ejml.data.DenseMatrix64F;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.Vector64F;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.util.ArrayList;

import static org.junit.Assert.fail;

public class HierarchicalKinematicSolverTest {

   private RobotModel model;
   private HierarchicalTaskCOM task_com;
   private HierarchicalTaskBodyPose task_pos_L;
   private HierarchicalTaskBodyPose task_pos_R;
   private HierarchicalTaskJointsPose task_joints;
   private HierarchicalKinematicSolver  wb_solver;
   private DenseMatrix64F               couple_j_weights;

   protected boolean DEBUG = false;

   @Before
   public void setUp() throws Exception {

      // load the C++ shared library. Generate it first with cmake (and make install)
      NativeLibraryLoader.loadLibrary("us.ihmc.robotics.hierarchicalKinematics", "hik_java");
      NativeLibraryLoader.loadLibrary("us.ihmc.convexOptimization", "qpOASESSwig_rel");
      //		NativeLibraryLoader.loadLibrary("us.ihmc.utilities.qpSolver", "qpOASES_java");

      model = new RobotModel();

      String urdfPath = getClass().getResource("atlas_v4_dual.urdf").getPath();
      model.loadURDF(urdfPath, DEBUG);
      wb_solver = new HierarchicalKinematicSolver(model, 35, 0.00001);

      int numJoints =  wb_solver.getNumberOfJoints();

      // note: order is important.
      task_com    = new HierarchicalTaskCOM("", wb_solver.getForwardSolver() );
      task_pos_R  = new HierarchicalTaskBodyPose("", wb_solver.getForwardSolver(), model, "r_ee_link" );
      task_pos_L  = new HierarchicalTaskBodyPose("", wb_solver.getForwardSolver(), model, "l_ee_link"  );
      task_joints = new HierarchicalTaskJointsPose("", wb_solver.getForwardSolver() );

      wb_solver.addTask( (HierarchicalTask) task_com );
      wb_solver.addTask( (HierarchicalTask) task_pos_R );
      wb_solver.addTask( (HierarchicalTask) task_pos_L );
      wb_solver.addTask( (HierarchicalTask) task_joints );

      //	task_pos_R.setEnabled(true);
      ///	task_pos_L.setEnabled(true);
      //	task_joints.setEnabled(true);


      // --------------------------
      // controls only the position of COM in X direction
      Vector64F weights_COM = new Vector64F(3);
      weights_COM.zero();
      weights_COM.set(0, 1);

      task_com.setWeightsTaskSpace(weights_COM);
      // --------------------------
      // controls only the position of end effector but not the rotation
      Vector64F weights_POS = new Vector64F(6);
      weights_POS.zero();
      weights_POS.set(0, 1);
      weights_POS.set(1, 1);
      weights_POS.set(2, 1);

      task_pos_R.setWeightsTaskSpace(weights_POS);
      task_pos_L.setWeightsTaskSpace(weights_POS);
      // --------------------------
      Vector64F weights_Joints = new Vector64F(numJoints); 
      weights_Joints.zero();
      weights_Joints.set(1, 1); // KNEE
      weights_Joints.set(2, 1); // HIP   X
      weights_Joints.set(3, 1); // waist Z
      weights_Joints.set(4, 1); // waist Y
      weights_Joints.set(5, 1); // waist X

      // this is a little tricky to understand: it means 
      // 		q(2) - q(0) - q(1) = 0
      // this will keep the hip upright (i.e. parallel to the foot)
      couple_j_weights = new DenseMatrix64F(numJoints, numJoints);
      couple_j_weights.zero();
      couple_j_weights.set(2, 0, -1);
      couple_j_weights.set(2, 1, -1);

      task_joints.setWeightsTaskSpace( weights_Joints );
      task_joints.setCoupledJointWeights(couple_j_weights);

   }

	@DeployableTestMethod
	@Ignore //This test hangs bamboo, we need to figure out why? see http://bamboo.ihmc.us/browse/RC-ALL2-IHMCUTILITIES-51/log
	@Test(timeout=300000)
   public void test() {

      int numJoints =  wb_solver.getNumberOfJoints();
      final Quat4d orientation = new Quat4d(0,0,0,1);
      final ArrayList<Point3d> position = new ArrayList<Point3d>();

      final Vector64F com_desired = new Vector64F(3);
      com_desired.zero();
      task_com.setErrorTolerance( 0.01 );
      task_com.setTarget(com_desired);

      final Vector64F desired_J = new Vector64F( numJoints );
      desired_J.zero();


      final Vector64F Q_out  = new Vector64F( numJoints );
      final Vector64F Q_init = new Vector64F( numJoints ); 

      for (int i=0; i<numJoints; i++)
      {
         Q_init.set(i, 0.5* (model.q_max(i) + model.q_min(i)) );
      }

      //position.add( new Point3d( 0.5,   -0.1,   1.4 ));
      position.add( new Point3d( 0.3,   -0.3,   0.3 ));
      position.add( new Point3d( 0.4,   -0.0,   0.35 ));
      position.add( new Point3d( 0.8,   -0.1,    1.2 ));
      position.add( new Point3d( 0.7,   -0.2,    1.1 ));
      position.add( new Point3d( 0.4,   -0.3,    1.2 ));
      position.add( new Point3d( 0.5,   -0.1,    0.5 ));

      int REPEAT = 1000;
      wb_solver.setVerbosityLevel(1);

      for (long iter = 0; iter< REPEAT ; iter++)
      {
         int C = (int) (iter  % position.size());
         final Vector64F pos_desired = new Vector64F(7);
         pos_desired.zero();

         pos_desired.set(0, position.get(C).x );
         pos_desired.set(1, position.get(C).y );
         pos_desired.set(2, position.get(C).z );

         pos_desired.set(3, orientation.x );
         pos_desired.set(4, orientation.y );
         pos_desired.set(5, orientation.z );
         pos_desired.set(6, orientation.w );

         // update the desired position
         task_pos_R.setTarget(pos_desired);

         pos_desired.set( 0, 0.4 );
         pos_desired.set( 1, 0.3 );
         pos_desired.set( 2, 0.7 );
         task_pos_L.setTarget(pos_desired);

         desired_J.set(1, 2.5 - position.get(C).z );
         task_joints.setTarget( desired_J );

         int ret = -1;
         try
         {
            ret = wb_solver.solve(Q_init, Q_out, false);
         }
         catch (Exception e)
         {
            // TODO Auto-generated catch block
            e.printStackTrace();
         }

         if( ret < 0 )
         {
            if( DEBUG )   System.out.println("failed test "+ C + " at iteration "+ iter);
            fail("failed to solve case " + C + " at iteration "+ iter );
         }
         else {
            if( DEBUG )   System.out.println("Passed test "+ C + " in " + ret);
         }

         Q_init.set( Q_out );
      }
   }
}
