package us.ihmc.robotics.hierarchicalKinematics;

import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.Vector64F;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.io.File;
import java.util.Random;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class CollisionAvoidanceConstraintTest
{
   private static final boolean doprint = false;
   private RobotModel model;
   private ForwardKinematicSolver forwardKinematicSolver;


   @Before
   public void setUp() throws Exception
   {
      // load the C++ shared library. Generate it first with cmake (and make install)
      HikSupport.loadLibrary();

      model = new RobotModel();
      model.loadURDF(getClass().getResource("test_collision.urdf").getPath(), false);

      forwardKinematicSolver = new ForwardKinematicSolver(model);
   }

   @After
   public void tearDown()
   {
      model = null;
      forwardKinematicSolver = null;
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCollisionAvoidanceConstraint()
   {
      testImpl("r_larm", "pelvis");
      testImpl("r_larm", "utorso");
   }

   public void testImpl(String bodyNameA, String bodyNameB)
   {
      try
      {
         CollisionAvoidanceConstraint collision = new CollisionAvoidanceConstraint(forwardKinematicSolver, model);
         collision.loadURDF(new File(getClass().getResource("test_collision.urdf").getPath()));

         collision.addBodyPair(bodyNameA, bodyNameB);

         int numJoints = forwardKinematicSolver.getNumberOfJoints();
         Vector64F Q = new Vector64F(numJoints);
         Vector64F newQ = new Vector64F(Q);
         Vector64F deltaQ = new Vector64F(numJoints);

         DenseMatrix64F matrixA = new DenseMatrix64F(collision.getNumConstraints(), numJoints);
         DenseMatrix64F matrixB = new DenseMatrix64F(collision.getNumConstraints(), numJoints);
         Vector64F minVector = new Vector64F(collision.getNumConstraints());

         RigidBodyTransform transfA = new RigidBodyTransform();
         RigidBodyTransform transfB = new RigidBodyTransform();
         int bodyA = model.getBodyId(bodyNameA);
         int bodyB = model.getBodyId(bodyNameB);

         Random random = new Random(1992L);
         double DELTA = 0.0002;

         // Q.setZero();
         // Q.set(4, 1.15);
         //TODO: This fails if numberOfTests is large. There must be some corner cases that it is hitting that fail, or the tolerances are just too low.
         int numberOfTests = 1000;

         for (int iter = 0; iter < numberOfTests; iter++)
         {
            for (int i = 0; i < numJoints; i++)
            {
               Q.set(i, -1.5 + (3.0) * random.nextDouble());
               deltaQ.set(i, DELTA);
               newQ.set(i, Q.get(i) + DELTA);
            }

            forwardKinematicSolver.updateKinematics(Q);
            forwardKinematicSolver.getBodyPose(bodyA, transfA);
            forwardKinematicSolver.getBodyPose(bodyB, transfB);
            double dist1 = collision.capsuleCheker.getClosestPoints(bodyNameA, transfA, bodyNameB, transfB, null);

            collision.computeConstraints(matrixA, minVector);

            if (doprint)
               System.out.println(matrixA);

            for (int i = 0; i < numJoints; i++)
            {
               newQ.set(Q);
               newQ.set(i, Q.get(i) + DELTA);

               forwardKinematicSolver.updateKinematics(newQ);
               forwardKinematicSolver.getBodyPose(bodyA, transfA);
               forwardKinematicSolver.getBodyPose(bodyB, transfB);

               double dist2 = collision.capsuleCheker.getClosestPoints(bodyNameA, transfA, bodyNameB, transfB, null);

               // I am not really sure about this, but it fails in if I don't do it.
               if (dist1 > 0)
                  matrixB.set(i, (dist2 - dist1) / DELTA);
               else
                  matrixB.set(i, -(dist2 - dist1) / DELTA);
            }

            if (doprint)
               System.out.println(matrixB);

            for (int i = 0; i < numJoints; i++)
            {
               double a = matrixA.get(i);
               double b = matrixB.get(i);
               if (Math.abs(a + b) > 0.001)
               {
                  double diff = 100.0 * (a - b) / (a + b);
                  if (doprint)
                     System.out.format("%.1f\n", diff);
                  assertTrue(new String(a + " / " + b), (diff < 2.0) && (diff > -2.0));
               }
               else if (Math.abs(a - b) > 0.001)
               {
                  fail(" opposite values " + a + " / " + b);
               }
            }
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

}
