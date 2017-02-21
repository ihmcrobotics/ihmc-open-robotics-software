package us.ihmc.robotics.kinematics;

import static org.junit.Assert.fail;

import java.util.List;
import java.util.Random;

import org.apache.commons.math3.stat.descriptive.SummaryStatistics;
import org.junit.Test;

import us.ihmc.commons.Conversions;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.ScrewTestTools;

/**
 * @author twan
 *         Date: 6/1/13
 */
public class NumericalInverseKinematicsCalculatorTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);
   private static final boolean DEBUG = false;

   /*
    * make sure there are no exceptions when you pass in an infeasible desired transform
    */

	@ContinuousIntegrationTest(estimatedDuration = 3.7)
	@Test(timeout = 30000)
   public void testInfeasible()
   {
      Random random = new Random(1235125L);
      Vector3D[] jointAxes = new Vector3D[]
      {
         X, Y, Z, Y, Y, X
      };
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      GeometricJacobian jacobian = new GeometricJacobian(randomFloatingChain.getRootJoint().getSuccessor(), randomFloatingChain.getLeafBody(),
                                      randomFloatingChain.getLeafBody().getBodyFixedFrame());

      RandomRestartInverseKinematicsCalculator calculator = createCalculator(jacobian, 100);

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();

      int nTests = 200;
      SummaryStatistics iterationStatistics = new SummaryStatistics();
      SummaryStatistics timeStatistics = new SummaryStatistics();
      for (int i = 0; i < nTests; i++)
      {
         setRandomPositions(random, revoluteJoints, Math.PI);

         AxisAngle axisAngle = RandomTools.generateRandomRotation(random);
         RotationMatrix rotation = new RotationMatrix();
         rotation.set(axisAngle);
         Vector3D translation = RandomTools.generateRandomVector(random, 50.0);
         RigidBodyTransform desiredTransform = new RigidBodyTransform(rotation, translation);

         long t0 = System.nanoTime();
         calculator.solve(desiredTransform);
         long tf = System.nanoTime();
         long solutionTime = tf - t0;

         timeStatistics.addValue(Conversions.nanoSecondstoSeconds(solutionTime));
         iterationStatistics.addValue(calculator.getNumberOfIterations());
      }

      if (DEBUG) printStatistics(iterationStatistics, timeStatistics);
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.4)
	@Test(timeout = 30000)
   public void testForwardThenInverse()
   {
      Random random = new Random(125125L);
      Vector3D[] jointAxes = new Vector3D[]
      {
         X, Y, Z, Y, Y, X
      };
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      GeometricJacobian jacobian = new GeometricJacobian(randomFloatingChain.getRootJoint().getSuccessor(), randomFloatingChain.getLeafBody(),
                                      randomFloatingChain.getLeafBody().getBodyFixedFrame());

      RandomRestartInverseKinematicsCalculator calculator = createCalculator(jacobian, 200);

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();

      int nTests = 1000;
      SummaryStatistics iterationStatistics = new SummaryStatistics();
      SummaryStatistics timeStatistics = new SummaryStatistics();

      for (int i = 0; i < nTests; i++)
      {
         setRandomPositions(random, revoluteJoints, 0.25 * Math.PI);
         
         jacobian.compute();
         double det = jacobian.det();
         RigidBodyTransform desiredTransform = jacobian.getEndEffectorFrame().getTransformToDesiredFrame(jacobian.getBaseFrame());

         setRandomPositions(random, revoluteJoints, 0.25 * Math.PI);

         long t0 = System.nanoTime();
         calculator.solve(desiredTransform);
         long tf = System.nanoTime();
         long solutionTime = tf - t0;
         timeStatistics.addValue(Conversions.nanoSecondstoSeconds(solutionTime));
         iterationStatistics.addValue(calculator.getNumberOfIterations());

         RigidBodyTransform solvedTransform = jacobian.getEndEffectorFrame().getTransformToDesiredFrame(jacobian.getBaseFrame());



         boolean equal = solvedTransform.epsilonEquals(desiredTransform, 0.02); //1e-5);
         if (!equal)
         {
            System.out.println("Test number " + i + " failed.");
            System.out.println("Initial Jacobian determinant: " + det);
            System.out.println("Current Jacobian determinant: " + jacobian.det());
            System.out.println("Number of iterations: " + calculator.getNumberOfIterations());
            System.out.println("Error: " + calculator.getErrorScalar());
            System.out.print("Joint angles: ");

            for (RevoluteJoint revoluteJoint : revoluteJoints)
            {
               System.out.print(revoluteJoint.getQ() + ", ");
            }

            System.out.println("\n");

            fail();
         }
      }

      printStatistics(iterationStatistics, timeStatistics);
   }

   private void printStatistics(SummaryStatistics iterationStatistics, SummaryStatistics timeStatistics)
   {
      System.out.println("max time per solve: " + timeStatistics.getMax() + " [s]");
      System.out.println("avg time per solve: " + timeStatistics.getMean() + " [s]");
      System.out.println("max number of iterations necessary: " + iterationStatistics.getMax());
      System.out.println("avg number of iterations necessary: " + iterationStatistics.getMean());
   }

   private RandomRestartInverseKinematicsCalculator createCalculator(GeometricJacobian jacobian, int maxIterations)
   {
      double lambdaLeastSquares = 0.0009;
      double tolerance = 0.001;
      double maxStepSize = 0.2;
      double minRandomSearchScalar = 0.02;
      double maxRandomSearchScalar = 0.8;

      int maxRestarts = 20;
      double restartTolerance = 0.001;
      
      NumericalInverseKinematicsCalculator numericalInverseKinematicsCalculator = new NumericalInverseKinematicsCalculator(jacobian, lambdaLeastSquares, tolerance, maxIterations, maxStepSize, minRandomSearchScalar, maxRandomSearchScalar);
      numericalInverseKinematicsCalculator.setLimitJointAngles(false);
      RandomRestartInverseKinematicsCalculator randomRestartInverseKinematicsCalculator = new RandomRestartInverseKinematicsCalculator(maxRestarts, restartTolerance, jacobian, numericalInverseKinematicsCalculator);
      return randomRestartInverseKinematicsCalculator;
   }

   private void setRandomPositions(Random random, List<RevoluteJoint> revoluteJoints, double deltaThetaMax)
   {
      for (RevoluteJoint revoluteJoint : revoluteJoints)
      {
//         revoluteJoint.setQ(revoluteJoint.getQ() + RandomTools.generateRandomDouble(random, -deltaThetaMax, deltaThetaMax));
         revoluteJoint.setQ(RandomTools.generateRandomDouble(random, -deltaThetaMax, deltaThetaMax));
         revoluteJoint.getFrameAfterJoint().update();
      }
   }
}
