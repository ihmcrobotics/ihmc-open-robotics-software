package us.ihmc.robotics.screwTheory;

import org.ejml.alg.dense.linsol.LinearSolverSafe;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.junit.Test;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.Random;

public class DesiredJointAccelerationCalculatorTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

	@DeployableTestMethod(estimatedDuration = 0.2)
	@Test(timeout = 30000)
   public void testVersusSpatialAccelerationCalculator()
   {
      Random random = new Random(44345L);

      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(),
                                        new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoF", elevator, elevatorFrame);
      RigidBody floatingBase = ScrewTestTools.addRandomRigidBody("floatingBase", random, sixDoFJoint);

      ArrayList<RevoluteJoint> jointsList = new ArrayList<RevoluteJoint>();
      Vector3d[] jointAxes = new Vector3d[]
      {
         X, Y, X, Z, X, Y
      };

      ScrewTestTools.createRandomChainRobot("test", jointsList, floatingBase, jointAxes, random);



      RigidBody[] rigidBodiesInOrder = ScrewTools.computeSupportAndSubtreeSuccessors(elevator);
      RigidBody base = floatingBase;
      RigidBody endEffector = rigidBodiesInOrder[rigidBodiesInOrder.length - 1];
      GeometricJacobian jacobian = new GeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());
      LinearSolver<DenseMatrix64F> jacobianSolver = new LinearSolverSafe<DenseMatrix64F>(LinearSolverFactory.leastSquaresQrPivot(true, false));
      DesiredJointAccelerationCalculator desiredJointAccelerationCalculator = new DesiredJointAccelerationCalculator(jacobian, jacobianSolver);

      double minJacobianDeterminant = 1e-3;
      int testIndex = 0;
      int nTests = 10000;
      while (testIndex < nTests)
      {
         ScrewTestTools.setRandomPositionAndOrientation(sixDoFJoint, random);
         ScrewTestTools.setRandomVelocity(sixDoFJoint, random);
         ScrewTestTools.setRandomPositions(jointsList, random);
         ScrewTestTools.setRandomVelocities(jointsList, random);
         elevator.updateFramesRecursively();
         jacobian.compute();

         if (Math.abs(jacobian.det()) > minJacobianDeterminant)
         {
            Vector3d desiredAngularAcceleration = RandomTools.generateRandomVector(random);
            Vector3d desiredLinearAcceleration = RandomTools.generateRandomVector(random);
            SpatialAccelerationVector accelerationOfEndEffectorWithRespectToBase = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(),
                                                                                      base.getBodyFixedFrame(), endEffector.getBodyFixedFrame(),
                                                                                      desiredAngularAcceleration, desiredLinearAcceleration);

            desiredJointAccelerationCalculator.compute(accelerationOfEndEffectorWithRespectToBase);

            TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator);
            SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, twistCalculator, 0.0, true);

            twistCalculator.compute();
            spatialAccelerationCalculator.compute();

            SpatialAccelerationVector accelerationOfEndEffectorWithRespectToBaseBack = new SpatialAccelerationVector();
            spatialAccelerationCalculator.packRelativeAcceleration(accelerationOfEndEffectorWithRespectToBaseBack, base, endEffector);

            SpatialMotionVectorTest.assertSpatialMotionVectorEquals(accelerationOfEndEffectorWithRespectToBase, accelerationOfEndEffectorWithRespectToBaseBack, 1e-10);
            testIndex++;
         }
      }
   }
}
