package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Random;

import org.ejml.alg.dense.linsol.LinearSolverSafe;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class DesiredJointAccelerationCalculatorTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

	@ContinuousIntegrationTest(estimatedDuration = 0.6)
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
      Vector3D[] jointAxes = new Vector3D[]
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
            Vector3D desiredAngularAcceleration = RandomGeometry.nextVector3D(random);
            Vector3D desiredLinearAcceleration = RandomGeometry.nextVector3D(random);
            SpatialAccelerationVector accelerationOfEndEffectorWithRespectToBase = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(),
                                                                                      base.getBodyFixedFrame(), endEffector.getBodyFixedFrame(),
                                                                                      desiredAngularAcceleration, desiredLinearAcceleration);

            desiredJointAccelerationCalculator.compute(accelerationOfEndEffectorWithRespectToBase);

            TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator);
            SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, twistCalculator, 0.0, true);

            twistCalculator.compute();
            spatialAccelerationCalculator.compute();

            SpatialAccelerationVector accelerationOfEndEffectorWithRespectToBaseBack = new SpatialAccelerationVector();
            spatialAccelerationCalculator.getRelativeAcceleration(accelerationOfEndEffectorWithRespectToBaseBack, base, endEffector);

            SpatialMotionVectorTest.assertSpatialMotionVectorEquals(accelerationOfEndEffectorWithRespectToBase, accelerationOfEndEffectorWithRespectToBaseBack, 1e-10);
            testIndex++;
         }
      }
   }
}
