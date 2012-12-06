package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import org.ejml.alg.dense.linsol.LinearSolver;
import org.ejml.alg.dense.linsol.LinearSolverFactory;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;

import us.ihmc.utilities.MechanismGeometricJacobian;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.geometry.CenterOfMassReferenceFrame;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.MomentumCalculator;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTestTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.utilities.test.JUnitTools;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class MomentumSolverTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testFloatingChain()
   {
      Random random = new Random(44345L);

      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new Transform3D());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      SixDoFJoint rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
      RigidBody rootBody = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();
      Vector3d[] jointAxes = new Vector3d[]
      {
         X, Y, X, Z, X, Y
      };

      ScrewTestTools.createRandomChainRobot("test", joints, rootBody, jointAxes, random);

      ScrewTestTools.setRandomPositionAndOrientation(rootJoint, random);
      ScrewTestTools.setRandomVelocity(rootJoint, random);
      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);
      elevator.updateFramesRecursively();
      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);

      Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations = new HashMap<InverseDynamicsJoint, DenseMatrix64F>();
      for (RevoluteJoint joint : joints)
      {
         DenseMatrix64F jointSpaceAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         RandomMatrices.setRandom(jointSpaceAcceleration, -1.0, 1.0, random);
         jointSpaceAccelerations.put(joint, jointSpaceAcceleration);
      }

      Map<MechanismGeometricJacobian, SpatialAccelerationVector> taskSpaceAccelerations = new HashMap<MechanismGeometricJacobian, SpatialAccelerationVector>();

      doChecks(random, elevator, rootJoint, sixDoFJoints, joints, jointSpaceAccelerations, taskSpaceAccelerations);
   }


   @Test
   public void testFloatingTree()
   {
      Random random = new Random(44345L);

      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new Transform3D());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      SixDoFJoint rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
      RigidBody rootBody = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();

      ScrewTestTools.createRandomTreeRobot(joints, rootBody, 25, random);

      ScrewTestTools.setRandomPositionAndOrientation(rootJoint, random);
      ScrewTestTools.setRandomVelocity(rootJoint, random);
      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);
      elevator.updateFramesRecursively();

      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);

      Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations = new HashMap<InverseDynamicsJoint, DenseMatrix64F>();
      for (RevoluteJoint joint : joints)
      {
         DenseMatrix64F jointSpaceAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         RandomMatrices.setRandom(jointSpaceAcceleration, -1.0, 1.0, random);
         jointSpaceAccelerations.put(joint, jointSpaceAcceleration);
      }

      Map<MechanismGeometricJacobian, SpatialAccelerationVector> taskSpaceAccelerations = new HashMap<MechanismGeometricJacobian, SpatialAccelerationVector>();

      doChecks(random, elevator, rootJoint, sixDoFJoints, joints, jointSpaceAccelerations, taskSpaceAccelerations);
   }

   @Test
   public void testTwoFloatingBodiesWithTaskSpaceAcceleration()
   {
      Random random = new Random(12342L);
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new Transform3D());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      SixDoFJoint rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
      RigidBody rootBody = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

      SixDoFJoint secondFloatingJoint = new SixDoFJoint("secondFloatingJoint", rootBody, rootBody.getBodyFixedFrame());
      RigidBody secondBody = ScrewTestTools.addRandomRigidBody("secondBody", random, secondFloatingJoint);

      ScrewTestTools.setRandomPositionAndOrientation(rootJoint, random);
      ScrewTestTools.setRandomVelocity(rootJoint, random);

      ScrewTestTools.setRandomPositionAndOrientation(secondFloatingJoint, random);
      ScrewTestTools.setRandomVelocity(secondFloatingJoint, random);
      elevator.updateFramesRecursively();

      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);
      sixDoFJoints.add(secondFloatingJoint);
      ArrayList<RevoluteJoint> oneDoFJoints = new ArrayList<RevoluteJoint>();

      Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations = new HashMap<InverseDynamicsJoint, DenseMatrix64F>();
      Map<MechanismGeometricJacobian, SpatialAccelerationVector> taskSpaceAccelerations = new HashMap<MechanismGeometricJacobian, SpatialAccelerationVector>();
      MechanismGeometricJacobian jacobian = new MechanismGeometricJacobian(rootBody, secondBody, rootJoint.getFrameAfterJoint());
      SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(jacobian.getEndEffectorFrame(), elevatorFrame, jacobian.getJacobianFrame(),
                                                         RandomTools.getRandomVector(random), RandomTools.getRandomVector(random));

      taskSpaceAccelerations.put(jacobian, spatialAcceleration);

      doChecks(random, elevator, rootJoint, sixDoFJoints, oneDoFJoints, jointSpaceAccelerations, taskSpaceAccelerations);
   }

   private MomentumSolver createAndInitializeMomentumOptimizer(RigidBody elevator, SixDoFJoint rootJoint, List<SixDoFJoint> sixDoFJoints,
           ArrayList<RevoluteJoint> joints, double dt, ReferenceFrame centerOfMassFrame, TwistCalculator twistCalculator)
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");

//      DampedLeastSquaresSolver jacobianSolver = new DampedLeastSquaresSolver(SpatialMotionVector.SIZE);
//      jacobianSolver.setAlpha(0.0);
      LinearSolver<DenseMatrix64F> jacobianSolver = LinearSolverFactory.linear(SpatialMotionVector.SIZE);
      MomentumSolver solver = new MomentumSolver(rootJoint, elevator, centerOfMassFrame, twistCalculator, jacobianSolver, dt, registry);
      solver.initialize();

      for (SixDoFJoint sixDoFJoint : sixDoFJoints)
      {
         ScrewTestTools.integrateVelocities(sixDoFJoint, dt);
      }

      ScrewTestTools.integrateVelocities(joints, dt);
      elevator.updateFramesRecursively();

      return solver;
   }

   private void doChecks(Random random, RigidBody elevator, SixDoFJoint rootJoint, List<SixDoFJoint> sixDoFJoints, ArrayList<RevoluteJoint> oneDoFJoints,
                         Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations,
                         Map<MechanismGeometricJacobian, SpatialAccelerationVector> taskSpaceAccelerations)
   {
      double dt = 1e-8;
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);

      FrameVector desiredAngularCentroidalMomentumRate = new FrameVector(centerOfMassFrame, RandomTools.getRandomVector(random));
      FrameVector desiredLinearCentroidalMomentumRate = new FrameVector(centerOfMassFrame, RandomTools.getRandomVector(random));

      MomentumSolver solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, oneDoFJoints, dt, centerOfMassFrame, twistCalculator);

      twistCalculator.compute();
      solver.solve(desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate, jointSpaceAccelerations, taskSpaceAccelerations);

      checkJointSpaceAccelerations(jointSpaceAccelerations, 0.0);
      checkTaskSpaceAccelerations(rootJoint, twistCalculator, taskSpaceAccelerations, 1e-5);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, oneDoFJoints, dt, desiredAngularCentroidalMomentumRate,
              desiredLinearCentroidalMomentumRate, 1e-4);
   }

   private static void checkJointSpaceAccelerations(Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations, double epsilon)
   {
      for (InverseDynamicsJoint joint : jointSpaceAccelerations.keySet())
      {
         DenseMatrix64F check = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         joint.packDesiredAccelerationMatrix(check, 0);
         JUnitTools.assertMatrixEquals(jointSpaceAccelerations.get(joint), check, epsilon);
      }
   }

   private static void checkTaskSpaceAccelerations(SixDoFJoint rootJoint, TwistCalculator twistCalculator,
           Map<MechanismGeometricJacobian, SpatialAccelerationVector> taskSpaceAccelerations, double epsilon)
   {
      RigidBody elevator = rootJoint.getPredecessor();
      ReferenceFrame rootFrame = elevator.getBodyFixedFrame();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootFrame, rootFrame, rootFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, rootFrame, rootAcceleration, twistCalculator,
                                                                       true, true);
      spatialAccelerationCalculator.compute();

      for (MechanismGeometricJacobian jacobian : taskSpaceAccelerations.keySet())
      {
         SpatialAccelerationVector acceleration = taskSpaceAccelerations.get(jacobian);

         SpatialAccelerationVector checkAcceleration = new SpatialAccelerationVector();
         spatialAccelerationCalculator.packAccelerationOfBody(checkAcceleration, jacobian.getEndEffector());

         Twist twistOfCurrentWithRespectToNew = new Twist();
         Twist twistOfBodyWithRespectToBase = new Twist();
         twistCalculator.packRelativeTwist(twistOfCurrentWithRespectToNew, jacobian.getBase(), jacobian.getEndEffector());
         twistOfCurrentWithRespectToNew.changeBaseFrameNoRelativeTwist(rootJoint.getFrameAfterJoint());

         twistCalculator.packTwistOfBody(twistOfBodyWithRespectToBase, jacobian.getEndEffector());
         checkAcceleration.changeFrame(acceleration.getExpressedInFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);

         JUnitTools.assertSpatialMotionVectorEquals(acceleration, checkAcceleration, epsilon);
      }
   }

   private static void checkAgainstNumericalDifferentiation(SixDoFJoint rootJoint, List<SixDoFJoint> sixDoFJoints, ArrayList<RevoluteJoint> joints, double dt,
           FrameVector desiredAngularCentroidalMomentumRate, FrameVector desiredLinearCentroidalMomentumRate, double epsilon)
   {
      desiredAngularCentroidalMomentumRate.checkReferenceFrameMatch(desiredLinearCentroidalMomentumRate.getReferenceFrame());
      ReferenceFrame referenceFrame = desiredAngularCentroidalMomentumRate.getReferenceFrame();

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getPredecessor());
      MomentumCalculator momentumCalculator = new MomentumCalculator(twistCalculator);
      Momentum momentum0 = new Momentum(ReferenceFrame.getWorldFrame());
      twistCalculator.compute();
      momentumCalculator.computeAndPack(momentum0);

      for (SixDoFJoint sixDoFJoint : sixDoFJoints)
      {
         ScrewTestTools.copyDesiredAccelerationToActual(sixDoFJoint);
         ScrewTestTools.integrateAccelerations(sixDoFJoint, dt);
         ScrewTestTools.integrateVelocities(sixDoFJoint, dt);
      }

      ScrewTestTools.copyDesiredAccelerationsToActual(joints);
      ScrewTestTools.integrateAccelerations(joints, dt);
      ScrewTestTools.integrateVelocities(joints, dt);

      rootJoint.getPredecessor().updateFramesRecursively();
      twistCalculator.compute();
      Momentum momentum1 = new Momentum(ReferenceFrame.getWorldFrame());
      momentumCalculator.computeAndPack(momentum1);

      Momentum momentumRateNumerical = new Momentum(momentum1);
      momentumRateNumerical.sub(momentum0);
      momentumRateNumerical.scale(1.0 / dt);
      momentumRateNumerical.changeFrame(referenceFrame);

      JUnitTools.assertTuple3dEquals(desiredAngularCentroidalMomentumRate.getVector(), momentumRateNumerical.getAngularPartCopy(), epsilon);
      JUnitTools.assertTuple3dEquals(desiredLinearCentroidalMomentumRate.getVector(), momentumRateNumerical.getLinearPartCopy(), epsilon);
   }

   private static void checkAgainstInverseDynamicsCalculator(SixDoFJoint rootJoint, FrameVector desiredAngularCentroidalMomentumRate,
           FrameVector desiredLinearCentroidalMomentumRate, double epsilonInverseDynamics)
   {
      desiredAngularCentroidalMomentumRate.checkReferenceFrameMatch(desiredLinearCentroidalMomentumRate.getReferenceFrame());
      ReferenceFrame referenceFrame = desiredAngularCentroidalMomentumRate.getReferenceFrame();

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getPredecessor());
      twistCalculator.compute();

      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, 0.0);
      inverseDynamicsCalculator.compute();
      Wrench rootJointWrench = new Wrench();
      rootJoint.packWrench(rootJointWrench);
      rootJointWrench.changeFrame(referenceFrame);

      JUnitTools.assertTuple3dEquals(desiredAngularCentroidalMomentumRate.getVector(), rootJointWrench.getAngularPartCopy(), epsilonInverseDynamics);
      JUnitTools.assertTuple3dEquals(desiredLinearCentroidalMomentumRate.getVector(), rootJointWrench.getLinearPartCopy(), epsilonInverseDynamics);
   }
}
