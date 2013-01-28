package us.ihmc.commonWalkingControlModules.momentumBasedController;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.ops.RandomMatrices;
import org.junit.Test;

import us.ihmc.utilities.MechanismGeometricJacobian;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.CenterOfMassReferenceFrame;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.MomentumCalculator;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTestTools;
import us.ihmc.utilities.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.utilities.test.JUnitTools;

import com.yobotics.simulationconstructionset.YoVariableRegistry;


//TODO: nullspace tests, tests where MomentumSolver is called multiple times in a row
public class MomentumSolverTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double DT = 1e-8;

   @Test
   public void testFloatingChain()
   {
      Random random = new Random(44345L);
      Vector3d[] jointAxes = new Vector3d[]
      {
         X, Y, X, Z, X, Y
      };

      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, jointAxes);

      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();
      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);
      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();

      setRandomPositionsAndVelocities(random, randomFloatingChain.getElevator(), sixDoFJoints, revoluteJoints);


      RigidBody elevator = randomFloatingChain.getElevator();
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();


      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      MomentumSolver solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, revoluteJoints, DT, centerOfMassFrame, twistCalculator);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));

      Map<InverseDynamicsJoint, DenseMatrix64F> jointAccelerations = new HashMap<InverseDynamicsJoint, DenseMatrix64F>();
      for (RevoluteJoint joint : revoluteJoints)
      {
         DenseMatrix64F jointSpaceAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         RandomMatrices.setRandom(jointSpaceAcceleration, -1.0, 1.0, random);
         solver.setDesiredJointAcceleration(joint, jointSpaceAcceleration);
         jointAccelerations.put(joint, jointSpaceAcceleration);
      }

      twistCalculator.compute();
      solver.compute();
      solver.solve(desiredMomentumRate);

      for (RevoluteJoint joint : revoluteJoints)
      {
         checkJointAcceleration(joint, jointAccelerations.get(joint), 1e-6);
      }

      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-4);
   }


   @Test
   public void testFloatingTree()
   {
      Random random = new Random(44345L);

      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new Transform3D());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      SixDoFJoint rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
      RigidBody body0 = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);
      ArrayList<RevoluteJoint> revoluteJoints = new ArrayList<RevoluteJoint>();

      ScrewTestTools.createRandomTreeRobot(revoluteJoints, body0, 25, random);

      setRandomPositionsAndVelocities(random, elevator, sixDoFJoints, revoluteJoints);

      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      MomentumSolver solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, revoluteJoints, DT, centerOfMassFrame, twistCalculator);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));

      Map<InverseDynamicsJoint, DenseMatrix64F> jointAccelerations = new HashMap<InverseDynamicsJoint, DenseMatrix64F>();
      for (RevoluteJoint joint : revoluteJoints)
      {
         DenseMatrix64F jointSpaceAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         RandomMatrices.setRandom(jointSpaceAcceleration, -1.0, 1.0, random);
         solver.setDesiredJointAcceleration(joint, jointSpaceAcceleration);
         jointAccelerations.put(joint, jointSpaceAcceleration);
      }

      twistCalculator.compute();
      solver.compute();
      solver.solve(desiredMomentumRate);

      for (RevoluteJoint joint : revoluteJoints)
      {
         checkJointAcceleration(joint, jointAccelerations.get(joint), 1e-6);
      }

      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-4);
   }

   @Test
   public void testTwoFloatingBodiesWithTaskSpaceAcceleration()
   {
      Random random = new Random(12342L);
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new Transform3D());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      SixDoFJoint rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
      RigidBody body0 = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

      SixDoFJoint body1 = new SixDoFJoint("secondFloatingJoint", body0, body0.getBodyFixedFrame());
      RigidBody secondBody = ScrewTestTools.addRandomRigidBody("secondBody", random, body1);

      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);
      sixDoFJoints.add(body1);
      ArrayList<RevoluteJoint> oneDoFJoints = new ArrayList<RevoluteJoint>();

      setRandomPositionsAndVelocities(random, elevator, sixDoFJoints, oneDoFJoints);


      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();


      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      MomentumSolver solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, oneDoFJoints, DT, centerOfMassFrame, twistCalculator);



      MechanismGeometricJacobian jacobian = new MechanismGeometricJacobian(body0, secondBody, rootJoint.getFrameAfterJoint());

      SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(jacobian.getEndEffectorFrame(), elevatorFrame,
                                                         jacobian.getEndEffectorFrame(), RandomTools.generateRandomVector(random),
                                                         RandomTools.generateRandomVector(random));

      DenseMatrix64F nullspaceMultiplier = new DenseMatrix64F(0, 1);
      solver.setDesiredSpatialAcceleration(jacobian, spatialAcceleration, nullspaceMultiplier);

      twistCalculator.compute();
      solver.compute();

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));
      solver.solve(desiredMomentumRate);

      checkTaskSpaceAcceleration(rootJoint, twistCalculator, jacobian, spatialAcceleration, 1e-9, false);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, new ArrayList<RevoluteJoint>(), DT, desiredMomentumRate, 1e-4);
   }

   @Test
   public void testJointSpaceAndTaskSpaceAccelerations()
   {
      Random random = new Random(13552L);

      Vector3d[] jointAxes = new Vector3d[]
      {
         X, X, Y, Z, Y, Y, X
      };

      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, jointAxes);

      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();
      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);
      
      setRandomPositionsAndVelocities(random, randomFloatingChain.getElevator(), sixDoFJoints, randomFloatingChain.getRevoluteJoints());

      RigidBody elevator = randomFloatingChain.getElevator();
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      MomentumSolver solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, randomFloatingChain.getRevoluteJoints(), DT,
                                 centerOfMassFrame, twistCalculator);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
            RandomTools.generateRandomVector(random));
      
      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();
      RevoluteJoint firstJoint = revoluteJoints.get(0);
      RigidBody base = firstJoint.getSuccessor();
      RigidBody endEffector = revoluteJoints.get(revoluteJoints.size() - 1).getSuccessor();
      MechanismGeometricJacobian jacobian = new MechanismGeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());
      SpatialAccelerationVector taskSpaceAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), elevator.getBodyFixedFrame(),
            endEffector.getBodyFixedFrame(), RandomTools.generateRandomVector(random),
            RandomTools.generateRandomVector(random));
      DenseMatrix64F nullspaceMultiplier = new DenseMatrix64F(0, 1);
      solver.setDesiredSpatialAcceleration(jacobian, taskSpaceAcceleration, nullspaceMultiplier);

      DenseMatrix64F firstJointAcceleration = new DenseMatrix64F(1, 1);
      firstJointAcceleration.set(0, 0, random.nextDouble());
      solver.setDesiredJointAcceleration(firstJoint, firstJointAcceleration);

      twistCalculator.compute();
      solver.compute();
      solver.solve(desiredMomentumRate);

      checkJointAcceleration(firstJoint, firstJointAcceleration, 1e-9);
      checkTaskSpaceAcceleration(rootJoint, twistCalculator, jacobian, taskSpaceAcceleration, 1e-9, false);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-4);
   }

   @Test
   public void testInternalSpatialAcceleration()
   {
      Random random = new Random(44345L);
      Vector3d[] jointAxes = new Vector3d[]
      {
         X, Y, X, Z, X, Y
      };

      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, jointAxes);

      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();
      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);

      setRandomPositionsAndVelocities(random, randomFloatingChain.getElevator(), sixDoFJoints, randomFloatingChain.getRevoluteJoints());

      RigidBody elevator = randomFloatingChain.getElevator();
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      MomentumSolver solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, randomFloatingChain.getRevoluteJoints(), DT,
                                 centerOfMassFrame, twistCalculator);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();
      RigidBody base = rootJoint.getSuccessor();
      RigidBody endEffector = revoluteJoints.get(revoluteJoints.size() - 1).getSuccessor();
      MechanismGeometricJacobian jacobian = new MechanismGeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());
      SpatialAccelerationVector internalAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), base.getBodyFixedFrame(),
                                                          endEffector.getBodyFixedFrame(), RandomTools.generateRandomVector(random),
                                                          RandomTools.generateRandomVector(random));
      DenseMatrix64F nullspaceMultiplier = new DenseMatrix64F(0, 1);

      solver.setDesiredSpatialAcceleration(jacobian, internalAcceleration, nullspaceMultiplier);

      twistCalculator.compute();
      solver.compute();
      solver.solve(desiredMomentumRate);

      checkInternalAcceleration(rootJoint, twistCalculator, jacobian, internalAcceleration, 1e-9);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-4);
   }

   @Test
   public void testSubspaces()
   {
      Random random = new Random(12342L);
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new Transform3D());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      SixDoFJoint rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
      RigidBody body0 = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

      SixDoFJoint body1 = new SixDoFJoint("secondFloatingJoint", body0, body0.getBodyFixedFrame());
      RigidBody secondBody = ScrewTestTools.addRandomRigidBody("secondBody", random, body1);

      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);
      sixDoFJoints.add(body1);
      ArrayList<RevoluteJoint> oneDoFJoints = new ArrayList<RevoluteJoint>();

      setRandomPositionsAndVelocities(random, elevator, sixDoFJoints, oneDoFJoints);

      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();


      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      MomentumSolver solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, oneDoFJoints, DT, centerOfMassFrame, twistCalculator);



      MechanismGeometricJacobian jacobian = new MechanismGeometricJacobian(body0, secondBody, rootJoint.getFrameAfterJoint());

      SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(jacobian.getEndEffectorFrame(), elevatorFrame,
                                                         jacobian.getEndEffectorFrame(), RandomTools.generateRandomVector(random),
                                                         RandomTools.generateRandomVector(random));

      DenseMatrix64F nullspaceMultiplier = new DenseMatrix64F(0, 1);
      solver.setDesiredSpatialAcceleration(jacobian, spatialAcceleration, nullspaceMultiplier);

      twistCalculator.compute();
      solver.compute();

      Vector3d rateOfChangeOfLinearMomentum = RandomTools.generateRandomVector(random);
      DenseMatrix64F momentumSubspace = new DenseMatrix64F(SpatialForceVector.SIZE, 3);
      momentumSubspace.set(3, 0, 1.0);
      momentumSubspace.set(4, 1, 1.0);
      momentumSubspace.set(5, 2, 1.0);

      DenseMatrix64F momentumMultipliers = new DenseMatrix64F(3, 1);
      MatrixTools.setDenseMatrixFromTuple3d(momentumMultipliers, rateOfChangeOfLinearMomentum, 0, 0);

      Vector3d angularAcceleration = RandomTools.generateRandomVector(random);
      DenseMatrix64F accelerationSubspace = new DenseMatrix64F(SpatialMotionVector.SIZE, 3);
      accelerationSubspace.set(0, 0, 1.0);
      accelerationSubspace.set(1, 1, 1.0);
      accelerationSubspace.set(2, 2, 1.0);

      DenseMatrix64F accelerationMultipliers = new DenseMatrix64F(3, 1);
      MatrixTools.setDenseMatrixFromTuple3d(accelerationMultipliers, angularAcceleration, 0, 0);

      solver.solve(accelerationSubspace, accelerationMultipliers, momentumSubspace, momentumMultipliers);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame);
      solver.getRateOfChangeOfMomentum(desiredMomentumRate);

      JUnitTools.assertTuple3dEquals(rateOfChangeOfLinearMomentum, desiredMomentumRate.getLinearPartCopy(), 1e-9);

      SpatialAccelerationVector pelvisAcceleration = new SpatialAccelerationVector();
      rootJoint.packDesiredJointAcceleration(pelvisAcceleration);
      JUnitTools.assertTuple3dEquals(angularAcceleration, pelvisAcceleration.getAngularPartCopy(), 1e-9);

      checkTaskSpaceAcceleration(rootJoint, twistCalculator, jacobian, spatialAcceleration, 1e-9, false);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);

      // solve again using the desiredMomentumRate, check if we also get the right pelvis acceleration back again
      solver.solve(desiredMomentumRate);
      checkTaskSpaceAcceleration(rootJoint, twistCalculator, jacobian, spatialAcceleration, 1e-9, false);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);

      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, oneDoFJoints, DT, desiredMomentumRate, 1e-4);
   }

   @Test
   public void testAngularAcceleration()
   {
      Random random = new Random(2534L);
      Vector3d[] jointAxes = new Vector3d[] {X, Y, Z};

      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, jointAxes);

      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();
      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);

      setRandomPositionsAndVelocities(random, randomFloatingChain.getElevator(), sixDoFJoints, randomFloatingChain.getRevoluteJoints());

      RigidBody elevator = randomFloatingChain.getElevator();
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      MomentumSolver solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, randomFloatingChain.getRevoluteJoints(), DT,
                                 centerOfMassFrame, twistCalculator);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();
      RigidBody base = rootJoint.getSuccessor();
      RigidBody endEffector = revoluteJoints.get(revoluteJoints.size() - 1).getSuccessor();
      MechanismGeometricJacobian jacobian = new MechanismGeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());
      FrameVector endEffectorAngularAcceleration = new FrameVector(rootJoint.getFrameAfterJoint(), RandomTools.generateRandomVector(random));
      DenseMatrix64F nullspaceMultiplier = new DenseMatrix64F(0, 1);

      solver.setDesiredAngularAcceleration(jacobian, elevator.getBodyFixedFrame(), endEffectorAngularAcceleration, nullspaceMultiplier);

      twistCalculator.compute();
      solver.compute();
      solver.solve(desiredMomentumRate);

      SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(jacobian.getEndEffector().getBodyFixedFrame(),
                                                         elevator.getBodyFixedFrame(), endEffectorAngularAcceleration.getReferenceFrame());
      spatialAcceleration.setAngularPart(endEffectorAngularAcceleration.getVector());

      checkTaskSpaceAcceleration(rootJoint, twistCalculator, jacobian, spatialAcceleration, 1e-9, true);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-4);
   }

   private MomentumSolver createAndInitializeMomentumOptimizer(RigidBody elevator, SixDoFJoint rootJoint, List<SixDoFJoint> sixDoFJoints,
           List<RevoluteJoint> joints, double dt, ReferenceFrame centerOfMassFrame, TwistCalculator twistCalculator)
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");

//    DampedLeastSquaresSolver jacobianSolver = new DampedLeastSquaresSolver(SpatialMotionVector.SIZE);
//    jacobianSolver.setAlpha(0.0);
      LinearSolver<DenseMatrix64F> jacobianSolver = LinearSolverFactory.linear(SpatialMotionVector.SIZE);
      MomentumSolver solver = new MomentumSolver(rootJoint, elevator, centerOfMassFrame, twistCalculator, jacobianSolver, dt, registry);
      solver.initialize();

      for (SixDoFJoint sixDoFJoint : sixDoFJoints)
      {
         ScrewTestTools.integrateVelocities(sixDoFJoint, dt);
      }


      ScrewTestTools.integrateVelocities(joints, dt);
      elevator.updateFramesRecursively();
      twistCalculator.compute();

      return solver;
   }

   private static void checkJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration, double epsilon)
   {
      DenseMatrix64F check = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
      joint.packDesiredAccelerationMatrix(check, 0);
      JUnitTools.assertMatrixEquals(jointAcceleration, check, epsilon);
   }

   private static void checkTaskSpaceAcceleration(SixDoFJoint rootJoint, TwistCalculator twistCalculator, MechanismGeometricJacobian jacobian,
           SpatialAccelerationVector spatialAcceleration, double epsilon, boolean angularPartOnly)
   {
      RigidBody elevator = rootJoint.getPredecessor();
      ReferenceFrame rootFrame = elevator.getBodyFixedFrame();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootFrame, rootFrame, rootFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, rootFrame, rootAcceleration, twistCalculator,
                                                                       true, true);
      spatialAccelerationCalculator.compute();

      SpatialAccelerationVector checkAcceleration = new SpatialAccelerationVector();

      spatialAccelerationCalculator.packRelativeAcceleration(checkAcceleration, elevator, jacobian.getEndEffector());

      Twist twistOfCurrentWithRespectToNew = new Twist();
      Twist twistOfBodyWithRespectToBase = new Twist();
      twistCalculator.packRelativeTwist(twistOfCurrentWithRespectToNew, jacobian.getBase(), jacobian.getEndEffector());
      twistOfCurrentWithRespectToNew.changeBaseFrameNoRelativeTwist(rootJoint.getFrameAfterJoint());

      twistCalculator.packTwistOfBody(twistOfBodyWithRespectToBase, jacobian.getEndEffector());
      checkAcceleration.changeFrame(spatialAcceleration.getExpressedInFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);

      if (angularPartOnly)
      {
         assertEquals(spatialAcceleration.getBodyFrame(), checkAcceleration.getBodyFrame());
         assertEquals(spatialAcceleration.getBaseFrame(), checkAcceleration.getBaseFrame());
         assertEquals(spatialAcceleration.getExpressedInFrame(), checkAcceleration.getExpressedInFrame());
         JUnitTools.assertTuple3dEquals(spatialAcceleration.getAngularPartCopy(), checkAcceleration.getAngularPartCopy(), epsilon);
      }
      else
      {
         JUnitTools.assertSpatialMotionVectorEquals(spatialAcceleration, checkAcceleration, epsilon);
      }
   }

   private void checkInternalAcceleration(SixDoFJoint rootJoint, TwistCalculator twistCalculator, MechanismGeometricJacobian jacobian,
           SpatialAccelerationVector spatialAcceleration, double epsilon)
   {
      RigidBody elevator = rootJoint.getPredecessor();
      ReferenceFrame rootFrame = elevator.getBodyFixedFrame();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootFrame, rootFrame, rootFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, rootFrame, rootAcceleration, twistCalculator,
                                                                       true, true);
      spatialAccelerationCalculator.compute();

      SpatialAccelerationVector checkAcceleration = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packRelativeAcceleration(checkAcceleration, jacobian.getBase(), jacobian.getEndEffector());

      JUnitTools.assertSpatialMotionVectorEquals(spatialAcceleration, checkAcceleration, epsilon);
   }

   private static void checkAgainstNumericalDifferentiation(SixDoFJoint rootJoint, List<SixDoFJoint> sixDoFJoints, List<RevoluteJoint> joints, double dt,
           SpatialForceVector desiredMomentumRate, double epsilon)
   {
      ReferenceFrame referenceFrame = desiredMomentumRate.getExpressedInFrame();

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

      JUnitTools.assertSpatialForceVectorEquals(desiredMomentumRate, momentumRateNumerical, epsilon);
   }

   private static void checkAgainstInverseDynamicsCalculator(SixDoFJoint rootJoint, SpatialForceVector desiredMomentumRate, double epsilonInverseDynamics)
   {
      ReferenceFrame referenceFrame = desiredMomentumRate.getExpressedInFrame();

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getPredecessor());
      twistCalculator.compute();

      InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, 0.0);
      inverseDynamicsCalculator.compute();
      Wrench rootJointWrench = new Wrench();
      rootJoint.packWrench(rootJointWrench);
      rootJointWrench.changeFrame(referenceFrame);

      JUnitTools.assertSpatialForceVectorEquals(desiredMomentumRate, rootJointWrench, epsilonInverseDynamics);
   }

   private static void setRandomPositionsAndVelocities(Random random, RigidBody elevator, List<SixDoFJoint> sixDoFJoints,
           List<? extends OneDoFJoint> oneDoFJoints)
   {
      ScrewTestTools.setRandomPositions(oneDoFJoints, random);
      ScrewTestTools.setRandomVelocities(oneDoFJoints, random);

      for (SixDoFJoint sixDoFJoint : sixDoFJoints)
      {
         ScrewTestTools.setRandomPositionAndOrientation(sixDoFJoint, random);
         ScrewTestTools.setRandomVelocity(sixDoFJoint, random);
      }

      elevator.updateFramesRecursively();
   }
}
