package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.ejml.ops.RandomMatrices;
import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.linearAlgebra.NullspaceCalculator;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.MomentumCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.SpatialForceVectorTest;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVectorTest;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.test.JUnitTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;


//TODO: tests where MomentumSolver is called multiple times in a row
public class MomentumSolverTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double DT = 1e-8;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testFloatingChainWithJointSpaceConstraints()
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
      MomentumSolverInterface solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, revoluteJoints, DT, centerOfMassFrame,
                                          twistCalculator);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));

      Map<InverseDynamicsJoint, DenseMatrix64F> jointAccelerations = new LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F>();
      for (RevoluteJoint joint : revoluteJoints)
      {
         DenseMatrix64F jointSpaceAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         RandomMatrices.setRandom(jointSpaceAcceleration, -1.0, 1.0, random);
         solver.setDesiredJointAcceleration(joint, jointSpaceAcceleration);
         jointAccelerations.put(joint, jointSpaceAcceleration);
      }

      solver.compute();
      solver.solve(desiredMomentumRate);

      for (RevoluteJoint joint : revoluteJoints)
      {
         checkJointAcceleration(joint, jointAccelerations.get(joint), 1e-6);
      }

      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-4);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testFloatingTreeWithJointSpaceConstraints()
   {
      Random random = new Random(44345L);

      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
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
      MomentumSolverInterface solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, revoluteJoints, DT, centerOfMassFrame,
                                          twistCalculator);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));

      Map<InverseDynamicsJoint, DenseMatrix64F> jointAccelerations = new LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F>();
      for (RevoluteJoint joint : revoluteJoints)
      {
         DenseMatrix64F jointSpaceAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         RandomMatrices.setRandom(jointSpaceAcceleration, -1.0, 1.0, random);
         solver.setDesiredJointAcceleration(joint, jointSpaceAcceleration);
         jointAccelerations.put(joint, jointSpaceAcceleration);
      }

      solver.compute();
      solver.solve(desiredMomentumRate);

      for (RevoluteJoint joint : revoluteJoints)
      {
         checkJointAcceleration(joint, jointAccelerations.get(joint), 1e-6);
      }

      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-4);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testTwoFloatingBodiesWithTaskSpaceAcceleration()
   {
      Random random = new Random(12342L);
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      SixDoFJoint rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
      RigidBody firstBody = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

      SixDoFJoint sixDoFJointBetweenFirstAndSecondBody = new SixDoFJoint("secondFloatingJoint", firstBody, firstBody.getBodyFixedFrame());
      RigidBody secondBody = ScrewTestTools.addRandomRigidBody("secondBody", random, sixDoFJointBetweenFirstAndSecondBody);

      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);
      sixDoFJoints.add(sixDoFJointBetweenFirstAndSecondBody);
      ArrayList<RevoluteJoint> oneDoFJoints = new ArrayList<RevoluteJoint>();

      setRandomPositionsAndVelocities(random, elevator, sixDoFJoints, oneDoFJoints);

      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      MomentumSolverInterface solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, oneDoFJoints, DT, centerOfMassFrame,
                                          twistCalculator);

      // TODO: I'm really confused here. Shouldn't the Jacobian and the Spatial Acceleration have the same frames?
      GeometricJacobian jacobian = new GeometricJacobian(firstBody, secondBody, rootJoint.getFrameAfterJoint());

//    GeometricJacobian jacobian = new GeometricJacobian(elevator, secondBody, secondBody.getBodyFixedFrame());

//    SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(jacobian.getEndEffectorFrame(), jacobian.getBaseFrame(),
//                                                       jacobian.getEndEffectorFrame(), RandomTools.generateRandomVector(random),
//                                                       RandomTools.generateRandomVector(random));

      SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(jacobian.getEndEffectorFrame(), elevatorFrame,
                                                         jacobian.getEndEffectorFrame(), RandomTools.generateRandomVector(random),
                                                         RandomTools.generateRandomVector(random));

      DenseMatrix64F nullspaceMultiplier = new DenseMatrix64F(0, 1);

//    InverseDynamicsJoint[] contrainedJoints = extractJoints(jacobian.getJointsInOrder(), 1);
      InverseDynamicsJoint[] contrainedJoints = jacobian.getJointsInOrder();

      TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
      taskspaceConstraintData.set(spatialAcceleration, nullspaceMultiplier);

      solver.setDesiredSpatialAcceleration(contrainedJoints, jacobian, taskspaceConstraintData);

      solver.compute();

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));
      solver.solve(desiredMomentumRate);

      checkTaskSpaceAcceleration(rootJoint, twistCalculator, jacobian, spatialAcceleration, 1e-9, false);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, new ArrayList<RevoluteJoint>(), DT, desiredMomentumRate, 1e-4);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
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

//    randomFloatingChain.getRevoluteJoints().get(0).setQd(0.0);

      RigidBody elevator = randomFloatingChain.getElevator();
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      MomentumSolverInterface solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, randomFloatingChain.getRevoluteJoints(), DT,
                                          centerOfMassFrame, twistCalculator);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();
      RevoluteJoint firstJoint = revoluteJoints.get(0);
      RigidBody base = firstJoint.getSuccessor();
      RigidBody endEffector = revoluteJoints.get(revoluteJoints.size() - 1).getSuccessor();
      GeometricJacobian jacobian = new GeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());
      SpatialAccelerationVector taskSpaceAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), elevator.getBodyFixedFrame(),
                                                           endEffector.getBodyFixedFrame(), RandomTools.generateRandomVector(random),
                                                           RandomTools.generateRandomVector(random));
      DenseMatrix64F nullspaceMultiplier = new DenseMatrix64F(0, 1);

      TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
      taskspaceConstraintData.set(taskSpaceAcceleration, nullspaceMultiplier);
      solver.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);

      DenseMatrix64F firstJointAcceleration = new DenseMatrix64F(1, 1);
      firstJointAcceleration.set(0, 0, random.nextDouble());
      solver.setDesiredJointAcceleration(firstJoint, firstJointAcceleration);

      solver.compute();
      solver.solve(desiredMomentumRate);

      checkJointAcceleration(firstJoint, firstJointAcceleration, 1e-9);
      checkTaskSpaceAcceleration(rootJoint, twistCalculator, jacobian, taskSpaceAcceleration, 1e-9, false);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-4);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
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
      MomentumSolverInterface solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, randomFloatingChain.getRevoluteJoints(), DT,
                                          centerOfMassFrame, twistCalculator);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();
      RigidBody base = rootJoint.getSuccessor();
      RigidBody endEffector = revoluteJoints.get(revoluteJoints.size() - 1).getSuccessor();
      GeometricJacobian jacobian = new GeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());
      SpatialAccelerationVector internalAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), base.getBodyFixedFrame(),
                                                          endEffector.getBodyFixedFrame(), RandomTools.generateRandomVector(random),
                                                          RandomTools.generateRandomVector(random));
      DenseMatrix64F nullspaceMultiplier = new DenseMatrix64F(0, 1);

      TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
      taskspaceConstraintData.set(internalAcceleration, nullspaceMultiplier);
      solver.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);

      solver.compute();
      solver.solve(desiredMomentumRate);

      checkInternalAcceleration(rootJoint, twistCalculator, jacobian, internalAcceleration, 1e-9);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-4);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testTwoInternalSpatialAccelerations()
   {
      Random random = new Random(44345L);
      Vector3d[] jointAxes = new Vector3d[]
      {
         X, Y, X, Z, X, Y, X, Y,
         Z    // , X, Y, Z
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
      MomentumSolverInterface solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, randomFloatingChain.getRevoluteJoints(), DT,
                                          centerOfMassFrame, twistCalculator);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();
      RigidBody base = rootJoint.getSuccessor();

      RigidBody endEffectorOne = revoluteJoints.get(6 - 1).getSuccessor();
      GeometricJacobian jacobianOne = new GeometricJacobian(base, endEffectorOne, endEffectorOne.getBodyFixedFrame());
      SpatialAccelerationVector internalAccelerationOne = new SpatialAccelerationVector(endEffectorOne.getBodyFixedFrame(), base.getBodyFixedFrame(),
                                                             endEffectorOne.getBodyFixedFrame(), RandomTools.generateRandomVector(random),
                                                             RandomTools.generateRandomVector(random));
      DenseMatrix64F nullspaceMultiplierOne = new DenseMatrix64F(0, 1);

      TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();

      taskspaceConstraintData.set(internalAccelerationOne, nullspaceMultiplierOne);
      solver.setDesiredSpatialAcceleration(jacobianOne, taskspaceConstraintData);

      RigidBody endEffectorTwo = revoluteJoints.get(revoluteJoints.size() - 1).getSuccessor();
      GeometricJacobian jacobianTwo = new GeometricJacobian(endEffectorOne, endEffectorTwo, endEffectorOne.getBodyFixedFrame());
      SpatialAccelerationVector internalAccelerationTwo = new SpatialAccelerationVector(endEffectorTwo.getBodyFixedFrame(), endEffectorOne.getBodyFixedFrame(),
                                                             endEffectorTwo.getBodyFixedFrame(), RandomTools.generateRandomVector(random),
                                                             RandomTools.generateRandomVector(random));
      DenseMatrix64F nullspaceMultiplierTwo = new DenseMatrix64F(0, 1);

      // Constrain x, y, z acceleration of the very end with respect to the world.
      DenseMatrix64F selectionMatrixTwo = new DenseMatrix64F(3, SpatialMotionVector.SIZE);
      selectionMatrixTwo.set(0, 3, 1.0);
      selectionMatrixTwo.set(1, 4, 1.0);
      selectionMatrixTwo.set(1, 5, 1.0);
      selectionMatrixTwo.set(2, 5, 1.0);

      taskspaceConstraintData.set(internalAccelerationTwo, nullspaceMultiplierTwo, selectionMatrixTwo);

      solver.setDesiredSpatialAcceleration(jacobianTwo, taskspaceConstraintData);

      solver.compute();
      solver.solve(desiredMomentumRate);

      checkInternalAcceleration(rootJoint, twistCalculator, jacobianOne, internalAccelerationOne, 1e-9);
      checkInternalAcceleration(rootJoint, twistCalculator, jacobianTwo, internalAccelerationTwo, selectionMatrixTwo, 1e-9);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-4);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-3);
   }


   @Ignore    // Not working yet. We need to revisit momentum solving, add cases like this, clean it up, optimize it, and make it pretty

	@EstimatedDuration
	@Test(timeout=300000)
   public void testTwoInternalOverlappingSpatialAccelerations()
   {
      // This is the chicken and the egg test with the GFE Robot. We want to
      // do both chest orientation control and head orientation control.
      // However, the constrained joints of one contraint are a function
      // of the unconstrained joints of the other constraint and vice versa.

      Random random = new Random(44345L);
      Vector3d[] jointAxes = new Vector3d[]
      {
         Z, Y, X,
         Y    // back_bkz, back_bky, back_bkx, neck_ry
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
      MomentumSolverInterface solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, randomFloatingChain.getRevoluteJoints(), DT,
                                          centerOfMassFrame, twistCalculator);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();
      RigidBody baseRigidBody = rootJoint.getPredecessor();

      InverseDynamicsJoint back_bkz = revoluteJoints.get(0);
      InverseDynamicsJoint back_bky = revoluteJoints.get(1);
      InverseDynamicsJoint back_bkx = revoluteJoints.get(2);
      InverseDynamicsJoint neck_ry = revoluteJoints.get(3);

      RigidBody pelvisRigidBody = back_bkz.getPredecessor();
      RigidBody chestRigidBody = neck_ry.getPredecessor();
      RigidBody headRigidBody = neck_ry.getSuccessor();

      // Constrain pitch and roll acceleration of the chest with respect to the pelvis,
      // using back_mdy and back_bkx

      GeometricJacobian pelvisToChestJacobian = new GeometricJacobian(pelvisRigidBody, chestRigidBody, chestRigidBody.getBodyFixedFrame());
      SpatialAccelerationVector pelvisToChestInternalAcceleration = new SpatialAccelerationVector(chestRigidBody.getBodyFixedFrame(),
                                                                       pelvisRigidBody.getBodyFixedFrame(), chestRigidBody.getBodyFixedFrame(),
                                                                       RandomTools.generateRandomVector(random), RandomTools.generateRandomVector(random));
      DenseMatrix64F pelvisToChestNullspaceMultiplier = new DenseMatrix64F(0, 1);

      DenseMatrix64F pelvisToChestSelectionMatrix = new DenseMatrix64F(2, SpatialMotionVector.SIZE);
      pelvisToChestSelectionMatrix.set(0, 0, 1.0);    // wx (roll of chest)
      pelvisToChestSelectionMatrix.set(1, 1, 1.0);    // wy (pitch of chest)


      InverseDynamicsJoint[] pelvisToChestConstrainedJoints = new InverseDynamicsJoint[] {back_bky, back_bkx};

      TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
      taskspaceConstraintData.set(pelvisToChestInternalAcceleration, pelvisToChestNullspaceMultiplier, pelvisToChestSelectionMatrix);

      solver.setDesiredSpatialAcceleration(pelvisToChestConstrainedJoints, pelvisToChestJacobian, taskspaceConstraintData);

      // Constrain pitch and yaw acceleration of the head with respect to the elevator
      // using neck_ry and back_bkz
      GeometricJacobian elevatorToHeadJacobian = new GeometricJacobian(pelvisRigidBody, headRigidBody, headRigidBody.getBodyFixedFrame());
      SpatialAccelerationVector elevatorToHeadInternalAcceleration = new SpatialAccelerationVector(headRigidBody.getBodyFixedFrame(),
                                                                        baseRigidBody.getBodyFixedFrame(), headRigidBody.getBodyFixedFrame(),
                                                                        RandomTools.generateRandomVector(random), RandomTools.generateRandomVector(random));
      DenseMatrix64F elevatorToHeadNullspaceMultiplier = new DenseMatrix64F(0, 1);

      DenseMatrix64F elevatorToHeadSelectionMatrix = new DenseMatrix64F(2, SpatialMotionVector.SIZE);
      elevatorToHeadSelectionMatrix.set(0, 1, 1.0);    // wy (pitch of head)
      elevatorToHeadSelectionMatrix.set(1, 2, 1.0);    // wz (yaw of head)

      InverseDynamicsJoint[] elevatorToHeadConstrainedJoints = new InverseDynamicsJoint[] {back_bkz, neck_ry};

      taskspaceConstraintData.set(elevatorToHeadInternalAcceleration, elevatorToHeadNullspaceMultiplier, elevatorToHeadSelectionMatrix);
      solver.setDesiredSpatialAcceleration(elevatorToHeadConstrainedJoints, elevatorToHeadJacobian, taskspaceConstraintData);

      solver.compute();
      solver.solve(desiredMomentumRate);

      checkInternalAcceleration(rootJoint, twistCalculator, pelvisToChestJacobian, pelvisToChestInternalAcceleration, pelvisToChestSelectionMatrix, 1e-9);
      checkInternalAcceleration(rootJoint, twistCalculator, elevatorToHeadJacobian, elevatorToHeadInternalAcceleration, elevatorToHeadSelectionMatrix, 1e-9);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-4);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-3);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testSubspaces()
   {
      Random random = new Random(12342L);
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
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
      MomentumSolverInterface solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, oneDoFJoints, DT, centerOfMassFrame,
                                          twistCalculator);

      GeometricJacobian jacobian = new GeometricJacobian(body0, secondBody, rootJoint.getFrameAfterJoint());

      SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(jacobian.getEndEffectorFrame(), elevatorFrame,
                                                         jacobian.getEndEffectorFrame(), RandomTools.generateRandomVector(random),
                                                         RandomTools.generateRandomVector(random));

      DenseMatrix64F nullspaceMultiplier = new DenseMatrix64F(0, 1);
      TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
      taskspaceConstraintData.set(spatialAcceleration, nullspaceMultiplier);
      solver.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);

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
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-4);

      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, oneDoFJoints, DT, desiredMomentumRate, 1e-4);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
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
      MomentumSolverInterface solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, randomFloatingChain.getRevoluteJoints(), DT,
                                          centerOfMassFrame, twistCalculator);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();
      RigidBody base = rootJoint.getSuccessor();
      RigidBody endEffector = revoluteJoints.get(revoluteJoints.size() - 1).getSuccessor();
      GeometricJacobian jacobian = new GeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());
      FrameVector endEffectorAngularAcceleration = new FrameVector(endEffector.getBodyFixedFrame(), RandomTools.generateRandomVector(random));
      DenseMatrix64F nullspaceMultiplier = new DenseMatrix64F(0, 1);

      TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
      taskspaceConstraintData.setAngularAcceleration(jacobian.getEndEffector().getBodyFixedFrame(), elevator.getBodyFixedFrame(),
              endEffectorAngularAcceleration, nullspaceMultiplier);

      solver.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);

      solver.compute();
      solver.solve(desiredMomentumRate);

      SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(jacobian.getEndEffector().getBodyFixedFrame(),
                                                         elevator.getBodyFixedFrame(), endEffectorAngularAcceleration.getReferenceFrame());
      spatialAcceleration.setAngularPart(endEffectorAngularAcceleration.getVector());

      checkTaskSpaceAcceleration(rootJoint, twistCalculator, jacobian, spatialAcceleration, 1e-9, true);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-4);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testNullspaceMultipliers()
   {
      Random random = new Random(2534L);
      Vector3d[] jointAxes = new Vector3d[]
      {
         X, Y, Z, Y, Y, X
      };
      RandomFloatingChain randomFloatingChain = new RandomFloatingChain(random, jointAxes);

      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();
      ArrayList<SixDoFJoint> sixDoFJoints = new ArrayList<SixDoFJoint>();
      sixDoFJoints.add(rootJoint);

      List<RevoluteJoint> revoluteJoints = randomFloatingChain.getRevoluteJoints();
      setRandomPositionsAndVelocities(random, randomFloatingChain.getElevator(), sixDoFJoints, revoluteJoints);

      int kneeIndex = 3;
      RevoluteJoint kneeJoint = revoluteJoints.get(kneeIndex);
      kneeJoint.setQ(0.0);
      randomFloatingChain.getElevator().updateFramesRecursively();

      RigidBody elevator = randomFloatingChain.getElevator();
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);

//    DampedLeastSquaresSolver jacobianSolver = new DampedLeastSquaresSolver(SpatialMotionVector.SIZE);
//    jacobianSolver.setAlpha(5e-2);
      LinearSolver<DenseMatrix64F> jacobianSolver = LinearSolverFactory.pseudoInverse(true);
      MomentumSolverInterface solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, revoluteJoints, DT, centerOfMassFrame,
                                          twistCalculator, jacobianSolver);

      SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random),
                                                  RandomTools.generateRandomVector(random));

      RigidBody base = rootJoint.getSuccessor();
      RigidBody endEffector = revoluteJoints.get(revoluteJoints.size() - 1).getSuccessor();
      GeometricJacobian jacobian = new GeometricJacobian(base, endEffector, endEffector.getBodyFixedFrame());

      jacobian.compute();

//    assertTrue(jacobian.det() < 1e-9);

      SpatialAccelerationVector taskSpaceAcceleration = new SpatialAccelerationVector(endEffector.getBodyFixedFrame(), elevator.getBodyFixedFrame(),
                                                           endEffector.getBodyFixedFrame(), RandomTools.generateRandomVector(random),
                                                           RandomTools.generateRandomVector(random));
      int nullity = 1;
      DenseMatrix64F nullspaceMultipliers = new DenseMatrix64F(nullity, 1);
      CommonOps.fill(nullspaceMultipliers, 0.0);
      RandomMatrices.setRandom(nullspaceMultipliers, random);

      TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
      taskspaceConstraintData.set(taskSpaceAcceleration, nullspaceMultipliers);
      solver.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);

      solver.compute();
      solver.solve(desiredMomentumRate);

      NullspaceCalculator nullspaceCalculator = new NullspaceCalculator(jacobian.getNumberOfColumns(), true);
      nullspaceCalculator.setMatrix(jacobian.getJacobianMatrix(), nullity);
      DenseMatrix64F nullspace = nullspaceCalculator.getNullspace();

      InverseDynamicsJoint[] jointList = ScrewTools.computeSubtreeJoints(base);
      DenseMatrix64F vdotTaskSpace = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointList), 1);
      ScrewTools.packDesiredJointAccelerationsMatrix(jointList, vdotTaskSpace);
      DenseMatrix64F nullspaceMultiplierCheck = new DenseMatrix64F(nullspace.getNumCols(), vdotTaskSpace.getNumCols());
      CommonOps.multTransA(nullspace, vdotTaskSpace, nullspaceMultiplierCheck);
      CommonOps.subtractEquals(nullspaceMultiplierCheck, nullspaceMultipliers);

      assertTrue(MatrixFeatures.isConstantVal(nullspaceMultiplierCheck, 0.0, 1e-7));

//    checkTaskSpaceAcceleration(rootJoint, twistCalculator, jacobian, taskSpaceAcceleration, 1e-9, false);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, sixDoFJoints, revoluteJoints, DT, desiredMomentumRate, 1e-4);
   }

   private MomentumSolverInterface createAndInitializeMomentumOptimizer(RigidBody elevator, SixDoFJoint rootJoint, List<SixDoFJoint> sixDoFJoints,
           List<RevoluteJoint> joints, double dt, ReferenceFrame centerOfMassFrame, TwistCalculator twistCalculator)
   {
      LinearSolver<DenseMatrix64F> jacobianSolver = LinearSolverFactory.linear(SpatialMotionVector.SIZE);

      return createAndInitializeMomentumOptimizer(elevator, rootJoint, sixDoFJoints, joints, dt, centerOfMassFrame, twistCalculator, jacobianSolver);
   }

   private MomentumSolverInterface createAndInitializeMomentumOptimizer(RigidBody elevator, SixDoFJoint rootJoint, List<SixDoFJoint> sixDoFJoints,
           List<RevoluteJoint> joints, double dt, ReferenceFrame centerOfMassFrame, TwistCalculator twistCalculator,
           LinearSolver<DenseMatrix64F> jacobianSolver)
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");

      MomentumSolverInterface solver = new MomentumSolver3(rootJoint, elevator, centerOfMassFrame, twistCalculator, jacobianSolver, dt, registry);
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

   private static void checkTaskSpaceAcceleration(SixDoFJoint rootJoint, TwistCalculator twistCalculator, GeometricJacobian jacobian,
           SpatialAccelerationVector spatialAcceleration, double epsilon, boolean angularPartOnly)
   {
      RigidBody elevator = rootJoint.getPredecessor();
      SpatialAccelerationCalculator spatialAccelerationCalculator = createSpatialAccelerationCalculator(twistCalculator, elevator);
      spatialAccelerationCalculator.compute();

      SpatialAccelerationVector checkAcceleration = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packRelativeAcceleration(checkAcceleration, elevator, jacobian.getEndEffector());

      Twist twistOfCurrentWithRespectToNew = new Twist();
      Twist twistOfBodyWithRespectToBase = new Twist();
      twistCalculator.packRelativeTwist(twistOfCurrentWithRespectToNew, jacobian.getBase(), jacobian.getEndEffector());
      twistOfCurrentWithRespectToNew.changeBaseFrameNoRelativeTwist(rootJoint.getFrameAfterJoint());

      twistCalculator.packTwistOfBody(twistOfBodyWithRespectToBase, jacobian.getEndEffector());

//    checkAcceleration.changeFrame(spatialAcceleration.getExpressedInFrame(), twistOfCurrentWithRespectToNew, twistOfBodyWithRespectToBase);

      if (angularPartOnly)
      {
         assertEquals(spatialAcceleration.getBodyFrame(), checkAcceleration.getBodyFrame());
         assertEquals(spatialAcceleration.getBaseFrame(), checkAcceleration.getBaseFrame());
         assertEquals(spatialAcceleration.getExpressedInFrame(), checkAcceleration.getExpressedInFrame());
         JUnitTools.assertTuple3dEquals(spatialAcceleration.getAngularPartCopy(), checkAcceleration.getAngularPartCopy(), epsilon);
      }
      else
      {
         SpatialMotionVectorTest.assertSpatialMotionVectorEquals(spatialAcceleration, checkAcceleration, epsilon);
      }
   }


   private static SpatialAccelerationCalculator createSpatialAccelerationCalculator(TwistCalculator twistCalculator, RigidBody elevator)
   {
      ReferenceFrame rootFrame = elevator.getBodyFixedFrame();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootFrame, rootFrame, rootFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, rootFrame, rootAcceleration, twistCalculator,
                                                                       true, true);

      return spatialAccelerationCalculator;
   }

   private void checkInternalAcceleration(SixDoFJoint rootJoint, TwistCalculator twistCalculator, GeometricJacobian jacobian,
           SpatialAccelerationVector spatialAcceleration, double epsilon)
   {
      checkInternalAcceleration(rootJoint, twistCalculator, jacobian, spatialAcceleration, null, epsilon);
   }

   private void checkInternalAcceleration(SixDoFJoint rootJoint, TwistCalculator twistCalculator, GeometricJacobian jacobian,
           SpatialAccelerationVector spatialAcceleration, DenseMatrix64F selectionMatrix, double epsilon)
   {
      RigidBody elevator = rootJoint.getPredecessor();
      SpatialAccelerationCalculator spatialAccelerationCalculator = createSpatialAccelerationCalculator(twistCalculator, elevator);
      spatialAccelerationCalculator.compute();

      SpatialAccelerationVector checkAcceleration = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packRelativeAcceleration(checkAcceleration, jacobian.getBase(), jacobian.getEndEffector());

      SpatialMotionVectorTest.assertSpatialMotionVectorEquals(spatialAcceleration, checkAcceleration, selectionMatrix, epsilon);
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

      SpatialForceVectorTest.assertSpatialForceVectorEquals(desiredMomentumRate, momentumRateNumerical, epsilon);
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

      SpatialForceVectorTest.assertSpatialForceVectorEquals(desiredMomentumRate, rootJointWrench, epsilonInverseDynamics);
   }

   private static void setRandomPositionsAndVelocities(Random random, RigidBody elevator, List<SixDoFJoint> sixDoFJoints,
           List<? extends OneDoFJoint> oneDoFJoints)
   {
      for (SixDoFJoint sixDoFJoint : sixDoFJoints)
      {
         ScrewTestTools.setRandomPositionAndOrientation(sixDoFJoint, random);
         ScrewTestTools.setRandomVelocity(sixDoFJoint, random);
      }

      ScrewTestTools.setRandomPositions(oneDoFJoints, random);
      ScrewTestTools.setRandomVelocities(oneDoFJoints, random);

      elevator.updateFramesRecursively();
   }
}
