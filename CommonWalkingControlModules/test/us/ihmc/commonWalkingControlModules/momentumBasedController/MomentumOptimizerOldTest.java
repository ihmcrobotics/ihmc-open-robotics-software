package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.utilities.MemoryTools;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.MomentumCalculator;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.utilities.test.JUnitTools;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class MomentumOptimizerOldTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

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
   public void testFloatingChain()
   {
      Random random = new Random(44345L);

      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
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

      doChecks(random, elevator, rootJoint, joints);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testFloatingTree()
   {
      Random random = new Random(44345L);

      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
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

      doChecks(random, elevator, rootJoint, joints);
   }

   private void doChecks(Random random, RigidBody elevator, SixDoFJoint rootJoint, ArrayList<RevoluteJoint> joints)
   {
      double dt = 1e-8;
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();

      FrameVector desiredAngularCentroidalMomentumRate = new FrameVector(centerOfMassFrame, RandomTools.generateRandomVector(random));
      FrameVector desiredLinearCentroidalMomentumRate = new FrameVector(centerOfMassFrame, RandomTools.generateRandomVector(random));
      YoVariableRegistry registry = new YoVariableRegistry("test");
      MomentumOptimizerOld optimizer = new BasicMomentumOptimizerOld(rootJoint, elevator, centerOfMassFrame, dt, registry);

      initializeOptimizer(elevator, rootJoint, joints, dt, desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate, optimizer);
      checkAgainstInverseDynamicsCalculator(rootJoint, desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, joints, dt, desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate, 1e-5);
   }


   public static void initializeOptimizer(RigidBody elevator, SixDoFJoint rootJoint, List<RevoluteJoint> joints, double dt,
         FrameVector desiredAngularCentroidalMomentumRate, FrameVector desiredLinearCentroidalMomentumRate, MomentumOptimizerOld optimizer)
   {
      optimizer.initialize();
      ScrewTestTools.integrateVelocities(rootJoint, dt);
      ScrewTestTools.integrateVelocities(joints, dt);
      elevator.updateFramesRecursively();
      optimizer.solveForRootJointAcceleration(desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate);
   }

   public static void checkAgainstNumericalDifferentiation(SixDoFJoint rootJoint, List<RevoluteJoint> joints, double dt,
           FrameVector desiredAngularCentroidalMomentumRate, FrameVector desiredLinearCentroidalMomentumRate, double epsilon)
   {
      desiredAngularCentroidalMomentumRate.checkReferenceFrameMatch(desiredLinearCentroidalMomentumRate.getReferenceFrame());
      ReferenceFrame referenceFrame = desiredAngularCentroidalMomentumRate.getReferenceFrame();

      TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), rootJoint.getPredecessor());
      MomentumCalculator momentumCalculator = new MomentumCalculator(twistCalculator);
      Momentum momentum0 = new Momentum(ReferenceFrame.getWorldFrame());
      twistCalculator.compute();
      momentumCalculator.computeAndPack(momentum0);

      ScrewTestTools.copyDesiredAccelerationToActual(rootJoint);
      ScrewTestTools.copyDesiredAccelerationsToActual(joints);

      ScrewTestTools.integrateAccelerations(rootJoint, dt);
      ScrewTestTools.integrateAccelerations(joints, dt);

      ScrewTestTools.integrateVelocities(rootJoint, dt);
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

   public static void checkAgainstInverseDynamicsCalculator(SixDoFJoint rootJoint, FrameVector desiredAngularCentroidalMomentumRate,
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

   private static final class BasicMomentumOptimizerOld extends MomentumOptimizerOld
   {
      private final SixDoFJoint rootJoint;
      private final DenseMatrix64F sixDoFJointAccelerations = new DenseMatrix64F(6, 1);

      private BasicMomentumOptimizerOld(SixDoFJoint rootJoint, RigidBody elevator, ReferenceFrame centerOfMassFrame, double controlDT,
                                        YoVariableRegistry parentRegistry)
      {
         super(elevator, centerOfMassFrame, controlDT, parentRegistry);
         this.rootJoint = rootJoint;
      }

      protected void updateBeforeSolving(double[] x)
      {
         updateAtStartOfFcn(x);
      }

      protected void updateAtStartOfFcn(double[] x)
      {
         MatrixTools.setMatrixFromOneBasedArray(sixDoFJointAccelerations, x);
         rootJoint.setDesiredAcceleration(sixDoFJointAccelerations, 0);
      }
   }
}
