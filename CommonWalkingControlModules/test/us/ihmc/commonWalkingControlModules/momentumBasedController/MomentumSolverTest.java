package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.ejml.ops.SpecializedOps;
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
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
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
      doChecks(random, elevator, rootJoint, joints);  
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

      doChecks(random, elevator, rootJoint, joints);
   }

   private MomentumSolver createAndInitializeMomentumOptimizer(RigidBody elevator, SixDoFJoint rootJoint, ArrayList<RevoluteJoint> joints, double dt,
           ReferenceFrame centerOfMassFrame)
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      MomentumSolver solver = new MomentumSolver(rootJoint, elevator, centerOfMassFrame, dt, registry);
      solver.initialize();
      ScrewTestTools.integrateVelocities(rootJoint, dt);
      ScrewTestTools.integrateVelocities(joints, dt);
      elevator.updateFramesRecursively();

      return solver;
   }

   private void doChecks(Random random, RigidBody elevator, SixDoFJoint rootJoint, ArrayList<RevoluteJoint> joints)
   {
      double dt = 1e-8;
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();

      FrameVector desiredAngularCentroidalMomentumRate = new FrameVector(centerOfMassFrame, RandomTools.getRandomVector(random));
      FrameVector desiredLinearCentroidalMomentumRate = new FrameVector(centerOfMassFrame, RandomTools.getRandomVector(random));

      MomentumSolver solver = createAndInitializeMomentumOptimizer(elevator, rootJoint, joints, dt, centerOfMassFrame);

      Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations = new HashMap<InverseDynamicsJoint, DenseMatrix64F>();
      for (RevoluteJoint joint : joints)
      {
         DenseMatrix64F jointSpaceAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         RandomMatrices.setRandom(jointSpaceAcceleration, -1.0, 1.0, random);
         jointSpaceAccelerations.put(joint, jointSpaceAcceleration);
      }

      Map<MechanismGeometricJacobian, SpatialAccelerationVector> taskSpaceAccelerations = new HashMap<MechanismGeometricJacobian, SpatialAccelerationVector>();
      solver.solve(desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate, jointSpaceAccelerations, taskSpaceAccelerations);

      checkAgainstInverseDynamicsCalculator(rootJoint, desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate, 1e-6);
      checkAgainstNumericalDifferentiation(rootJoint, joints, dt, desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate, 1e-5);
   }

   private static void checkAgainstNumericalDifferentiation(SixDoFJoint rootJoint, ArrayList<RevoluteJoint> joints, double dt,
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
