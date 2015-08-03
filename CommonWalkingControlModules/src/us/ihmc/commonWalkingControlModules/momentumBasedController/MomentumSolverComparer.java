package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.ops.RandomMatrices;

import us.ihmc.utilities.RandomTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.CenterOfMassReferenceFrame;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class MomentumSolverComparer
{
   private static final int nTests = 100000;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static double testMomentumSolver()
   {
      Random random = new Random(44345L);

      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      SixDoFJoint rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
      RigidBody rootBody = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();

      ScrewTestTools.createRandomTreeRobot(joints, rootBody, 25, random);
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);

      ScrewTestTools.setRandomPositionAndOrientation(rootJoint, random);
      ScrewTestTools.setRandomVelocity(rootJoint, random);
      ScrewTestTools.setRandomPositions(joints, random);
      ScrewTestTools.setRandomVelocities(joints, random);
      elevator.updateFramesRecursively();

      double dt = 1e-8;
      centerOfMassFrame.update();
      MomentumSolverInterface solver = createAndInitializeMomentumSolver(elevator, rootJoint, joints, dt, centerOfMassFrame);

      long startNanos = System.nanoTime();

      Map<InverseDynamicsJoint, DenseMatrix64F> jointSpaceAccelerations = new LinkedHashMap<InverseDynamicsJoint, DenseMatrix64F>();
      for (RevoluteJoint joint : joints)
      {
         DenseMatrix64F jointSpaceAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         jointSpaceAccelerations.put(joint, jointSpaceAcceleration);
      }

      for (int i = 0; i < nTests; i++)
      {
         SpatialForceVector desiredMomentumRate = new SpatialForceVector(centerOfMassFrame, RandomTools.generateRandomVector(random), RandomTools.generateRandomVector(random));
         for (RevoluteJoint joint : joints)
         {
            DenseMatrix64F jointAcceleration = jointSpaceAccelerations.get(joint);
            RandomMatrices.setRandom(jointAcceleration, -1.0, 1.0, random);
            solver.setDesiredJointAcceleration(joint, jointAcceleration);
         }
         solver.compute();
         solver.solve(desiredMomentumRate);
      }

      long stopNanos = System.nanoTime();

      return (stopNanos - startNanos) * 1e-9 / nTests;
   }

   private static MomentumSolverInterface createAndInitializeMomentumSolver(RigidBody elevator, SixDoFJoint rootJoint, ArrayList<RevoluteJoint> joints, double dt,
           ReferenceFrame centerOfMassFrame)
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      MomentumSolverInterface solver = new MomentumSolver(rootJoint, elevator, centerOfMassFrame, twistCalculator, LinearSolverFactory.linear(SpatialMotionVector.SIZE),
                                 dt, registry);
      solver.initialize();
      ScrewTestTools.integrateVelocities(rootJoint, dt);
      ScrewTestTools.integrateVelocities(joints, dt);
      elevator.updateFramesRecursively();
      twistCalculator.compute();

      return solver;
   }

   public static double testMomentumOptimizer()
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

      double dt = 1e-8;
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();
      MomentumOptimizerOld optimizer = createAndInitializeMomentumOptimizer(elevator, rootJoint, joints, dt, centerOfMassFrame);

      long startNanos = System.nanoTime();

      for (int i = 0; i < nTests; i++)
      {
         FrameVector desiredAngularCentroidalMomentumRate = new FrameVector(centerOfMassFrame, RandomTools.generateRandomVector(random));
         FrameVector desiredLinearCentroidalMomentumRate = new FrameVector(centerOfMassFrame, RandomTools.generateRandomVector(random));

         optimizer.solveForRootJointAcceleration(desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate);
      }

      long stopNanos = System.nanoTime();

      return (stopNanos - startNanos) * 1e-9 / nTests;
   }

   private static MomentumOptimizerOld createAndInitializeMomentumOptimizer(RigidBody elevator, SixDoFJoint rootJoint, ArrayList<RevoluteJoint> joints, double dt,
           ReferenceFrame centerOfMassFrame)
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      MomentumOptimizerOld optimizer = new BasicMomentumOptimizerOld(rootJoint, elevator, centerOfMassFrame, dt, registry);
      optimizer.initialize();
      ScrewTestTools.integrateVelocities(rootJoint, dt);
      ScrewTestTools.integrateVelocities(joints, dt);
      elevator.updateFramesRecursively();

      return optimizer;
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


   public static void main(String[] args)
   {
      double solverTime = testMomentumSolver();
      double optimizerTime = testMomentumOptimizer();

      System.out.println("solverTime = " + solverTime);
      System.out.println("optimizerTime = " + optimizerTime);
   }
}
