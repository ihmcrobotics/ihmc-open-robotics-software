package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.Random;

import javax.media.j3d.Transform3D;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.CenterOfMassReferenceFrame;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTestTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class MomentumSolverComparer
{
   private static final int nTests = 100000;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static double testMomentumSolver()
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

      double dt = 1e-8;
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();
      MomentumSolver solver = createAndInitializeMomentumSolver(elevator, rootJoint, joints, dt, centerOfMassFrame);

      long startNanos = System.nanoTime();

      for (int i = 0; i < nTests; i++)
      {
         FrameVector desiredAngularCentroidalMomentumRate = new FrameVector(centerOfMassFrame, RandomTools.getRandomVector(random));
         FrameVector desiredLinearCentroidalMomentumRate = new FrameVector(centerOfMassFrame, RandomTools.getRandomVector(random));
         solver.solveForRootJointAcceleration(desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate);
      }

      long stopNanos = System.nanoTime();

      return (stopNanos - startNanos) * 1e-9 / nTests;
   }

   private static MomentumSolver createAndInitializeMomentumSolver(RigidBody elevator, SixDoFJoint rootJoint, ArrayList<RevoluteJoint> joints, double dt,
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

   public static double testMomentumOptimizer()
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

      double dt = 1e-8;
      ReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();
      MomentumOptimizer optimizer = createAndInitializeMomentumOptimizer(elevator, rootJoint, joints, dt, centerOfMassFrame);

      long startNanos = System.nanoTime();

      for (int i = 0; i < nTests; i++)
      {
         FrameVector desiredAngularCentroidalMomentumRate = new FrameVector(centerOfMassFrame, RandomTools.getRandomVector(random));
         FrameVector desiredLinearCentroidalMomentumRate = new FrameVector(centerOfMassFrame, RandomTools.getRandomVector(random));

         optimizer.solveForRootJointAcceleration(desiredAngularCentroidalMomentumRate, desiredLinearCentroidalMomentumRate);
      }

      long stopNanos = System.nanoTime();

      return (stopNanos - startNanos) * 1e-9 / nTests;
   }

   private static MomentumOptimizer createAndInitializeMomentumOptimizer(RigidBody elevator, SixDoFJoint rootJoint, ArrayList<RevoluteJoint> joints, double dt,
           ReferenceFrame centerOfMassFrame)
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      MomentumOptimizer optimizer = new BasicMomentumOptimizer(rootJoint, elevator, centerOfMassFrame, dt, registry);
      optimizer.initialize();
      ScrewTestTools.integrateVelocities(rootJoint, dt);
      ScrewTestTools.integrateVelocities(joints, dt);
      elevator.updateFramesRecursively();

      return optimizer;
   }


   private static final class BasicMomentumOptimizer extends MomentumOptimizer
   {
      private final SixDoFJoint rootJoint;
      private final DenseMatrix64F sixDoFJointAccelerations = new DenseMatrix64F(6, 1);

      private BasicMomentumOptimizer(SixDoFJoint rootJoint, RigidBody elevator, ReferenceFrame centerOfMassFrame, double controlDT,
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
