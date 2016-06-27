package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanAnnotations;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

public class FourBarKinematicLoopJacobianSolverTest
{
   private final double eps = 1e-8;

   @TestPlanAnnotations.DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleUnitSquare()
   {
      RevoluteJoint[] joints = new RevoluteJoint[4];
      ReferenceFrame elevatorFrame = ReferenceFrame.getWorldFrame();
      double mass = 1.0, sideLength = 1.0;
      Vector3d rotationAxis = new Vector3d(0.0, 1.0, 0.0);

      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      RigidBodyTransform elevatorToMasterTransform = new RigidBodyTransform();

      joints[0] = ScrewTools.addRevoluteJoint("masterJointA", elevator, elevatorToMasterTransform, rotationAxis);
      RigidBody masterRB = createAndAttachCylinderRB("masterRB", mass, 0.05, sideLength, joints[0]);

      joints[1] = ScrewTools.addPassiveRevoluteJoint("jointB", masterRB, new Vector3d(0.0, 0.0, -sideLength), rotationAxis, true);
      RigidBody rigidBodyBC = createAndAttachCylinderRB("jointB_RB", mass, 0.05, sideLength, joints[1]);

      joints[2] = ScrewTools.addPassiveRevoluteJoint("jointC", rigidBodyBC, new Vector3d(sideLength, 0.0, 0.0), rotationAxis, true);
      RigidBody rigidBodyCD = createAndAttachCylinderRB("jointC_RB", mass, 0.05, sideLength, joints[2]);

      joints[3] = ScrewTools.addPassiveRevoluteJoint("jointD", rigidBodyCD, new Vector3d(0.0, 0.0, sideLength), rotationAxis, true);

      FourBarKinematicLoop fourBarKinematicLoop = new FourBarKinematicLoop("fourBarIDRobot", joints[0], (PassiveRevoluteJoint) joints[1], (PassiveRevoluteJoint) joints[2],
            (PassiveRevoluteJoint) joints[3], new Vector3d(sideLength, 0.0, 0.0), true);
      FourBarCalculatorWithDerivatives calculator = new FourBarCalculatorWithDerivatives(sideLength, sideLength, sideLength, sideLength);
      FourBarKinematicLoopJacobianSolver jacobianSolver = new FourBarKinematicLoopJacobianSolver(calculator, new InverseDynamicsJoint[] {joints[0]},
            (PassiveRevoluteJoint) joints[1]);

      DenseMatrix64F jacobianToPack = new DenseMatrix64F(6, 1);

      joints[0].setQ(0.0);
      joints[0].setQd(1.0);
      fourBarKinematicLoop.update();
      jacobianSolver.computeJacobian(jacobianToPack);

      assertEquals(jacobianToPack.get(0, 0), 0.0, eps);
      assertEquals(jacobianToPack.get(1, 0), 1.0, eps);
      assertEquals(jacobianToPack.get(2, 0), 0.0, eps);
      assertEquals(jacobianToPack.get(3, 0), -1.0, eps);
      assertEquals(jacobianToPack.get(4, 0), 0.0, eps);
      assertEquals(jacobianToPack.get(5, 0), 0.0, eps);

      joints[0].setQ(- 0.25 * Math.PI);
      joints[0].setQd(1.0);
      fourBarKinematicLoop.update();

      jacobianSolver.computeJacobian(jacobianToPack);

      System.out.println("jacobian to pack = " + jacobianToPack);

      assertEquals(jacobianToPack.get(0, 0), 0.0, eps);
      assertEquals(jacobianToPack.get(1, 0), 1.0, eps);
      assertEquals(jacobianToPack.get(2, 0), 0.0, eps);
      assertEquals(jacobianToPack.get(3, 0), - Math.sqrt(0.5), eps);
      assertEquals(jacobianToPack.get(4, 0), 0.0, eps);
      assertEquals(jacobianToPack.get(5, 0), - Math.sqrt(0.5), eps);
   }

   private RigidBody createAndAttachCylinderRB(String name, double mass, double radius, double length, RevoluteJoint joint)
   {
      Matrix3d inertiaCylinder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(mass, radius, length, Axis.Z);
      Vector3d comOffsetCylinder = new Vector3d(0.0, 0.0, length / 2.0);
      return ScrewTools.addRigidBody(name, joint, inertiaCylinder, mass, comOffsetCylinder);
   }
}
