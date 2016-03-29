package us.ihmc.robotics.screwTheory;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;

public class FourBarKinematicLoopJacobianSolver
{
   private final FourBarCalculatorWithDerivatives fourBarCalculator;
   private Vector3d jacobian;

   public FourBarKinematicLoopJacobianSolver(FourBarCalculatorWithDerivatives fourBarCalculator)
   {
      this.fourBarCalculator = fourBarCalculator;
      jacobian = new Vector3d();
   }

   public Vector3d computeJacobian(double fourBarInputJoint_q)
   {
      double ab = fourBarCalculator.getAB();
      double bc = fourBarCalculator.getBC();
      double cd = fourBarCalculator.getCD();
      double da = fourBarCalculator.getDA();

      double j11 = ab * Math.cos(fourBarInputJoint_q);
      double j21 = bc * Math.sin(fourBarInputJoint_q);
      double j31 = 0.5;

      jacobian.set(j11, j21, j31);
      return jacobian;
   }

   public void solveLinearVelFromAngularVel(Vector3d jacobian, double fourBarInputJoint_qd, Vector3d solutionToPack)
   {
      jacobian.scale(fourBarInputJoint_qd); 
      solutionToPack = jacobian;
   }
}
