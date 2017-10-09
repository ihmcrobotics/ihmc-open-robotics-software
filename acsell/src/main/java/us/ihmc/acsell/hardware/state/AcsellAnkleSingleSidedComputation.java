package us.ihmc.acsell.hardware.state;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.acsell.hardware.configuration.AcsellAnklePhysicalParameters;

public class AcsellAnkleSingleSidedComputation
{
   private double sx, sy, sq, cx, cy, cq;
   private double q;
   private final double Kz, Lr;
   private final double Rx, Ry, Rz;
   private final double Px, Py, Pz;
   private double b, bdot;
   
   private final DenseMatrix64F P0 = new DenseMatrix64F(3,1);
   private final DenseMatrix64F R0 = new DenseMatrix64F(3,1);
   private final DenseMatrix64F P = new DenseMatrix64F(3,1);
   private final DenseMatrix64F R = new DenseMatrix64F(3,1);
   private final DenseMatrix64F PR = new DenseMatrix64F(3,1);
   private final DenseMatrix64F PRT = new DenseMatrix64F(1,3);
   private final DenseMatrix64F Rmat = new DenseMatrix64F(3,3);
   private final DenseMatrix64F Pmat = new DenseMatrix64F(3,3);
   private final DenseMatrix64F dPdth = new DenseMatrix64F(3,2);
   private final DenseMatrix64F dRdq = new DenseMatrix64F(3,1);
   private final DenseMatrix64F Jrow = new DenseMatrix64F(1,2);
   private final DenseMatrix64F N2PR = new DenseMatrix64F(1,1);
   private final DenseMatrix64F sc1 = new DenseMatrix64F(3,1);
   private final DenseMatrix64F sc1dot = new DenseMatrix64F(3,1);
   
   private final DenseMatrix64F a = new DenseMatrix64F(1,2);
   private final DenseMatrix64F Pmatdot = new DenseMatrix64F(3,3);
   private final DenseMatrix64F Pdot = new DenseMatrix64F(3,1);
   private final DenseMatrix64F Rdot = new DenseMatrix64F(3,1);
   private final DenseMatrix64F PRdot = new DenseMatrix64F(3,1);
   private final DenseMatrix64F PRTdot = new DenseMatrix64F(1,3);
   private final DenseMatrix64F dPdthdot = new DenseMatrix64F(3,2);
   private final DenseMatrix64F dRdqdot = new DenseMatrix64F(3,1);
   private final DenseMatrix64F Jdotrow = new DenseMatrix64F(1,2);
   //create some temporary matrices to help with some matrix functions
   //these are created now as finals to help the garbage collector
   private final DenseMatrix64F temp1by1 = new DenseMatrix64F(1,1);
   private final DenseMatrix64F temp1by1_2 = new DenseMatrix64F(1,1);
   private final DenseMatrix64F temp1by2 = new DenseMatrix64F(1,2);
   
   public AcsellAnkleSingleSidedComputation(AcsellAnklePhysicalParameters parameters)
   {
      Kz = parameters.getKz();      //Kz = 0.317437;
      Lr = parameters.getLr();
      P0.set(3,1,true,parameters.getP0());
      R0.set(3,1,true,parameters.getR0());
      
      Rx = R0.get(0);
      Ry = R0.get(1);
      Rz = R0.get(2);
      
      Px = P0.get(0);
      Py = P0.get(1);
      Pz = P0.get(2);
      
      Rmat.set(3,3,true,new double[] {(Rz-Kz), Rx , 0, 0, 0, Ry, -Rx , (Rz-Kz), Kz});
   }
   
   public void update(double sin_x, double sin_y, double cos_x, double cos_y)
   {
      this.sx = sin_x;
      this.sy = sin_y;
      this.cx = cos_x;
      this.cy = cos_y;
      
      Pmat.set(3,3,true,new double[] {cy, sx*sy, cx*sy, 0, cx, -sx, -sy, sx*cy, cx*cy});
      CommonOps.mult(Pmat, P0, P);
      
      computePulley();
      computeJacobianRow();
   }
   
   private void computePulley()
   {
      double A = (2*Kz*Px*cy - 2*Kz*Rx - 2*Px*Rz*cy - 2*Px*Rx*sy + 2*Pz*Rx*cx*cy + 2*Kz*Pz*cx*sy + 2*Py*Rx*cy*sx - 2*Pz*Rz*cx*sy + 2*Kz*Py*sx*sy - 2*Py*Rz*sx*sy);
      double B = (2*Kz*Rz - 2*Kz*Kz - 2*Px*Rx*cy - 2*Kz*Px*sy + 2*Px*Rz*sy + 2*Kz*Pz*cx*cy - 2*Pz*Rz*cx*cy + 2*Kz*Py*cy*sx - 2*Pz*Rx*cx*sy - 2*Py*Rz*cy*sx - 2*Py*Rx*sx*sy);
      //     C = Lr^2 -  (2*Kz^2  + Rx^2  + Ry^2  + Rz^2  + Px^2*cy^2   + Py^2*cx^2   + Px^2*sy^2   + Pz^2*sx^2   - 2*Kz*Rz - 2*Py*Ry*cx + 2*Kz*Px*sy + 2*Pz*Ry*sx + Pz^2*cx^2*cy^2    + Py^2*cy^2*sx^2    + Pz^2*cx^2*sy^2    + Py^2*sx^2*sy^2    - 2*Kz*Pz*cx*cy - 2*Kz*Py*cy*sx - 2*Py*Pz*cx*sx + 2*Py*Pz*cx*cy^2*sx  + 2*Py*Pz*cx*sx*sy^2);
      double C = Lr*Lr - (2*Kz*Kz + Rx*Rx + Ry*Ry + Rz*Rz + Px*Px*cy*cy + Py*Py*cx*cx + Px*Px*sy*sy + Pz*Pz*sx*sx - 2*Kz*Rz - 2*Py*Ry*cx + 2*Kz*Px*sy + 2*Pz*Ry*sx + Pz*Pz*cx*cx*cy*cy + Py*Py*cy*cy*sx*sx + Pz*Pz*cx*cx*sy*sy + Py*Py*sx*sx*sy*sy - 2*Kz*Pz*cx*cy - 2*Kz*Py*cy*sx - 2*Py*Pz*cx*sx + 2*Py*Pz*cx*cy*cy*sx + 2*Py*Pz*cx*sx*sy*sy);

      double discriminant = A*A+B*B-C*C;
      cq = (discriminant>=0) ? (C*B+Math.abs(A)*Math.sqrt(discriminant))/(A*A+B*B) : C*B/(A*A+B*B);
      sq = (C-B*cq)/A;
      q = Math.atan2(sq,cq);
      sc1.set(3,1,true,new double[] {sq, cq, 1});
      CommonOps.mult(Rmat, sc1, R);
      
      CommonOps.subtract(R, P, PR);
      CommonOps.transpose(PR,PRT);
      CommonOps.mult(PRT, PR, N2PR);
      double Lr2Calc = N2PR.get(0);
      if (Math.abs(Lr-Math.sqrt(Lr2Calc))>1e-3)
          ;//System.err.println("Contstraint equation violation: " + Lr + " " + Math.sqrt(Lr2Calc));
   }
      
   private void computeJacobianRow()
   {   
      dPdth.set(0, 0, Py*cx*sy-Pz*sx*sy);
      dPdth.set(0, 1, -Px*sy+Py*sx*cy+Pz*cx*cy);
      dPdth.set(1, 0, -Py*sx-Pz*cx);
      dPdth.set(2, 0, Py*cx*cy-Pz*sx*cy);
      dPdth.set(2, 1, -Px*cy-Py*sx*sy-Pz*cx*sy);
      
      dRdq.set(0, -Rx*sq + (Rz-Kz)*cq);
      dRdq.set(2, -Rx*cq - (Rz-Kz)*sq);

      CommonOps.mult(PRT, dPdth, a);
      CommonOps.mult(PRT, dRdq, temp1by1);
      b = temp1by1.get(0);
      CommonOps.divide(a, b, Jrow);
   }
   
   public void computeJacobianTimeDerivativeRow(double xdot, double ydot, double qdot)
   {   
      dPdthdot.set(0, 0, Py*(cx*cy*ydot-sx*sy*xdot)-Pz*(cx*sy*xdot+sx*cy*ydot));
      dPdthdot.set(0, 1, -Px*cy*ydot+Py*(cx*cy*xdot-sx*sy*ydot)-Pz*(sx*cy*xdot+cx*sy*ydot));
      dPdthdot.set(1, 0, (-Py*cx+Pz*sx)*xdot);
      dPdthdot.set(2, 0, -Py*(sx*cy*xdot+cx*sy*ydot)-Pz*(cx*cy*xdot-sx*sy*ydot));
      dPdthdot.set(2, 1, Px*sy*ydot-Py*(cx*sy*xdot+sx*cy*ydot)-Pz*(cx*cy*ydot-sx*sy*xdot));
      
      dRdqdot.set(0, -(Rx*cq + (Rz-Kz)*sq)*qdot);
      dRdqdot.set(2, (Rx*sq - (Rz-Kz)*cq)*qdot);
      
      sc1dot.set(3,1,true,new double[] {cq*qdot, -sq*qdot, 0});
      CommonOps.mult(Rmat, sc1dot, Rdot);
      
      Pmatdot.set(3,3,true,new double[] {-sy*ydot, cx*sy*xdot+sx*cy*ydot, cx*cy*ydot-sx*sy*xdot, 0, -sx*xdot, -cx*xdot, -cy*ydot, cx*cy*xdot-sx*sy*ydot, -(sx*cy*xdot+cx*sy*xdot)});
      CommonOps.mult(Pmatdot, P0, Pdot);
      
      CommonOps.subtract(Rdot, Pdot, PRdot);
      CommonOps.transpose(PRdot,PRTdot);
      
      CommonOps.mult(PRTdot, dRdq, temp1by1);
      CommonOps.mult(PRT, dRdqdot, temp1by1_2);
      CommonOps.addEquals(temp1by1, temp1by1_2);
      bdot = temp1by1.get(0);
      
      CommonOps.mult(PRTdot, dPdth, temp1by2);
      CommonOps.mult(PRT, dPdthdot, Jdotrow);
      CommonOps.addEquals(Jdotrow,temp1by2); //adot = PRTdot*dPdth + PRT*dPdthdot
      
      CommonOps.scale(b, Jdotrow);
      CommonOps.scale(bdot, a, temp1by2);
      CommonOps.subtractEquals(Jdotrow, temp1by2);
      CommonOps.divide(Jdotrow, bdot*bdot); //Jdotrow = (adot*b-a*bdot)/bdot^2
   }
   
   public double getPulleyAngle()
   {
      return q;
   }
   
   public void getJacobianRow(int row, DenseMatrix64F J)
   {
      J.set(row, 0, Jrow.get(0));
      J.set(row, 1, Jrow.get(1));      
   }
   
   public void getJacobianTimeDerivativeRow(int row, DenseMatrix64F Jdot)
   {
      Jdot.set(row, 0, Jdotrow.get(0));
      Jdot.set(row, 1, Jdotrow.get(1));      
   }
   
}

