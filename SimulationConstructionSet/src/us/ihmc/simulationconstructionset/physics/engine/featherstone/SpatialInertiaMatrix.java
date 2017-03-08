package us.ihmc.simulationconstructionset.physics.engine.featherstone;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.SpatialVector;
import us.ihmc.simulationconstructionset.mathfunctions.Matrix;

/**
 * Title:        Yobotics! Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms<p>
 * Copyright:    Copyright (c) Jerry Pratt<p>
 * Company:      Yobotics, Inc. <p>
 * @author Jerry Pratt
 * @version Beta 1.0
 */



public final class SpatialInertiaMatrix implements java.io.Serializable
{
   /**
    *
    */
   private static final long serialVersionUID = 1765931620079414651L;
   public Matrix3D A = new Matrix3D(), B = new Matrix3D(), C = new Matrix3D(), D = new Matrix3D();

   public SpatialInertiaMatrix()
   {
   }

   private Vector3D IA_s_top = new Vector3D(), IB_s_bottom = new Vector3D(), IC_s_top = new Vector3D(), ID_s_bottom = new Vector3D();

   public double sIs(SpatialVector s)
   {
      IA_s_top.set(s.top);
      A.transform(IA_s_top);
      IB_s_bottom.set(s.bottom);
      B.transform(IB_s_bottom);
      IC_s_top.set(s.top);
      C.transform(IC_s_top);
      ID_s_bottom.set(s.bottom);
      D.transform(ID_s_bottom);

      return s.bottom.dot(IA_s_top) + s.bottom.dot(IB_s_bottom) + s.top.dot(IC_s_top) + s.top.dot(ID_s_bottom);


   }

   @Override
   public String toString()
   {
      return (A.toString() + B.toString() + C.toString() + D.toString() + "\n");


   }

   public void multiply(SpatialVector sV)
   {
      Atop.set(sV.top);
      A.transform(Atop);
      Bbot.set(sV.bottom);
      B.transform(Bbot);
      Ctop.set(sV.top);
      C.transform(Ctop);
      Dbot.set(sV.bottom);
      D.transform(Dbot);

      sV.top.add(Atop, Bbot);
      sV.bottom.add(Ctop, Dbot);

   }


   public void sub(SpatialInertiaMatrix M2)
   {
      A.sub(M2.A);
      B.sub(M2.B);
      C.sub(M2.C);
      D.sub(M2.D);
   }


   public void add(SpatialInertiaMatrix M2)
   {
      A.add(M2.A);
      B.add(M2.B);
      C.add(M2.C);
      D.add(M2.D);
   }

   public void add(SpatialInertiaMatrix M1, SpatialInertiaMatrix M2)
   {
      A.add(M1.A, M2.A);
      B.add(M1.B, M2.B);
      C.add(M1.C, M2.C);
      D.add(M1.D, M2.D);
   }

   public void sub(SpatialInertiaMatrix M1, SpatialInertiaMatrix M2)
   {
      A.sub(M1.A, M2.A);
      B.sub(M1.B, M2.B);
      C.sub(M1.C, M2.C);
      D.sub(M1.D, M2.D);
   }


   public void getMatrix(Matrix M)
   {
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            M.set(i, j, A.getElement(i, j));
            M.set(i, j + 3, B.getElement(i, j));
            M.set(i + 3, j, C.getElement(i, j));
            M.set(i + 3, j + 3, D.getElement(i, j));
         }
      }

   }

   public void getPlanarXYMatrix(Matrix M)
   {
      M.set(0, 0, A.getElement(0, 2));
      M.set(0, 1, B.getElement(0, 0));
      M.set(0, 2, B.getElement(0, 1));
      M.set(1, 0, A.getElement(1, 2));
      M.set(1, 1, B.getElement(1, 0));
      M.set(1, 2, B.getElement(1, 1));
      M.set(2, 0, C.getElement(2, 2));
      M.set(2, 1, D.getElement(2, 0));
      M.set(2, 2, D.getElement(2, 1));
   }

   public void getPlanarXZMatrix(Matrix M)
   {
      M.set(0, 0, A.getElement(0, 1));
      M.set(0, 1, B.getElement(0, 0));
      M.set(0, 2, B.getElement(0, 2));
      M.set(1, 0, A.getElement(2, 1));
      M.set(1, 1, B.getElement(2, 0));
      M.set(1, 2, B.getElement(2, 2));
      M.set(2, 0, C.getElement(1, 1));
      M.set(2, 1, D.getElement(1, 0));
      M.set(2, 2, D.getElement(1, 2));
   }

   public void getPlanarYZMatrix(Matrix M)
   {
      M.set(0, 0, A.getElement(1, 0));
      M.set(0, 1, B.getElement(1, 1));
      M.set(0, 2, B.getElement(1, 2));
      M.set(1, 0, A.getElement(2, 0));
      M.set(1, 1, B.getElement(2, 1));
      M.set(1, 2, B.getElement(2, 2));
      M.set(2, 0, C.getElement(0, 0));
      M.set(2, 1, D.getElement(0, 1));
      M.set(2, 2, D.getElement(0, 2));
   }


   /*
    * public void setInitArticulatedInertia(double mass, double Ixx, double Iyy, double Izz)
    * {
    * A.setElement(0,0, 0.0); A.setElement(0,1, 0.0); A.setElement(0,2, 0.0);     B.setElement(0,0, mass); B.setElement(0,1, 0.0 ); B.setElement(0,2, 0.0);
    * A.setElement(1,0, 0.0); A.setElement(1,1, 0.0); A.setElement(1,2, 0.0);     B.setElement(1,0, 0.0 ); B.setElement(1,1, mass); B.setElement(1,2, 0.0);
    * A.setElement(2,0, 0.0); A.setElement(2,1, 0.0); A.setElement(2,2, 0.0);     B.setElement(2,0, 0.0 ); B.setElement(2,1, 0.0 ); B.setElement(2,2, mass);
    *
    * C.setElement(0,0, Ixx); C.setElement(0,1, 0.0); C.setElement(0,2, 0.0);     D.setElement(0,0, 0.0); D.setElement(0,1, 0.0); D.setElement(0,2, 0.0);
    * C.setElement(1,0, 0.0); C.setElement(1,1, Iyy); C.setElement(1,2, 0.0);     D.setElement(1,0, 0.0); D.setElement(1,1, 0.0); D.setElement(1,2, 0.0);
    * C.setElement(2,0, 0.0); C.setElement(2,1, 0.0); C.setElement(2,2, Izz);     D.setElement(2,0, 0.0); D.setElement(2,1, 0.0); D.setElement(2,2, 0.0);
    *
    * }
    */

   public void setInitArticulatedInertia(double mass, Matrix3D Inertia)
   {
      A.setM00(0.0);
      A.setM01(0.0);
      A.setM02(0.0);
      B.setM00(mass);
      B.setM01(0.0);
      B.setM02(0.0);
      A.setM10(0.0);
      A.setM11(0.0);
      A.setM12(0.0);
      B.setM10(0.0);
      B.setM11(mass);
      B.setM12(0.0);
      A.setM20(0.0);
      A.setM21(0.0);
      A.setM22(0.0);
      B.setM20(0.0);
      B.setM21(0.0);
      B.setM22(mass);

      C.set(Inertia);
      D.setM00(0.0);
      D.setM01(0.0);
      D.setM02(0.0);
      D.setM10(0.0);
      D.setM11(0.0);
      D.setM12(0.0);
      D.setM20(0.0);
      D.setM21(0.0);
      D.setM22(0.0);


      /*
       * A.setElement(0,0, 0.0); A.setElement(0,1, 0.0); A.setElement(0,2, 0.0);     B.setElement(0,0, mass); B.setElement(0,1, 0.0 ); B.setElement(0,2, 0.0);
       * A.setElement(1,0, 0.0); A.setElement(1,1, 0.0); A.setElement(1,2, 0.0);     B.setElement(1,0, 0.0 ); B.setElement(1,1, mass); B.setElement(1,2, 0.0);
       * A.setElement(2,0, 0.0); A.setElement(2,1, 0.0); A.setElement(2,2, 0.0);     B.setElement(2,0, 0.0 ); B.setElement(2,1, 0.0 ); B.setElement(2,2, mass);
       *
       * C.set(Inertia);                                                             D.setElement(0,0, 0.0); D.setElement(0,1, 0.0); D.setElement(0,2, 0.0);
       *                                                                           D.setElement(1,0, 0.0); D.setElement(1,1, 0.0); D.setElement(1,2, 0.0);
       *                                                                           D.setElement(2,0, 0.0); D.setElement(2,1, 0.0); D.setElement(2,2, 0.0);
       */
   }

   public void set(SpatialInertiaMatrix spatialInertiaMatrix)
   {
      this.A.set(spatialInertiaMatrix.A);
      this.B.set(spatialInertiaMatrix.B);
      this.C.set(spatialInertiaMatrix.C);
      this.D.set(spatialInertiaMatrix.D);
   }

   private final RotationMatrix ONE3d = new RotationMatrix(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

   public void oneMinus()
   {
      // For Mirtich p. 141.

      A.scale(-1.0);
      B.scale(-1.0);
      C.scale(-1.0);
      D.scale(-1.0);

      A.add(ONE3d);
      D.add(ONE3d);

   }


   public void Iss_sIs(SpatialInertiaMatrix I_hat_i, SpatialVector s_hat_i, double sIs)
   {
      Atop.set(s_hat_i.top);
      I_hat_i.A.transform(Atop);
      Bbot.set(s_hat_i.bottom);
      I_hat_i.B.transform(Bbot);
      Ctop.set(s_hat_i.top);
      I_hat_i.C.transform(Ctop);
      Dbot.set(s_hat_i.bottom);
      I_hat_i.D.transform(Dbot);

      v1.add(Atop, Bbot);
      v1.scale(1.0 / sIs);

      v2.add(Ctop, Dbot);
      v2.scale(1.0 / sIs);


      setInnerMul(this.A, v1, s_hat_i.bottom);
      setInnerMul(this.B, v1, s_hat_i.top);
      setInnerMul(this.C, v2, s_hat_i.bottom);
      setInnerMul(this.D, v2, s_hat_i.top);
   }


   private Vector3D Atop = new Vector3D(), Bbot = new Vector3D(), Ctop = new Vector3D(), Dbot = new Vector3D();
   private Vector3D botA = new Vector3D(), botB = new Vector3D(), topC = new Vector3D(), topD = new Vector3D();
   private Vector3D v1 = new Vector3D(), v2 = new Vector3D(), v3 = new Vector3D(), v4 = new Vector3D();
   private Matrix3D A_trans = new Matrix3D(), B_trans = new Matrix3D(), C_trans = new Matrix3D(), D_trans = new Matrix3D();

   public void IssI(SpatialInertiaMatrix I_hat_i, SpatialVector s_hat_i, double sIs)
   {
      Atop.set(s_hat_i.top);
      I_hat_i.A.transform(Atop);
      Bbot.set(s_hat_i.bottom);
      I_hat_i.B.transform(Bbot);
      Ctop.set(s_hat_i.top);
      I_hat_i.C.transform(Ctop);
      Dbot.set(s_hat_i.bottom);
      I_hat_i.D.transform(Dbot);

      v1.add(Atop, Bbot);
      v1.scale(1.0 / sIs);

      v2.add(Ctop, Dbot);
      v2.scale(1.0 / sIs);

      A_trans.set(I_hat_i.A);
      A_trans.transpose();
      B_trans.set(I_hat_i.B);
      B_trans.transpose();
      C_trans.set(I_hat_i.C);
      C_trans.transpose();
      D_trans.set(I_hat_i.D);
      D_trans.transpose();

      botA.set(s_hat_i.bottom);
      A_trans.transform(botA);
      botB.set(s_hat_i.bottom);
      B_trans.transform(botB);
      topC.set(s_hat_i.top);
      C_trans.transform(topC);
      topD.set(s_hat_i.top);
      D_trans.transform(topD);

      v3.add(botA, topC);
      v4.add(botB, topD);

      setInnerMul(this.A, v1, v3);
      setInnerMul(this.B, v1, v4);
      setInnerMul(this.C, v2, v3);
      setInnerMul(this.D, v2, v4);

   }

   public void setInnerMul(Matrix3D M, Vector3D v1, Vector3D v2)
   {
      M.setM00(v1.getX() * v2.getX());
      M.setM01(v1.getX() * v2.getY());
      M.setM02(v1.getX() * v2.getZ());
      M.setM10(v1.getY() * v2.getX());
      M.setM11(v1.getY() * v2.getY());
      M.setM12(v1.getY() * v2.getZ());
      M.setM20(v1.getZ() * v2.getX());
      M.setM21(v1.getZ() * v2.getY());
      M.setM22(v1.getZ() * v2.getZ());
   }

}
