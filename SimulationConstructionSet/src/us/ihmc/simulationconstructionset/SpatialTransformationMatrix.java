package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;

public final class SpatialTransformationMatrix implements java.io.Serializable
{
   private static final long serialVersionUID = -1437839453250728870L;
   private final RotationMatrix R = new RotationMatrix();
   private final Matrix3D r_Twidle = new Matrix3D();

   public SpatialTransformationMatrix()
   {
   }

   @Override
   public String toString()
   {
      String ret = "R = " + R + "\n" + "r_Twidle = " + r_Twidle;

      return ret;
   }

   public void setFromOffsetAndRotation(Vector3D r_i, RotationMatrix Rin)
   {
      R.set(Rin);

      // r.set(r_i);

      r_Twidle.setM00(0.0);
      r_Twidle.setM01(-r_i.getZ());
      r_Twidle.setM02(r_i.getY());
      r_Twidle.setM10(r_i.getZ());
      r_Twidle.setM11(0.0);
      r_Twidle.setM12(-r_i.getX());
      r_Twidle.setM20(-r_i.getY());
      r_Twidle.setM21(r_i.getX());
      r_Twidle.setM22(0.0);

      /*
       * r_Twidle.setElement(0,0, 0.0) ; r_Twidle.setElement(0,1, -r_i.z);
       * r_Twidle.setElement(0,2, r_i.y); r_Twidle.setElement(1,0, r_i.z) ;
       * r_Twidle.setElement(1,1, 0.0) ; r_Twidle.setElement(1,2, -r_i.x);
       * r_Twidle.setElement(2,0, -r_i.y); r_Twidle.setElement(2,1, r_i.x) ;
       * r_Twidle.setElement(2,2, 0.0);
       */
   }

   private final Vector3D temp1 = new Vector3D();

   public void set(SpatialTransformationMatrix Min)
   {
      R.set(Min.R);
      r_Twidle.set(Min.r_Twidle);

      // r.set(Min.r);

   }

   private final Matrix3D x_Twidle = new Matrix3D();

   public void invert()
   {
      Rt.set(R);
      Rt.transpose();

      x_Twidle.set(Rt);
      x_Twidle.multiply(r_Twidle);
      x_Twidle.multiply(R);
      x_Twidle.scale(-1.0);

      R.set(Rt);
      r_Twidle.set(x_Twidle);
   }

   public void transform(SpatialVector v1)
   {
      R.transform(v1.top);
      temp1.set(v1.top);
      r_Twidle.transform(temp1);
      temp1.scale(-1.0);

      R.transform(v1.bottom);
      v1.bottom.add(temp1);
   }

   private final RotationMatrix Rt = new RotationMatrix();
   private final Matrix3D R_IA_Rt = new Matrix3D(), R_IB_Rt = new Matrix3D(), R_IC_Rt = new Matrix3D();
   private final Matrix3D R_IB_Rt_r_Twidle = new Matrix3D(), r_Twidle_R_IA_Rt = new Matrix3D(), r_Twidle_R_IB_Rt = new Matrix3D(),
         r_Twidle_R_IB_Rt_r_Twidle = new Matrix3D();

   public void transformSpatialInertia(SpatialInertiaMatrix I)
   {
      // Rt is R transpose;
      Rt.set(R);
      Rt.transpose();

      R_IA_Rt.set(R);
      R_IA_Rt.multiply(I.A);
      R_IA_Rt.multiply(Rt);
      R_IB_Rt.set(R);
      R_IB_Rt.multiply(I.B);
      R_IB_Rt.multiply(Rt);
      R_IC_Rt.set(R);
      R_IC_Rt.multiply(I.C);
      R_IC_Rt.multiply(Rt);

      R_IB_Rt_r_Twidle.set(R_IB_Rt);
      R_IB_Rt_r_Twidle.multiply(r_Twidle);

      r_Twidle_R_IB_Rt.set(r_Twidle);
      r_Twidle_R_IB_Rt.multiply(R_IB_Rt);
      r_Twidle_R_IA_Rt.set(r_Twidle);
      r_Twidle_R_IA_Rt.multiply(R_IA_Rt);
      r_Twidle_R_IB_Rt_r_Twidle.set(r_Twidle);
      r_Twidle_R_IB_Rt_r_Twidle.multiply(R_IB_Rt_r_Twidle);

      I.A.add(R_IA_Rt, R_IB_Rt_r_Twidle);
      I.B.set(R_IB_Rt);

      I.C.sub(R_IC_Rt, r_Twidle_R_IB_Rt_r_Twidle);
      I.C.sub(r_Twidle_R_IA_Rt);
      r_Twidle_R_IA_Rt.transpose();
      I.C.sub(r_Twidle_R_IA_Rt);

      I.D.set(I.A);
      I.D.transpose();
   }

   /*
    * private Matrix3d r_twidle_RIA = new Matrix3d(), r_twidle_RIB = new
    * Matrix3d(); private Matrix3d IA_temp = new Matrix3d(), IB_temp = new
    * Matrix3d(), IC_temp = new Matrix3d(), ID_temp = new Matrix3d(); public
    * void multiplySpatialInertia(SpatialInertiaMatrix I) {
    * IA_temp.set(I.A);I.A.set(R);I.A.multiply(IA_temp);
    * IB_temp.set(I.B);I.B.set(R);I.B.multiply(IB_temp);
    * IC_temp.set(I.C);I.C.set(R);I.C.multiply(IC_temp);
    * ID_temp.set(I.D);I.D.set(R);I.D.multiply(ID_temp); r_twidle_RIA.set(r_Twidle);
    * r_twidle_RIA.multiply(I.A); I.C.sub(r_twidle_RIA); r_twidle_RIB.set(r_Twidle);
    * r_twidle_RIB.multiply(I.B); I.D.sub(r_twidle_RIB); }
    */

   /*
    * private Matrix3d r_twidle_R = new Matrix3d(), IB_r_twidle_R = new
    * Matrix3d(), ID_r_twidle_R = new Matrix3d(); private Matrix3d IA_R = new
    * Matrix3d(), IB_R = new Matrix3d(), IC_R = new Matrix3d(), ID_R = new
    * Matrix3d(); public void postMultiplySpatialInertia(SpatialInertiaMatrix I)
    * { I.A.multiply(R); I.C.multiply(R); r_twidle_R.set(r_Twidle); r_twidle_R.multiply(R);
    * IB_r_twidle_R.set(I.B); IB_r_twidle_R.multiply(r_twidle_R);
    * I.A.sub(IB_r_twidle_R); ID_r_twidle_R.set(I.D);
    * ID_r_twidle_R.multiply(r_twidle_R); I.C.sub(ID_r_twidle_R); I.B.multiply(R);
    * I.D.multiply(R); }
    */

   /*
    * private Vector3d Rtop = new Vector3d(), Rbot = new Vector3d(),
    * r_Twidle_Rtop = new Vector3d(); public void
    * transformSpatialZeroAccel(SpatialVector Z) { Rtop.set(Z.top);
    * R.transform(Rtop); Rbot.set(Z.bottom); R.transform(Rbot);
    * r_Twidle_Rtop.set(Rtop); r_Twidle.transform(r_Twidle_Rtop);
    * Z.top.set(Rtop); Z.bottom.sub(Rbot, r_Twidle_Rtop); }
    */

}
