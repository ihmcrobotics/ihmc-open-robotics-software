package us.ihmc.simulationconstructionset.physics.engine.featherstone;

import java.util.ArrayList;
import java.util.Collection;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointGroup;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.SpatialVector;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.mathfunctions.Matrix;

public class FloatingJointPhysics extends JointPhysics<FloatingJoint>
{
   private double[] k_qdd_x = new double[4], k_qdd_y = new double[4], k_qdd_z = new double[4];
   private double[] k_qdd_wx = new double[4], k_qdd_wy = new double[4], k_qdd_wz = new double[4];
   private double[] k_qd_x = new double[4], k_qd_y = new double[4], k_qd_z = new double[4];
   private double[] k_qd_wx = new double[4], k_qd_wy = new double[4], k_qd_wz = new double[4];
   private double[] k_qd_qs = new double[4], k_qd_qx = new double[4], k_qd_qy = new double[4], k_qd_qz = new double[4];

   private Quaternion tempOrientation2 = new Quaternion();
   private Vector3D wXr1 = new Vector3D();

   // private Matrix3d R0_i = new Matrix3d();
   private Vector3D a_hat_world_top = new Vector3D(), a_hat_world_bot = new Vector3D();
   private Matrix a_hat_matrix = new Matrix(6, 1);
   private Matrix Z_hat_matrix = new Matrix(6, 1);
   private Matrix Y_hat_matrix = new Matrix(6, 1);
   private Matrix I_hat_matrix = new Matrix(6, 6);
   private Vector3D wdXr = new Vector3D(), wXr = new Vector3D(), wXwXr = new Vector3D();
   private Vector3D delta_qd_xyz = new Vector3D();
   private final Matrix I_hat_inverse = new Matrix(6, 6);

   private double q_x_n, q_y_n, q_z_n, qd_x_n, qd_y_n, qd_z_n, q_qs_n, q_qx_n, q_qy_n, q_qz_n, qd_wx_n, qd_wy_n, qd_wz_n;

   public FloatingJointPhysics(FloatingJoint owner)
   {
      super(owner);
   }

   @Override
   protected void jointDependentChangeVelocity(double delta_qd)
   {
      System.err.println("Error!!!! FloatingJoint.jointDependentChangeVelocity should never be called!!!");
   }

   // Override featherstonePassOne since don't need to do everything...
   @Override
   public void featherstonePassOne(Vector3D w_h, Vector3D v_h, RotationMatrix Rh_0)
   {
      owner.update();
      owner.jointTransform3D.getRotation(Ri_0);

      // this.jointDependentSetAndGetRotation(Ri_0);
      Ri_0.transpose();
      Rh_i = null;
      Ri_h = null;
      d_i = null;
      u_iXd_i = null;
      r_i = null;

      // r_h = null;
      // Now do the joint dependent stuff...
      this.jointDependentFeatherstonePassOne();

      // Now update the points attached to the joint:
      R0_i.set(Ri_0);
      R0_i.transpose();

      // +++JEP OPTIMIZE
      if (groundContactPointGroups != null)
      {
         Collection<GroundContactPointGroup> groups = groundContactPointGroups.values();
         for (GroundContactPointGroup groundContactPointGroup : groups)
         {
            ArrayList<GroundContactPoint> groundContactPoints = groundContactPointGroup.getGroundContactPointsInContact();

            for (int i = 0; i < groundContactPoints.size(); i++)
            {
               GroundContactPoint point = groundContactPoints.get(i);

               point.updatePointVelocity(R0_i, owner.link.comOffset, v_i, w_i);
            }
         }
      }

      if (kinematicPoints != null)
      {
         for (int i = 0; i < kinematicPoints.size(); i++)
         {
            KinematicPoint point = kinematicPoints.get(i);

            point.updatePointVelocity(R0_i, owner.link.comOffset, v_i, w_i);
         }
      }

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = (Joint) owner.childrenJoints.get(i);

         child.physics.featherstonePassOne(w_i, v_i, Ri_0);
      }
   }

   @Override
   protected void jointDependentSetAndGetRotation(RotationMatrix Rh_i)
   {
      tempOrientation2.set(owner.q_qx.getDoubleValue(), owner.q_qy.getDoubleValue(), owner.q_qz.getDoubleValue(), owner.q_qs.getDoubleValue());
      Rh_i.set(tempOrientation2);
   }

   @Override
   protected void jointDependentFeatherstonePassOne()
   {
      // No torques:
      Q_i = 0.0;

      // Are Holding on to the velocities as part of the state:
      // But we need to rotate them into joint coordinates!!
      w_i.set(owner.qd_wx.getDoubleValue(), owner.qd_wy.getDoubleValue(), owner.qd_wz.getDoubleValue());
      v_i.set(owner.qd_x.getDoubleValue(), owner.qd_y.getDoubleValue(), owner.qd_z.getDoubleValue());

      // Ri_0.transform(w_i);  // w and wd not in world coords.  Only x,y,z are
      Ri_0.transform(v_i);

      // ///////////////////////////////
      // +++JEP 10/17/01.  Transform v_i to be at com, not the joint location...
      wXr1.cross(w_i, owner.link.comOffset);
      v_i.add(wXr1);

      // ///////////////////////////////
   }

   @Override
   protected void jointDependentSet_d_i()
   {
      System.err.println("Error!!!! FloatingJoint.jointDependentSet_d_i should never be called!!!");
   }

   @Override
   protected void jointDependentFeatherstonePassTwo(Vector3D w_h)
   {
      // Coriolis Forces:
      c_hat_i.top = null;
      c_hat_i.bottom = null;

      // Spatial Joint axis:
      s_hat_i.top = null;
      s_hat_i.bottom = null;
   }

   /*
    * protected void jointDependentComputeExternalForceR(Vector3d point_offset, Vector3d comOffset, Vector3d externalForceR)
    * {
    * externalForceR.sub(point_offset, comOffset);  // No linear motion component!!!
    * }
    */
    @Override
   public void featherstonePassThree()
   {
      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = (Joint) owner.childrenJoints.get(i);

         child.physics.featherstonePassThree(I_hat_i, Z_hat_i);
      }
   }

   @Override
   public void featherstonePassFour(SpatialVector a_hat_h, int passNumber) throws UnreasonableAccelerationException
   {
      I_hat_i.getMatrix(I_hat_matrix);

      // if (I_hat_inverse == null) I_hat_inverse = new Matrix(I_hat_matrix.getRowDimension(), I_hat_matrix.getColumnDimension());
      I_hat_matrix.inverse(I_hat_inverse);
      Z_hat_i.getMatrix(Z_hat_matrix);

      // System.out.println(Z_hat_i);
      // a_hat_matrix = I_hat_inverse.times(Z_hat_matrix);
      a_hat_matrix.timesEquals(I_hat_inverse, Z_hat_matrix);
      a_hat_i.top.set(-a_hat_matrix.get(0, 0), -a_hat_matrix.get(1, 0), -a_hat_matrix.get(2, 0));
      a_hat_i.bottom.set(-a_hat_matrix.get(3, 0), -a_hat_matrix.get(4, 0), -a_hat_matrix.get(5, 0));

      // +++JEP 10/17/01.  Transform to joint location from com location.
      // +++TK 02/06/12 THIS WAS WRONG FOR 11 YEARS! We need a_hat_i to be expressed in com frame for the rest of the dynamics algorithm.
      // Transformation to joint location should be done as post processing only (to set the qdd_* variables)
      // Now doing this same transformation on data that does not matter for the rest of the algorithm
//    wXr.cross(w_i, this.link.comOffset);
//    wXwXr.cross(w_i, wXr);
//    wdXr.cross(a_hat_i.top, this.link.comOffset);
//    a_hat_i.bottom.sub(wdXr);
//    a_hat_i.bottom.sub(wXwXr);

      // //////////////////
      // Rotate into world coords...
      R0_i.set(Ri_0);
      R0_i.transpose();
      a_hat_world_top.set(a_hat_i.top);
      a_hat_world_bot.set(a_hat_i.bottom);

      // //////////////////
      // +++JEP 10/17/01.  Transform to joint location from com location.
      // +++TK 02/06/12 See comment above.
      wXr.cross(w_i, owner.link.comOffset);
      wXwXr.cross(w_i, wXr);
      wdXr.cross(a_hat_world_top, owner.link.comOffset);
      a_hat_world_bot.sub(wdXr);
      a_hat_world_bot.sub(wXwXr);

      // R0_i.transform(a_hat_world_top);  //+++JEP w and wd should be in joint coords, not world coords.  Only x,y,z in world coords.
      R0_i.transform(a_hat_world_bot);
      owner.qdd_x.set(a_hat_world_bot.getX());
      owner.qdd_y.set(a_hat_world_bot.getY());
      owner.qdd_z.set(a_hat_world_bot.getZ());
      owner.qdd_wx.set(a_hat_world_top.getX());
      owner.qdd_wy.set(a_hat_world_top.getY());
      owner.qdd_wz.set(a_hat_world_top.getZ());
      jointDependentRecordK(passNumber);

      /*
       * k_qdd_x[passNumber] = qdd_x.getDoubleValue(); k_qdd_y[passNumber] = qdd_y.getDoubleValue(); k_qdd_z[passNumber] = qdd_z.getDoubleValue();
       * k_qd_x[passNumber] = qd_x.getDoubleValue(); k_qd_y[passNumber] = qd_y.getDoubleValue(); k_qd_z[passNumber] = qd_z.getDoubleValue();
       * k_qdd_wx[passNumber] = qdd_wx.getDoubleValue(); k_qdd_wy[passNumber] = qdd_wy.getDoubleValue(); k_qdd_wz[passNumber] = qdd_wz.getDoubleValue();
       * k_qd_wx[passNumber] = qd_wx.getDoubleValue(); k_qd_wy[passNumber] = qd_wy.getDoubleValue(); k_qd_wz[passNumber] = qd_wz.getDoubleValue();
       *
       * k_qd_qs[passNumber] = 0.5*(-q_qx.getDoubleValue() * qd_wx.getDoubleValue() - q_qy.getDoubleValue() * qd_wy.getDoubleValue() - q_qz.getDoubleValue() * qd_wz.getDoubleValue());
       * k_qd_qx[passNumber] = 0.5*(+q_qs.getDoubleValue() * qd_wx.getDoubleValue() - q_qz.getDoubleValue() * qd_wy.getDoubleValue() + q_qy.getDoubleValue() * qd_wz.getDoubleValue());
       * k_qd_qy[passNumber] = 0.5*(+q_qz.getDoubleValue() * qd_wx.getDoubleValue() + q_qs.getDoubleValue() * qd_wy.getDoubleValue() - q_qx.getDoubleValue() * qd_wz.getDoubleValue());
       * k_qd_qz[passNumber] = 0.5*(-q_qy.getDoubleValue() * qd_wx.getDoubleValue() + q_qx.getDoubleValue() * qd_wy.getDoubleValue() + q_qs.getDoubleValue() * qd_wz.getDoubleValue());
       */

      // Check for unreasonable accelerations:
      if (!jointDependentVerifyReasonableAccelerations())
      {
         ArrayList<Joint> unreasonableAccelerationJoints = new ArrayList<Joint>();
         unreasonableAccelerationJoints.add(owner);

         throw new UnreasonableAccelerationException(unreasonableAccelerationJoints);
      }

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = (Joint) owner.childrenJoints.get(i);

         child.physics.featherstonePassFour(a_hat_i, passNumber);
      }

      // System.out.println(this);
   }

   @Override
   protected void jointDependentRecordK(int passNumber)
   {
      k_qdd_x[passNumber] = owner.qdd_x.getDoubleValue();
      k_qdd_y[passNumber] = owner.qdd_y.getDoubleValue();
      k_qdd_z[passNumber] = owner.qdd_z.getDoubleValue();
      k_qd_x[passNumber] = owner.qd_x.getDoubleValue();
      k_qd_y[passNumber] = owner.qd_y.getDoubleValue();
      k_qd_z[passNumber] = owner.qd_z.getDoubleValue();
      k_qdd_wx[passNumber] = owner.qdd_wx.getDoubleValue();
      k_qdd_wy[passNumber] = owner.qdd_wy.getDoubleValue();
      k_qdd_wz[passNumber] = owner.qdd_wz.getDoubleValue();
      k_qd_wx[passNumber] = owner.qd_wx.getDoubleValue();
      k_qd_wy[passNumber] = owner.qd_wy.getDoubleValue();
      k_qd_wz[passNumber] = owner.qd_wz.getDoubleValue();
      k_qd_qs[passNumber] = 0.5
            * (-owner.q_qx.getDoubleValue() * owner.qd_wx.getDoubleValue() - owner.q_qy.getDoubleValue() * owner.qd_wy.getDoubleValue()
            - owner.q_qz.getDoubleValue() * owner.qd_wz.getDoubleValue());
      k_qd_qx[passNumber] = 0.5
            * (+owner.q_qs.getDoubleValue() * owner.qd_wx.getDoubleValue() - owner.q_qz.getDoubleValue() * owner.qd_wy.getDoubleValue()
            + owner.q_qy.getDoubleValue() * owner.qd_wz.getDoubleValue());
      k_qd_qy[passNumber] = 0.5
            * (+owner.q_qz.getDoubleValue() * owner.qd_wx.getDoubleValue() + owner.q_qs.getDoubleValue() * owner.qd_wy.getDoubleValue()
            -owner. q_qx.getDoubleValue() * owner.qd_wz.getDoubleValue());
      k_qd_qz[passNumber] = 0.5
            * (-owner.q_qy.getDoubleValue() * owner.qd_wx.getDoubleValue() + owner.q_qx.getDoubleValue() * owner.qd_wy.getDoubleValue()
            + owner.q_qs.getDoubleValue() * owner.qd_wz.getDoubleValue());
   }

   @Override
   protected void jointDependentFeatherstonePassFour(double Q, int passNumber)
   {
      // Do nothing here...
   }

   @Override
   public void recursiveEulerIntegrate(double stepSize)
   {
      owner.q_x.set(q_x_n + owner.qd_x.getDoubleValue() * stepSize);
      owner.q_y.set(q_y_n + owner.qd_y.getDoubleValue() * stepSize);
      owner.q_z.set(q_z_n + owner.qd_z.getDoubleValue() * stepSize);
      owner.qd_x.set(qd_x_n + owner.qdd_x.getDoubleValue() * stepSize);
      owner.qd_y.set(qd_y_n + owner.qdd_y.getDoubleValue() * stepSize);
      owner.qd_z.set(qd_z_n + owner.qdd_z.getDoubleValue() * stepSize);
      owner.q_qs.set(q_qs_n + 0.5 * (-q_qx_n * qd_wx_n - q_qy_n * qd_wy_n - q_qz_n * qd_wz_n) * stepSize);
      owner.q_qx.set(q_qx_n + 0.5 * (+q_qs_n * qd_wx_n - q_qz_n * qd_wy_n + q_qy_n * qd_wz_n) * stepSize);
      owner.q_qy.set(q_qy_n + 0.5 * (+q_qz_n * qd_wx_n + q_qs_n * qd_wy_n - q_qx_n * qd_wz_n) * stepSize);
      owner.q_qz.set(q_qz_n + 0.5 * (-q_qy_n * qd_wx_n + q_qx_n * qd_wy_n + q_qs_n * qd_wz_n) * stepSize);
      owner.qd_wx.set(qd_wx_n + owner.qdd_wx.getDoubleValue() * stepSize);
      owner.qd_wy.set(qd_wy_n + owner.qdd_wy.getDoubleValue() * stepSize);
      owner.qd_wz.set(qd_wz_n + owner.qdd_wz.getDoubleValue() * stepSize);

      double q_qlength = Math.sqrt(owner.q_qs.getDoubleValue() * owner.q_qs.getDoubleValue() + owner.q_qx.getDoubleValue() * owner.q_qx.getDoubleValue()
            + owner.q_qy.getDoubleValue() * owner.q_qy.getDoubleValue() + owner.q_qz.getDoubleValue() * owner.q_qz.getDoubleValue());

      owner.q_qs.set(owner.q_qs.getDoubleValue() / q_qlength);
      owner.q_qx.set(owner.q_qx.getDoubleValue() / q_qlength);
      owner.q_qy.set(owner.q_qy.getDoubleValue() / q_qlength);
      owner.q_qz.set(owner.q_qz.getDoubleValue() / q_qlength);

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = (Joint) owner.childrenJoints.get(i);

         child.physics.recursiveEulerIntegrate(stepSize);
      }
   }

   @Override
   public void recursiveRungeKuttaSum(double stepSize)
   {
      owner.q_x.set(q_x_n + stepSize * (k_qd_x[0] / 6.0 + k_qd_x[1] / 3.0 + k_qd_x[2] / 3.0 + k_qd_x[3] / 6.0));
      owner.q_y.set(q_y_n + stepSize * (k_qd_y[0] / 6.0 + k_qd_y[1] / 3.0 + k_qd_y[2] / 3.0 + k_qd_y[3] / 6.0));
      owner.q_z.set(q_z_n + stepSize * (k_qd_z[0] / 6.0 + k_qd_z[1] / 3.0 + k_qd_z[2] / 3.0 + k_qd_z[3] / 6.0));
      owner.qd_x.set(qd_x_n + stepSize * (k_qdd_x[0] / 6.0 + k_qdd_x[1] / 3.0 + k_qdd_x[2] / 3.0 + k_qdd_x[3] / 6.0));
      owner.qd_y.set(qd_y_n + stepSize * (k_qdd_y[0] / 6.0 + k_qdd_y[1] / 3.0 + k_qdd_y[2] / 3.0 + k_qdd_y[3] / 6.0));
      owner.qd_z.set(qd_z_n + stepSize * (k_qdd_z[0] / 6.0 + k_qdd_z[1] / 3.0 + k_qdd_z[2] / 3.0 + k_qdd_z[3] / 6.0));
      owner.q_qs.set(q_qs_n + stepSize * (k_qd_qs[0] / 6.0 + k_qd_qs[1] / 3.0 + k_qd_qs[2] / 3.0 + k_qd_qs[3] / 6.0));
      owner.q_qx.set(q_qx_n + stepSize * (k_qd_qx[0] / 6.0 + k_qd_qx[1] / 3.0 + k_qd_qx[2] / 3.0 + k_qd_qx[3] / 6.0));
      owner.q_qy.set(q_qy_n + stepSize * (k_qd_qy[0] / 6.0 + k_qd_qy[1] / 3.0 + k_qd_qy[2] / 3.0 + k_qd_qy[3] / 6.0));
      owner.q_qz.set(q_qz_n + stepSize * (k_qd_qz[0] / 6.0 + k_qd_qz[1] / 3.0 + k_qd_qz[2] / 3.0 + k_qd_qz[3] / 6.0));

      double q_qlength = Math.sqrt(owner.q_qs.getDoubleValue() * owner.q_qs.getDoubleValue() + owner.q_qx.getDoubleValue() * owner.q_qx.getDoubleValue()
            + owner.q_qy.getDoubleValue() * owner.q_qy.getDoubleValue() + owner.q_qz.getDoubleValue() * owner.q_qz.getDoubleValue());

      // Normalize the quaternion:
      owner.q_qs.set(owner.q_qs.getDoubleValue() / q_qlength);
      owner.q_qx.set(owner.q_qx.getDoubleValue() / q_qlength);
      owner.q_qy.set(owner.q_qy.getDoubleValue() / q_qlength);
      owner.q_qz.set(owner.q_qz.getDoubleValue() / q_qlength);
      owner.qd_wx.set(qd_wx_n + stepSize * (k_qdd_wx[0] / 6.0 + k_qdd_wx[1] / 3.0 + k_qdd_wx[2] / 3.0 + k_qdd_wx[3] / 6.0));
      owner.qd_wy.set(qd_wy_n + stepSize * (k_qdd_wy[0] / 6.0 + k_qdd_wy[1] / 3.0 + k_qdd_wy[2] / 3.0 + k_qdd_wy[3] / 6.0));
      owner.qd_wz.set(qd_wz_n + stepSize * (k_qdd_wz[0] / 6.0 + k_qdd_wz[1] / 3.0 + k_qdd_wz[2] / 3.0 + k_qdd_wz[3] / 6.0));

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = (Joint) owner.childrenJoints.get(i);

         child.physics.recursiveRungeKuttaSum(stepSize);
      }
   }

   @Override
   public void recursiveSaveTempState()
   {
      q_x_n = owner.q_x.getDoubleValue();
      q_y_n = owner.q_y.getDoubleValue();
      q_z_n = owner.q_z.getDoubleValue();
      qd_x_n = owner.qd_x.getDoubleValue();
      qd_y_n = owner.qd_y.getDoubleValue();
      qd_z_n = owner.qd_z.getDoubleValue();
      q_qs_n = owner.q_qs.getDoubleValue();
      q_qx_n = owner.q_qx.getDoubleValue();
      q_qy_n = owner.q_qy.getDoubleValue();
      q_qz_n = owner.q_qz.getDoubleValue();
      qd_wx_n = owner.qd_wx.getDoubleValue();
      qd_wy_n = owner.qd_wy.getDoubleValue();
      qd_wz_n = owner.qd_wz.getDoubleValue();

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = (Joint) owner.childrenJoints.get(i);

         child.physics.recursiveSaveTempState();
      }
   }

   @Override
   public void recursiveRestoreTempState()
   {
      owner.q_x.set(q_x_n);
      owner.q_y.set(q_y_n);
      owner.q_z.set(q_z_n);
      owner.qd_x.set(qd_x_n);
      owner.qd_y.set(qd_y_n);
      owner.qd_z.set(qd_z_n);
      owner.q_qs.set(q_qs_n);
      owner.q_qx.set(q_qx_n);
      owner.q_qy.set(q_qy_n);
      owner.q_qz.set(q_qz_n);
      owner.qd_wx.set(qd_wx_n);
      owner.qd_wy.set(qd_wy_n);
      owner.qd_wz.set(qd_wz_n);

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = (Joint) owner.childrenJoints.get(i);

         child.physics.recursiveRestoreTempState();
      }
   }

   @Override
   protected void impulseResponseComputeDeltaV(SpatialVector delta_v_parent, SpatialVector delta_v_me)
   {
      // Override with FloatingJoint as in Mirtich p. 144
      // Already have computed I_hat_inverse from the dynamics.  Use that...
      Y_hat_i.getMatrix(Y_hat_matrix);

      // a_hat_matrix = I_hat_inverse.times(Y_hat_matrix);
      a_hat_matrix.timesEquals(I_hat_inverse, Y_hat_matrix);
      delta_v_me.top.set(-a_hat_matrix.get(0, 0), -a_hat_matrix.get(1, 0), -a_hat_matrix.get(2, 0));
      delta_v_me.bottom.set(-a_hat_matrix.get(3, 0), -a_hat_matrix.get(4, 0), -a_hat_matrix.get(5, 0));
   }

   @Override
   protected void propagateImpulseSetDeltaVOnPath(SpatialVector delta_v_parent, SpatialVector delta_v_me)
   {
      // Override with FloatingJoint as in Mirtich p. 144
      // Already have computed I_hat_inverse from the dynamics.  Use that...
      Y_hat_i.getMatrix(Y_hat_matrix);

      // a_hat_matrix = I_hat_inverse.times(Y_hat_matrix);
      a_hat_matrix.timesEquals(I_hat_inverse, Y_hat_matrix);

      // System.out.println("Y_hat_i: " + Y_hat_i);
      delta_v_me.top.set(-a_hat_matrix.get(0, 0), -a_hat_matrix.get(1, 0), -a_hat_matrix.get(2, 0));
      delta_v_me.bottom.set(-a_hat_matrix.get(3, 0), -a_hat_matrix.get(4, 0), -a_hat_matrix.get(5, 0));

      // These are defined in body coords. Don't rotate.
      owner.qd_wx.set(owner.qd_wx.getDoubleValue() + delta_v_me.top.getX());
      owner.qd_wy.set(owner.qd_wy.getDoubleValue() + delta_v_me.top.getY());
      owner.qd_wz.set(owner.qd_wz.getDoubleValue() + delta_v_me.top.getZ());

      // Rotate into world coords.
      delta_qd_xyz.set(delta_v_me.bottom);
      R0_i.transform(delta_qd_xyz);
      owner.qd_x.set(owner.qd_x.getDoubleValue() + delta_qd_xyz.getX());
      owner.qd_y.set(owner.qd_y.getDoubleValue() + delta_qd_xyz.getY());
      owner.qd_z.set(owner.qd_z.getDoubleValue() + delta_qd_xyz.getZ());
   }

   @Override
   protected boolean jointDependentVerifyReasonableAccelerations()
   {
      if (Math.abs(owner.qdd_x.getDoubleValue()) > Joint.MAX_TRANS_ACCEL)
      {
         return false;
      }

      if (Math.abs(owner.qdd_y.getDoubleValue()) > Joint.MAX_TRANS_ACCEL)
      {
         return false;
      }

      if (Math.abs(owner.qdd_z.getDoubleValue()) > Joint.MAX_TRANS_ACCEL)
      {
         return false;
      }

      if (Math.abs(owner.qdd_wx.getDoubleValue()) > Joint.MAX_ROT_ACCEL)
      {
         return false;
      }

      if (Math.abs(owner.qdd_wy.getDoubleValue()) > Joint.MAX_ROT_ACCEL)
      {
         return false;
      }

      if (Math.abs(owner.qdd_wz.getDoubleValue()) > Joint.MAX_ROT_ACCEL)
      {
         return false;
      }

      return true;
   }
}
