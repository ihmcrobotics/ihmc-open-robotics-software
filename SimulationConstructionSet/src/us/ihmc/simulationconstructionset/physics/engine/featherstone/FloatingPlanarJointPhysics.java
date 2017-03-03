package us.ihmc.simulationconstructionset.physics.engine.featherstone;

import java.util.ArrayList;
import java.util.Collection;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.Plane;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointGroup;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.SpatialVector;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.mathfunctions.Matrix;


public class FloatingPlanarJointPhysics extends JointPhysics<FloatingPlanarJoint>
{
   private double[] k_qdd_t1 = new double[4], k_qdd_t2 = new double[4];
   private double[] k_qd_t1 = new double[4], k_qd_t2 = new double[4];
   private double[] k_qdd_rot = new double[4];
   private double[] k_qd_rot = new double[4];

   private double q_t1_n, q_t2_n, qd_t1_n, qd_t2_n, q_rot_n, qd_rot_n;

   public FloatingPlanarJointPhysics(FloatingPlanarJoint owner)
   {
      super(owner);
   }

   @Override
   protected void jointDependentChangeVelocity(double delta_qd)
   {
      System.err.println("Error!!!! FloatingPlanarJoint.jointDependentChangeVelocity should never be called!!!");
   }

   // Override featherstonePassOne since don't need to do everything...

   @Override
   public void featherstonePassOne(Vector3D w_h, Vector3D v_h, RotationMatrix Rh_0)
   {
      // this.update(false);
      // this.jointTransform3D.get(Ri_0);
      this.jointDependentSetAndGetRotation(Ri_0);

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
         Joint child = owner.childrenJoints.get(i);
         child.physics.featherstonePassOne(w_i, v_i, Ri_0);
      }
   }


   @Override
   protected void jointDependentFeatherstonePassOne()
   {
      // No torques:
      Q_i = 0.0;

      // Are Holding on to the velocities as part of the state:

      if (owner.type == Plane.YZ)
      {
         w_i.set(owner.qd_rot.getDoubleValue(), 0.0, 0.0);
         v_i.set(0.0, owner.qd_t1.getDoubleValue(), owner.qd_t2.getDoubleValue());
      }

      else if (owner.type == Plane.XZ)
      {
         w_i.set(0.0, owner.qd_rot.getDoubleValue(), 0.0);
         v_i.set(owner.qd_t1.getDoubleValue(), 0.0, owner.qd_t2.getDoubleValue());
      }

      else    // (type == XY)
      {
         w_i.set(0.0, 0.0, owner.qd_rot.getDoubleValue());
         v_i.set(owner.qd_t1.getDoubleValue(), owner.qd_t2.getDoubleValue(), 0.0);
      }

      Ri_0.transform(w_i);
      Ri_0.transform(v_i);
   }

   @Override
   protected void jointDependentSet_d_i()
   {
      System.err.println("Error!!!! FloatingPlanarJoint.jointDependentSet_d_i should never be called!!!");
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


   @Override
   public void featherstonePassThree()
   {
      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.featherstonePassThree(I_hat_i, Z_hat_i);
      }

   }

   private Vector3D wdXr = new Vector3D(), wXr = new Vector3D(), wXwXr = new Vector3D();
   @Override
   public void featherstonePassFour(SpatialVector a_hat_h, int passNumber) throws UnreasonableAccelerationException
   {
      if (owner.type == Plane.XY)
      {
         I_hat_i.getPlanarXYMatrix(I_hat_matrix);

         // I_hat_inverse = I_hat_matrix.inverse();
         I_hat_matrix.inverse(I_hat_inverse);
         Z_hat_i.getPlanarXYMatrix(Z_hat_matrix);

         // a_hat_matrix = I_hat_inverse.times(Z_hat_matrix);
         a_hat_matrix.timesEquals(I_hat_inverse, Z_hat_matrix);

         a_hat_i.top.set(0.0, 0.0, -a_hat_matrix.get(0, 0));
         a_hat_i.bottom.set(-a_hat_matrix.get(1, 0), -a_hat_matrix.get(2, 0), 0.0);
      }

      else if (owner.type == Plane.XZ)
      {
         I_hat_i.getPlanarXZMatrix(I_hat_matrix);

         // I_hat_inverse = I_hat_matrix.inverse();
         I_hat_matrix.inverse(I_hat_inverse);
         Z_hat_i.getPlanarXZMatrix(Z_hat_matrix);

         // a_hat_matrix = I_hat_inverse.times(Z_hat_matrix);
         a_hat_matrix.timesEquals(I_hat_inverse, Z_hat_matrix);

         a_hat_i.top.set(0.0, -a_hat_matrix.get(0, 0), 0.0);
         a_hat_i.bottom.set(-a_hat_matrix.get(1, 0), 0.0, -a_hat_matrix.get(2, 0));
      }

      else    // if (type == YZ)
      {
         I_hat_i.getPlanarYZMatrix(I_hat_matrix);

         // I_hat_inverse = I_hat_matrix.inverse();
         I_hat_matrix.inverse(I_hat_inverse);
         Z_hat_i.getPlanarYZMatrix(Z_hat_matrix);

         // a_hat_matrix = I_hat_inverse.times(Z_hat_matrix);
         a_hat_matrix.timesEquals(I_hat_inverse, Z_hat_matrix);

         a_hat_i.top.set(-a_hat_matrix.get(0, 0), 0.0, 0.0);
         a_hat_i.bottom.set(0.0, -a_hat_matrix.get(1, 0), -a_hat_matrix.get(2, 0));
      }

      // System.out.println(Z_hat_i);
      // System.out.println(I_hat_i);

      // Rotate into world coords...
      R0_i.set(Ri_0);
      R0_i.transpose();

      // a_hat_world_top.set(a_hat_i.top);  // Don't need to do this since planar!
      a_hat_world_bot.set(a_hat_i.bottom);

      // +++TK 02/09/12 Transform to joint location from com location.
      wXr.cross(w_i, owner.link.comOffset);
      wXwXr.cross(w_i, wXr);
      wdXr.cross(a_hat_i.top, owner.link.comOffset);
      a_hat_world_bot.sub(wdXr);
      a_hat_world_bot.sub(wXwXr);

      // R0_i.transform(a_hat_world_top);  // Don't need to do this since planar!
      R0_i.transform(a_hat_world_bot);

      if (owner.type == Plane.YZ)
      {
         owner.qdd_t1.set(a_hat_world_bot.getY());
         owner.qdd_t2.set(a_hat_world_bot.getZ());
         owner.qdd_rot.set(a_hat_i.top.getX());    // a_hat_world_top.x;  // Don't need to do this since planar!
      }

      else if (owner.type == Plane.XZ)
      {
         owner.qdd_t1.set(a_hat_world_bot.getX());
         owner.qdd_t2.set(a_hat_world_bot.getZ());
         owner.qdd_rot.set(a_hat_i.top.getY());    // a_hat_world_top.y;  // Don't need to do this since planar!
      }

      else    // if (type == XY)
      {
         owner.qdd_t1.set(a_hat_world_bot.getX());
         owner.qdd_t2.set(a_hat_world_bot.getY());
         owner.qdd_rot.set(a_hat_i.top.getZ());    // a_hat_world_top.z;  // Don't need to do this since planar!
      }


      k_qdd_t1[passNumber] = owner.qdd_t1.getDoubleValue();
      k_qdd_t2[passNumber] = owner.qdd_t2.getDoubleValue();
      k_qd_t1[passNumber] = owner.qd_t1.getDoubleValue();
      k_qd_t2[passNumber] = owner.qd_t2.getDoubleValue();
      k_qdd_rot[passNumber] = owner.qdd_rot.getDoubleValue();
      k_qd_rot[passNumber] = owner.qd_rot.getDoubleValue();


      //Check for unreasonable accelerations:
      if (!jointDependentVerifyReasonableAccelerations())
      {
         ArrayList<Joint> unreasonableAccelerationJoints = new ArrayList<Joint>();
         unreasonableAccelerationJoints.add(owner);
         throw new UnreasonableAccelerationException(unreasonableAccelerationJoints);
      }

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.featherstonePassFour(a_hat_i, passNumber);
      }

      // System.out.println(this);
   }


   @Override
   protected void jointDependentRecordK(int passNumber)
   {
      k_qdd_t1[passNumber] = owner.qdd_t1.getDoubleValue();
      k_qdd_t2[passNumber] = owner.qdd_t2.getDoubleValue();
      k_qd_t1[passNumber] = owner.qd_t1.getDoubleValue();
      k_qd_t2[passNumber] = owner.qd_t2.getDoubleValue();
      k_qdd_rot[passNumber] = owner.qdd_rot.getDoubleValue();
      k_qd_rot[passNumber] = owner.qd_rot.getDoubleValue();
   }

   @Override
   protected void jointDependentFeatherstonePassFour(double Q, int passNumber)
   {
      // Do nothing here...
   }

   @Override
   public void recursiveEulerIntegrate(double stepSize)
   {
      owner.q_t1.set(q_t1_n + owner.qd_t1.getDoubleValue() * stepSize);
      owner.q_t2.set(q_t2_n + owner.qd_t2.getDoubleValue() * stepSize);
      owner.q_rot.set(q_rot_n + owner.qd_rot.getDoubleValue() * stepSize);
      owner.qd_t1.set(qd_t1_n + owner.qdd_t1.getDoubleValue() * stepSize);
      owner.qd_t2.set(qd_t2_n + owner.qdd_t2.getDoubleValue() * stepSize);
      owner.qd_rot.set(qd_rot_n + owner.qdd_rot.getDoubleValue() * stepSize);

      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveEulerIntegrate(stepSize);
      }

   }

   @Override
   public void recursiveRungeKuttaSum(double stepSize)
   {
      owner.q_t1.set(q_t1_n + stepSize * (k_qd_t1[0] / 6.0 + k_qd_t1[1] / 3.0 + k_qd_t1[2] / 3.0 + k_qd_t1[3] / 6.0));
      owner.q_t2.set(q_t2_n + stepSize * (k_qd_t2[0] / 6.0 + k_qd_t2[1] / 3.0 + k_qd_t2[2] / 3.0 + k_qd_t2[3] / 6.0));
      owner.q_rot.set(q_rot_n + stepSize * (k_qd_rot[0] / 6.0 + k_qd_rot[1] / 3.0 + k_qd_rot[2] / 3.0 + k_qd_rot[3] / 6.0));

      owner.qd_t1.set(qd_t1_n + stepSize * (k_qdd_t1[0] / 6.0 + k_qdd_t1[1] / 3.0 + k_qdd_t1[2] / 3.0 + k_qdd_t1[3] / 6.0));
      owner.qd_t2.set(qd_t2_n + stepSize * (k_qdd_t2[0] / 6.0 + k_qdd_t2[1] / 3.0 + k_qdd_t2[2] / 3.0 + k_qdd_t2[3] / 6.0));
      owner.qd_rot.set(qd_rot_n + stepSize * (k_qdd_rot[0] / 6.0 + k_qdd_rot[1] / 3.0 + k_qdd_rot[2] / 3.0 + k_qdd_rot[3] / 6.0));

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveRungeKuttaSum(stepSize);
      }
   }



   @Override
   public void recursiveSaveTempState()
   {
      q_t1_n = owner.q_t1.getDoubleValue();
      q_t2_n = owner.q_t2.getDoubleValue();
      q_rot_n = owner.q_rot.getDoubleValue();
      qd_t1_n = owner.qd_t1.getDoubleValue();
      qd_t2_n = owner.qd_t2.getDoubleValue();
      qd_rot_n = owner.qd_rot.getDoubleValue();

      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveSaveTempState();
      }
   }

   @Override
   public void recursiveRestoreTempState()
   {
      owner.q_t1.set(q_t1_n);
      owner.q_t2.set(q_t2_n);
      owner.q_rot.set(q_rot_n);
      owner.qd_t1.set(qd_t1_n);
      owner.qd_t2.set(qd_t2_n);
      owner.qd_rot.set(qd_rot_n);

      // Recurse over the children:

      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);
         child.physics.recursiveRestoreTempState();
      }
   }


   @Override
   protected void impulseResponseComputeDeltaV(SpatialVector delta_v_parent, SpatialVector delta_v_me)
   {
      // Override with FloatingPlanarJoint as in Mirtich p. 144
      // Already have computed I_hat_inverse from the dynamics.  Use that...

      if (owner.type == Plane.XY)
      {
         Y_hat_i.getPlanarXYMatrix(Y_hat_matrix);

         // a_hat_matrix = I_hat_inverse.times(Y_hat_matrix);
         a_hat_matrix.timesEquals(I_hat_inverse, Y_hat_matrix);

         delta_v_me.top.set(0.0, 0.0, -a_hat_matrix.get(0, 0));
         delta_v_me.bottom.set(-a_hat_matrix.get(1, 0), -a_hat_matrix.get(2, 0), 0.0);
      }

      else if (owner.type == Plane.XZ)
      {
         Y_hat_i.getPlanarXZMatrix(Y_hat_matrix);

         // a_hat_matrix = I_hat_inverse.times(Y_hat_matrix);
         a_hat_matrix.timesEquals(I_hat_inverse, Y_hat_matrix);

         delta_v_me.top.set(0.0, -a_hat_matrix.get(0, 0), 0.0);
         delta_v_me.bottom.set(-a_hat_matrix.get(1, 0), 0.0, -a_hat_matrix.get(2, 0));
      }

      else
      {
         Y_hat_i.getPlanarYZMatrix(Y_hat_matrix);

         // a_hat_matrix = I_hat_inverse.times(Y_hat_matrix);
         a_hat_matrix.timesEquals(I_hat_inverse, Y_hat_matrix);

         delta_v_me.top.set(-a_hat_matrix.get(0, 0), 0.0, 0.0);
         delta_v_me.bottom.set(0.0, -a_hat_matrix.get(1, 0), -a_hat_matrix.get(2, 0));
      }


   }


   private Vector3D delta_qd_xyz = new Vector3D();

   @Override
   protected void propagateImpulseSetDeltaVOnPath(SpatialVector delta_v_parent, SpatialVector delta_v_me)
   {
      // Override with FloatingJoint as in Mirtich p. 144
      // Already have computed I_hat_inverse from the dynamics.  Use that...

      if (owner.type == Plane.XY)
      {
         Y_hat_i.getPlanarXYMatrix(Y_hat_matrix);

         // a_hat_matrix = I_hat_inverse.times(Y_hat_matrix);
         a_hat_matrix.timesEquals(I_hat_inverse, Y_hat_matrix);

         delta_v_me.top.set(0.0, 0.0, -a_hat_matrix.get(0, 0));
         delta_v_me.bottom.set(-a_hat_matrix.get(1, 0), -a_hat_matrix.get(2, 0), 0.0);

         // Rotate velocity, but not rotation into world coords.
         delta_qd_xyz.set(delta_v_me.bottom);
         R0_i.transform(delta_qd_xyz);

         owner.qd_rot.set(owner.qd_rot.getDoubleValue() + delta_v_me.top.getZ());
         owner.qd_t1.set(owner.qd_t1.getDoubleValue() + delta_qd_xyz.getX());
         owner.qd_t2.set(owner.qd_t2.getDoubleValue() + delta_qd_xyz.getY());
      }

      else if (owner.type == Plane.XZ)
      {
         Y_hat_i.getPlanarXZMatrix(Y_hat_matrix);

         // a_hat_matrix = I_hat_inverse.times(Y_hat_matrix);
         a_hat_matrix.timesEquals(I_hat_inverse, Y_hat_matrix);

         delta_v_me.top.set(0.0, -a_hat_matrix.get(0, 0), 0.0);
         delta_v_me.bottom.set(-a_hat_matrix.get(1, 0), 0.0, -a_hat_matrix.get(2, 0));

         // Rotate velocity, but not rotation into world coords.
         delta_qd_xyz.set(delta_v_me.bottom);
         R0_i.transform(delta_qd_xyz);

         owner.qd_rot.set(owner.qd_rot.getDoubleValue() + delta_v_me.top.getY());
         owner.qd_t1.set(owner.qd_t1.getDoubleValue() + delta_qd_xyz.getX());
         owner.qd_t2.set(owner.qd_t2.getDoubleValue() + delta_qd_xyz.getZ());
      }

      else
      {
         Y_hat_i.getPlanarYZMatrix(Y_hat_matrix);

         // a_hat_matrix = I_hat_inverse.times(Y_hat_matrix);
         a_hat_matrix.timesEquals(I_hat_inverse, Y_hat_matrix);

         delta_v_me.top.set(-a_hat_matrix.get(0, 0), 0.0, 0.0);
         delta_v_me.bottom.set(0.0, -a_hat_matrix.get(1, 0), -a_hat_matrix.get(2, 0));

         // Rotate velocity, but not rotation into world coords.
         delta_qd_xyz.set(delta_v_me.bottom);
         R0_i.transform(delta_qd_xyz);

         owner.qd_rot.set(owner.qd_rot.getDoubleValue() + delta_v_me.top.getX());
         owner.qd_t1.set(owner.qd_t1.getDoubleValue() + delta_qd_xyz.getY());
         owner.qd_t2.set(owner.qd_t2.getDoubleValue() + delta_qd_xyz.getZ());
      }

   }

   private Matrix I_hat_matrix = new Matrix(3, 3);    // Matrix(6,6);
   private Matrix Z_hat_matrix = new Matrix(3, 1);    // new Matrix(6,1);
   private Matrix a_hat_matrix = new Matrix(3, 1);    // new Matrix(6,1);

   // private Matrix3d R0_i = new Matrix3d();
   private final Vector3D a_hat_world_bot = new Vector3D();

   private Matrix Y_hat_matrix = new Matrix(3, 1);    // new Matrix(6,1);
   private Matrix I_hat_inverse = new Matrix(3, 3);

   @Override
   protected boolean jointDependentVerifyReasonableAccelerations()
   {
      if (Math.abs(owner.qdd_t1.getDoubleValue()) > Joint.MAX_TRANS_ACCEL)
         return false;
      if (Math.abs(owner.qdd_t2.getDoubleValue()) > Joint.MAX_TRANS_ACCEL)
         return false;
      if (Math.abs(owner.qdd_rot.getDoubleValue()) > Joint.MAX_ROT_ACCEL)
         return false;

      return true;
   }

   @Override
   protected void jointDependentSetAndGetRotation(RotationMatrix Rh_i)
   {
      Rh_i.setIdentity();    // We probably can rely on Rh_i not changing its 1 and 0 elements but let's just be safe.

      if (owner.type == Plane.YZ)
      {
         Rh_i.setToRollMatrix(owner.q_rot.getDoubleValue());
         
      }    // Rotation about X
      else if (owner.type == Plane.XZ)
      {
         Rh_i.setToPitchMatrix(owner.q_rot.getDoubleValue());
      }    // Rotation about Y
      else
      {
         Rh_i.setToYawMatrix(owner.q_rot.getDoubleValue());
      }    // Rotation about Z
   }
}
