package us.ihmc.simulationconstructionset.physics.engine.jerry;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.robotics.MathTools;

/**
 * @author Peter Abeles
 */
public class PinJointPhysics extends JointPhysics<PinJoint>
{

   private Vector3d vel_iXd_i = new Vector3d();
   private Vector3d w_hXr_i = new Vector3d();
   private Vector3d vel_i = new Vector3d();    // vel_i is the vector velocity of joint i (vel_i = q_dot_i * u_i)
   private Vector3d temp1 = new Vector3d(), temp2 = new Vector3d(), temp3 = new Vector3d();

   private double[] k_qdd = new double[4], k_qd = new double[4];

   private double q_n, qd_n;

   public PinJointPhysics(PinJoint owner)
   {
      super(owner);
   }

   /**
    * Updates the velocity of this joint.  This function is called during collision events
    * when the impulse is propagating up and back down the tree.
    *
    * @param delta_qd change in velocity to be added
    */
   protected void jointDependentChangeVelocity(double delta_qd)
   {
      owner.qd.set(owner.qd.getDoubleValue() + delta_qd);
   }

   /**
    * Calculates the current rotation matrix and stores it in the provided Matrix3d.  This is used in the first featherstone pass
    * to update the rotations and transforms between each reference frame.
    *
    * @param Rh_i Matrix3d in which to store the rotation
    */
   protected void jointDependentSetAndGetRotation(Matrix3d Rh_i)
   {
      Rh_i.setIdentity();    // We probably can rely on Rh_i not changing its 1 and 0 elements but let's just be safe.

      double cosQ = Math.cos(owner.q.getDoubleValue()), sinQ = Math.sin(owner.q.getDoubleValue());
      @SuppressWarnings("unused") double
            one_cosQ = 1.0 - cosQ, one_sinQ = 1.0 - sinQ;
      double ux_sinQ = u_i.x * sinQ, uy_sinQ = u_i.y * sinQ, uz_sinQ = u_i.z * sinQ;
      double uxy_one_cosQ = u_i.x * u_i.y * one_cosQ, uxz_one_cosQ = u_i.x * u_i.z * one_cosQ, uyz_one_cosQ = u_i.y * u_i.z * one_cosQ;

      Rh_i.m00 = cosQ + u_i.x * u_i.x * one_cosQ;
      Rh_i.m01 = uxy_one_cosQ - uz_sinQ;
      Rh_i.m02 = uxz_one_cosQ + uy_sinQ;
      Rh_i.m10 = uxy_one_cosQ + uz_sinQ;
      Rh_i.m11 = cosQ + u_i.y * u_i.y * one_cosQ;
      Rh_i.m12 = uyz_one_cosQ - ux_sinQ;
      Rh_i.m20 = uxz_one_cosQ - uy_sinQ;
      Rh_i.m21 = uyz_one_cosQ + ux_sinQ;
      Rh_i.m22 = cosQ + u_i.z * u_i.z * one_cosQ;

      /*
       * if (this.axis == Axis.X) {Rh_i.setElement(1,1,cosQ);Rh_i.setElement(2,2,cosQ);Rh_i.setElement(1,2,-sinQ);Rh_i.setElement(2,1,sinQ);}
       *     else if (this.axis == Axis.Y) {Rh_i.setElement(0,0,cosQ);Rh_i.setElement(2,2,cosQ);Rh_i.setElement(0,2,sinQ);Rh_i.setElement(2,0,-sinQ);}
       *     else if (this.axis == Axis.Z) {Rh_i.setElement(0,0,cosQ);Rh_i.setElement(1,1,cosQ);Rh_i.setElement(0,1,-sinQ);Rh_i.setElement(1,0,sinQ);}
       */
   }

   /**
    * Calculates the joint dependent components of Featherstone pass one.  The primary purpose of this method is the computation of angular and linear
    * velocities although limits associated with this joint are also applied here.
    */
   protected void jointDependentFeatherstonePassOne()
   {
      // User torque-speed curve:
      if (owner.torqueSpeedCurve != null)
      {
         owner.tau.set(owner.torqueSpeedCurve.limitTorque(owner.tau.getDoubleValue(), owner.qd.getDoubleValue()));
      }

      // Torque Limits
      if (owner.tau_max != null)
      {
         double maxTorque = owner.tau_max.getDoubleValue();
         owner.tau.set(MathTools.clipToMinMax(owner.tau.getDoubleValue(), -maxTorque, maxTorque));
      }

      Q_i = owner.doPDControl() + owner.tau.getDoubleValue();

      // Limit stops:
      if (owner.tauJointLimit != null)
      {
         if (owner.q.getDoubleValue() < owner.q_min)
         {
            owner.tauJointLimit.set(owner.k_limit * (owner.q_min - owner.q.getDoubleValue()) - owner.b_limit * owner.qd.getDoubleValue());
         }
         else if (owner.q.getDoubleValue() > owner.q_max)
         {
            owner.tauJointLimit.set(owner.k_limit * (owner.q_max - owner.q.getDoubleValue()) - owner.b_limit * owner.qd.getDoubleValue());
         }
         else
         {
            owner.tauJointLimit.set(0.0);
         }

         Q_i = Q_i + owner.tauJointLimit.getDoubleValue();
      }

      // Velocity Limits:
      if (owner.tauVelocityLimit != null)
      {
         if (owner.qd.getDoubleValue() < -owner.qd_max.getDoubleValue())
         {
            owner.tauVelocityLimit.set(-owner.b_vel_limit.getDoubleValue() * (owner.qd.getDoubleValue() + owner.qd_max.getDoubleValue()));
         }
         else if (owner.qd.getDoubleValue() > owner.qd_max.getDoubleValue())
         {
            owner.tauVelocityLimit.set(-owner.b_vel_limit.getDoubleValue() * (owner.qd.getDoubleValue() - owner.qd_max.getDoubleValue()));
         }
         else
         {
            owner.tauVelocityLimit.set(0.0);
         }

         Q_i = Q_i + owner.tauVelocityLimit.getDoubleValue();
      }

      if (owner.tauDamping != null)
      {
         if (owner.qd.getDoubleValue() > 0.0)
         {
            owner.tauDamping.set(-owner.getJointStiction() -owner.getDamping() * owner.qd.getDoubleValue());
         }
         else if (owner.qd.getDoubleValue() < -0.0)
         {
            owner.tauDamping.set(owner.getJointStiction() - owner.getDamping() * owner.qd.getDoubleValue());
         }
         else
         {
            owner.tauDamping.set(0.0 -owner.getDamping() * owner.qd.getDoubleValue());
         }

         Q_i = Q_i + owner.tauDamping.getDoubleValue();
      }

      // w_i <- w_i + q_i_dot u_i
      w_i.x = w_i.x + owner.qd.getDoubleValue() * u_i.x;
      w_i.y = w_i.y + owner.qd.getDoubleValue() * u_i.y;
      w_i.z = w_i.z + owner.qd.getDoubleValue() * u_i.z;

      // v_i <- v_i + q_i_dot (u_i X d_i)
      v_i.x = v_i.x + owner.qd.getDoubleValue() * u_iXd_i.x;
      v_i.y = v_i.y + owner.qd.getDoubleValue() * u_iXd_i.y;
      v_i.z = v_i.z + owner.qd.getDoubleValue() * u_iXd_i.z;
   }

   /**
    * Sets the offset between this joint and its link.  This method is called only once.
    */
   protected void jointDependentSet_d_i()
   {
      d_i.set(owner.getLink().getComOffset());
   }

   /**
    * While the majority of the second featherstone pass is carried out via the method in Joint, coriolis forces
    * and the spatial joint axis are joint dependent and therefore calculated here.
    *
    * @param w_h Vector3d representing the rotational velocity of the previous link in this link's coordinates
    */
   protected void jointDependentFeatherstonePassTwo(Vector3d w_h)
   {
      // Coriolis Forces:
      vel_i.set(u_i);
      vel_i.scale(owner.qd.getDoubleValue());
      c_hat_i.top.cross(w_h, vel_i);
      vel_iXd_i.cross(vel_i, d_i);
      w_hXr_i.cross(w_h, r_i);
      temp1.cross(w_h, w_hXr_i);
      temp2.cross(w_h, vel_iXd_i);
      temp3.cross(vel_i, vel_iXd_i);
      temp2.scale(2.0);
      c_hat_i.bottom.add(temp1, temp2);
      c_hat_i.bottom.add(temp3);

      // Spatial Joint axis:
      s_hat_i.top.set(u_i);
      s_hat_i.bottom.cross(u_i, d_i);

      // System.out.print(this.name + ":   " );System.out.println(s_hat_i);
   }


   /**
    * The fourth pass of the featherstone algorithm saves the resulting values so that they may be used in Runge-Kutta calculations.
    * For pin joints the position and velocity is saved.  The featherstone algorithm is executed four times to calculate the
    *
    * @param Q acceleration of the joint
    * @param passNumber number indicating the current pass
    */
   protected void jointDependentFeatherstonePassFour(double Q, int passNumber)
   {
      owner.qdd.set(Q);
      k_qdd[passNumber] = Q;
      k_qd[passNumber] = owner.qd.getDoubleValue();
   }

   /**
    * If this joint is a child of a non-dynamic root only the first component of the featherstone algorithm is executed.
    * As the k values for RK4 are saved during the fourth component this method saves the relevant information.
    *
    * @param passNumber int representing the current pass in the featherstone algorithm
    */
   protected void jointDependentRecordK(int passNumber)
   {
      k_qdd[passNumber] = owner.qdd.getDoubleValue();
      k_qd[passNumber] = owner.qd.getDoubleValue();
   }

   /**
    * In between each pass through the featherstone algorithm the relevant values must be
    * updated so that the k's needed for RK4 may be generated.  These k's represent
    * slope at several stages between the current and future points.
    *
    * @param stepSize time in seconds between the current value and next value
    */
   public void recursiveEulerIntegrate(double stepSize)
   {
      owner.q.set(q_n + stepSize * owner.qd.getDoubleValue());
      owner.qd.set(qd_n + stepSize * owner.qdd.getDoubleValue());

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);

         child.physics.recursiveEulerIntegrate(stepSize);
      }
   }

   /**
    * Recurse through each joint and perform a Runge-Kutta sum on the relevant information.
    * For pin joints only the position and velocity values are calculated.
    *
    * @param stepSize time in seconds for each step
    */
   public void recursiveRungeKuttaSum(double stepSize)
   {
      owner.q.set(q_n + stepSize * (k_qd[0] / 6.0 + k_qd[1] / 3.0 + k_qd[2] / 3.0 + k_qd[3] / 6.0));
      owner.qd.set(qd_n + stepSize * (k_qdd[0] / 6.0 + k_qdd[1] / 3.0 + k_qdd[2] / 3.0 + k_qdd[3] / 6.0));

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);

         child.physics.recursiveRungeKuttaSum(stepSize);
      }
   }

   /**
    * Recurse over the children of this joint and save the relevant information.  Pin joints
    * save only position and velocity.
    */
   public void recursiveSaveTempState()
   {
      q_n = owner.q.getDoubleValue();
      qd_n = owner.qd.getDoubleValue();

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);

         child.physics.recursiveSaveTempState();
      }
   }

   /**
    * Recurse over each joint and restore the relevant information.  Pin joints save only
    * position (angle) and velocity.
    */
   public void recursiveRestoreTempState()
   {
      owner.q.set(q_n);
      owner.qd.set(qd_n);

      // Recurse over the children:
      for (int i = 0; i < owner.childrenJoints.size(); i++)
      {
         Joint child = owner.childrenJoints.get(i);

         child.physics.recursiveRestoreTempState();
      }
   }

   /**
    * Verify that the accelerations are reasonable.  Generally
    * when this method is called a false response triggers an exception.
    *
    * @return were the accelerations reasonable?
    */
   protected boolean jointDependentVerifyReasonableAccelerations()
   {
      if (Math.abs(owner.qdd.getDoubleValue()) > Joint.MAX_ROT_ACCEL)
      {
         return false;
      }

      return true;
   }
}
