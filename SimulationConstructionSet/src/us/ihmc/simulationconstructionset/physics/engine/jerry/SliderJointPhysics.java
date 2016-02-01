package us.ihmc.simulationconstructionset.physics.engine.jerry;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SliderJoint;

/**
 * @author Peter Abeles
 */
public class SliderJointPhysics extends JointPhysics<SliderJoint>
{

   private double q_n, qd_n;
   private double[] k_qdd = new double[4], k_qd = new double[4];

   public SliderJointPhysics(SliderJoint owner)
   {
      super(owner);
   }

   /**
    * Updates the joint velocity.  This function is used when applying impluse forces
    *
    * @param delta_qd change in velocity
    */
   protected void jointDependentChangeVelocity(double delta_qd)
   {
      owner.qd.set(owner.qd.getDoubleValue() + delta_qd);
   }

   /**
    * During the first featherstone pass the joint rotation matrix must be calculated.  As slider joints
    * support no rotation this matrix is always the identity matrix
    *
    * @param Rh_i Matrix3d in which the rotation matrix is stored
    */
   protected void jointDependentSetAndGetRotation(Matrix3d Rh_i)
   {
      Rh_i.setIdentity();
   }

   /**
    * The first featherstone pass updates the velocities and positions of all joints.  Such updates are joint
    * dependent and this method handles the velocity updates for slider joints.  If limit stops exist
    * they are also applied here.
    */
   protected void jointDependentFeatherstonePassOne()
   {
      Q_i = owner.doPDControl() + owner.tau.getDoubleValue();

      // Limit stops:
      if (owner.tauJointLimit != null)
      {
         if (owner.q.getDoubleValue() < owner.q_min)
         {
            double limitTorque = owner.k_limit * (owner.q_min - owner.q.getDoubleValue()) - owner.b_limit * owner.qd.getDoubleValue();
            if (limitTorque < 0.0) limitTorque = 0.0;
            owner.tauJointLimit.set(limitTorque);
         }
         else if (owner.q.getDoubleValue() > owner.q_max)
         {
            double limitTorque = owner.k_limit * (owner.q_max - owner.q.getDoubleValue()) - owner.b_limit * owner.qd.getDoubleValue();
            if (limitTorque > 0.0) limitTorque = 0.0;
            owner.tauJointLimit.set(limitTorque);
         }
         else
         {
            owner.tauJointLimit.set(0.0);
         }

         Q_i = Q_i + owner.tauJointLimit.getDoubleValue();
      }

      if (owner.tauDamping != null)
      {
         if (owner.qd.getDoubleValue() > 0.0)
         {
            owner.tauDamping.set(-owner.f_stiction - owner.b_damp * owner.qd.getDoubleValue());
         }
         else if (owner.qd.getDoubleValue() < -0.0)
         {
            owner.tauDamping.set(owner.f_stiction - owner.b_damp * owner.qd.getDoubleValue());
         }
         else
         {
            owner.tauDamping.set(0.0 - owner.b_damp * owner.qd.getDoubleValue());
         }

         Q_i = Q_i + owner.tauDamping.getDoubleValue();
      }

      // v_i <- v_i + q_i_dot u_i

      v_i.x = v_i.x + owner.qd.getDoubleValue() * u_i.x;
      v_i.y = v_i.y + owner.qd.getDoubleValue() * u_i.y;
      v_i.z = v_i.z + owner.qd.getDoubleValue() * u_i.z;

   }

   /**
    * Sets the distance betweent this joint and the center of mass belonging to the link
    * which it is attached.  This function is called only once.
    */
   protected void jointDependentSet_d_i()
   {
      d_i.set(u_i);
      d_i.scale(owner.q.getDoubleValue());
      d_i.add(owner.getLink().getComOffset());
   }

   private Vector3d w_hXr_i = new Vector3d();
   private Vector3d temp1 = new Vector3d(), temp2 = new Vector3d();
   private Vector3d vel_i = new Vector3d();    // vel_i is the vector velocity of joint i (vel_i = q_dot_i * u_i)

   /**
    *
    * @param w_h Vector3d
    */
   protected void jointDependentFeatherstonePassTwo(Vector3d w_h)
   {
      // Coriolis Forces:

      vel_i.set(u_i);
      vel_i.scale(owner.qd.getDoubleValue());

      c_hat_i.top.set(0.0, 0.0, 0.0);

      w_hXr_i.cross(w_h, r_i);

      temp1.cross(w_h, w_hXr_i);
      temp2.cross(w_h, vel_i);

      temp2.scale(2.0);

      c_hat_i.bottom.add(temp1, temp2);

      // Spatial Joint axis:

      s_hat_i.top.set(0.0, 0.0, 0.0);
      s_hat_i.bottom.set(u_i);

      // System.out.print(this.name + ":   " );System.out.println(s_hat_i);

   }

   /**
    *
    * @param Q double
    * @param passNumber int
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
    * Calculates the Runge-Kutta sum for the relevant values of this joint.  Slider joints
    * calculate only position and velocity.
    *
    * @param stepSize time in seconds between these values and the previous values
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
    * Recurse over each joint and save the relevant information.  Slider joints save only
    * position and velocity.  This function is used to save the intial state of these values
    * so that they may be restored prior to each euler integration.
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
    * Recurse over each joint and restore the relevant information.  Slider joints save only
    * position and velocity.  This function is used to restore values to their original state
    * prior to each euler integration.
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
    * Checks to see if accelerations were reasonable.  If not the function
    * returns false.  Generally a false result from this function is accompanied by
    * an UnreasonableAccelerationException
    *
    * @return were the accelerations reasonable?
    */
   protected boolean jointDependentVerifyReasonableAccelerations()
   {
      if (Math.abs(owner.qdd.getDoubleValue()) > Joint.MAX_TRANS_ACCEL)
      {
         return false;
      }

      return true;
   }
}
