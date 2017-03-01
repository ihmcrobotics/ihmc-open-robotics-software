package us.ihmc.simulationconstructionset.physics.engine.jerry;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SliderJoint;


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
   @Override
   protected void jointDependentChangeVelocity(double delta_qd)
   {
      owner.getQDYoVariable().set(owner.getQDYoVariable().getDoubleValue() + delta_qd);
   }

   /**
    * During the first featherstone pass the joint rotation matrix must be calculated.  As slider joints
    * support no rotation this matrix is always the identity matrix
    *
    * @param Rh_i Matrix3d in which the rotation matrix is stored
    */
   @Override
   protected void jointDependentSetAndGetRotation(RotationMatrix Rh_i)
   {
      Rh_i.setIdentity();
   }

   /**
    * The first featherstone pass updates the velocities and positions of all joints.  Such updates are joint
    * dependent and this method handles the velocity updates for slider joints.  If limit stops exist
    * they are also applied here.
    */
   @Override
   protected void jointDependentFeatherstonePassOne()
   {
      Q_i = owner.doPDControl() + owner.getTauYoVariable().getDoubleValue();

      // Limit stops:
      if (owner.tauJointLimit != null)
      {
         if (owner.getQYoVariable().getDoubleValue() < owner.q_min)
         {
            double limitTorque = owner.k_limit * (owner.q_min - owner.getQYoVariable().getDoubleValue()) - owner.b_limit * owner.getQDYoVariable().getDoubleValue();
            if (limitTorque < 0.0) limitTorque = 0.0;
            owner.tauJointLimit.set(limitTorque);
         }
         else if (owner.getQYoVariable().getDoubleValue() > owner.q_max)
         {
            double limitTorque = owner.k_limit * (owner.q_max - owner.getQYoVariable().getDoubleValue()) - owner.b_limit * owner.getQDYoVariable().getDoubleValue();
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
         if (owner.getQDYoVariable().getDoubleValue() > 0.0)
         {
            owner.tauDamping.set(-owner.f_stiction - owner.b_damp * owner.getQDYoVariable().getDoubleValue());
         }
         else if (owner.getQDYoVariable().getDoubleValue() < -0.0)
         {
            owner.tauDamping.set(owner.f_stiction - owner.b_damp * owner.getQDYoVariable().getDoubleValue());
         }
         else
         {
            owner.tauDamping.set(0.0 - owner.b_damp * owner.getQDYoVariable().getDoubleValue());
         }

         Q_i = Q_i + owner.tauDamping.getDoubleValue();
      }

      // v_i <- v_i + q_i_dot u_i

      v_i.setX(v_i.getX() + owner.getQDYoVariable().getDoubleValue() * u_i.getX());
      v_i.setY(v_i.getY() + owner.getQDYoVariable().getDoubleValue() * u_i.getY());
      v_i.setZ(v_i.getZ() + owner.getQDYoVariable().getDoubleValue() * u_i.getZ());

   }

   /**
    * Sets the distance betweent this joint and the center of mass belonging to the link
    * which it is attached.  This function is called only once.
    */
   @Override
   protected void jointDependentSet_d_i()
   {
      d_i.set(u_i);
      d_i.scale(owner.getQYoVariable().getDoubleValue());
      d_i.add(owner.getLink().getComOffset());
   }

   private Vector3D w_hXr_i = new Vector3D();
   private Vector3D temp1 = new Vector3D(), temp2 = new Vector3D();
   private Vector3D vel_i = new Vector3D();    // vel_i is the vector velocity of joint i (vel_i = q_dot_i * u_i)

   /**
    *
    * @param w_h Vector3d
    */
   @Override
   protected void jointDependentFeatherstonePassTwo(Vector3D w_h)
   {
      // Coriolis Forces:

      vel_i.set(u_i);
      vel_i.scale(owner.getQDYoVariable().getDoubleValue());

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
   @Override
   protected void jointDependentFeatherstonePassFour(double Q, int passNumber)
   {
      owner.getQDDYoVariable().set(Q);
      k_qdd[passNumber] = Q;
      k_qd[passNumber] = owner.getQDYoVariable().getDoubleValue();
   }

   /**
    * If this joint is a child of a non-dynamic root only the first component of the featherstone algorithm is executed.
    * As the k values for RK4 are saved during the fourth component this method saves the relevant information.
    *
    * @param passNumber int representing the current pass in the featherstone algorithm
    */
   @Override
   protected void jointDependentRecordK(int passNumber)
   {
      k_qdd[passNumber] = owner.getQDDYoVariable().getDoubleValue();
      k_qd[passNumber] = owner.getQDYoVariable().getDoubleValue();

   }

   /**
    * In between each pass through the featherstone algorithm the relevant values must be
    * updated so that the k's needed for RK4 may be generated.  These k's represent
    * slope at several stages between the current and future points.
    *
    * @param stepSize time in seconds between the current value and next value
    */
   @Override
   public void recursiveEulerIntegrate(double stepSize)
   {
      owner.getQYoVariable().set(q_n + stepSize * owner.getQDYoVariable().getDoubleValue());
      owner.getQDYoVariable().set(qd_n + stepSize * owner.getQDDYoVariable().getDoubleValue());

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
   @Override
   public void recursiveRungeKuttaSum(double stepSize)
   {
      owner.getQYoVariable().set(q_n + stepSize * (k_qd[0] / 6.0 + k_qd[1] / 3.0 + k_qd[2] / 3.0 + k_qd[3] / 6.0));
      owner.getQDYoVariable().set(qd_n + stepSize * (k_qdd[0] / 6.0 + k_qdd[1] / 3.0 + k_qdd[2] / 3.0 + k_qdd[3] / 6.0));

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
   @Override
   public void recursiveSaveTempState()
   {
      q_n = owner.getQYoVariable().getDoubleValue();
      qd_n = owner.getQDYoVariable().getDoubleValue();

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
   @Override
   public void recursiveRestoreTempState()
   {
      owner.getQYoVariable().set(q_n);
      owner.getQDYoVariable().set(qd_n);

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
   @Override
   protected boolean jointDependentVerifyReasonableAccelerations()
   {
      if (Math.abs(owner.getQDDYoVariable().getDoubleValue()) > Joint.MAX_TRANS_ACCEL)
      {
         return false;
      }

      return true;
   }
}
