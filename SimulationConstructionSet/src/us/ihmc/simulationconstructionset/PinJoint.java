package us.ihmc.simulationconstructionset;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.physics.engine.jerry.PinJointPhysics;
import us.ihmc.simulationconstructionset.torqueSpeedCurve.TorqueSpeedCurve;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 *
 * <p>Title: SimulationConstructionSet</p>
 *
 * <p>Description: A rotational joint with a single degree of freedom.  Pin joints allow rotation around a single
 * axis specified upon creation.  There are several joint types which extend this joint: universal, gimbal and cylinder joints.  These
 * joints are either multiple pin joints grouped together or a combination of pin and slider joints.</p>
 *
 * <p>Copyright: Copyright (c) 2000</p>
 *
 * <p>Company: Yobotics, Inc.</p>
 *
 * @author not attributable
 * @version 1.0
 * @see Joint Joint
 */
// TODO Move crap into physics that deals with limits
public class PinJoint extends OneDegreeOfFreedomJoint
{
   private static final long serialVersionUID = -8016564065453170730L;

   private AxisAngle4d axisAngle = new AxisAngle4d();
   public DoubleYoVariable q, qd, qdd, tau;
   
   public DoubleYoVariable tauJointLimit, tauVelocityLimit, tauDamping;
   public double q_min = Double.NEGATIVE_INFINITY, q_max = Double.POSITIVE_INFINITY, k_limit, b_limit;

   private DoubleYoVariable b_damp, f_stiction;
   public DoubleYoVariable qd_max, b_vel_limit;
   public DoubleYoVariable tau_max;

   private YoVariableRegistry registry;

   public TorqueSpeedCurve torqueSpeedCurve;

   /**
    * Creates a new pin joint and adds it to the specified robot.  There are three possible axis of rotation
    * for this method: X, Y, and Z.  To specified a particular axis use the public int provided in Joint.
    * 
    * @param jname name of this joint
    * @param offset Vector3d representing the offset from the robot origin to the joint.
    * @param rob Robot to which this joint will belong
    * @param jaxis int representing the axis
    */
   public PinJoint(String jname, Vector3d offset, Robot rob, Axis jaxis)
   {
      super(jname, offset, rob);
      physics = new PinJointPhysics(this);

      registry = rob.getRobotsYoVariableRegistry();

      initializeYoVariables(jname, registry);

      this.physics.u_i = new Vector3d();

      if (jaxis == Axis.X)
      {
         physics.u_i.x = 1.0;
      }
      else if (jaxis == Axis.Y)
      {
         physics.u_i.y = 1.0;
      }
      else if (jaxis == Axis.Z)
      {
         physics.u_i.z = 1.0;
      }
      else
      {
         throw new RuntimeException("Undefined jaxis value!");
      }

      this.setPinTransform3D(this.jointTransform3D, physics.u_i);
   }

   /**
    * Creates a new pin joint and adds it to the specified robot.  This method allows the specification
    * of an arbitrary joint axis.
    *
    * @param jname name of this joint
    * @param offset Vector3d representing the offset from the robot origin to the joint.
    * @param rob Robot to which this joint will belong
    * @param u_hat Vector3d representing the axis of rotation
    */
   public PinJoint(String jname, Vector3d offset, Robot rob, Vector3d u_hat)
   {
      super(jname, offset, rob);
      physics = new PinJointPhysics(this);

      registry = rob.getRobotsYoVariableRegistry();

      initializeYoVariables(jname, registry);

      physics.u_i = new Vector3d();
      physics.u_i.set(u_hat);
      physics.u_i.normalize();
      setPinTransform3D(this.jointTransform3D, physics.u_i);
   }

   /**
    * This function updates the transform, velocity, and joint axis.  If specified
    * the graphics are also updated, however, this is nolonger the primary means of
    * graphics updates.
    */
   protected void update()
   {
      this.setPinTransform3D(this.jointTransform3D, physics.u_i, q.getDoubleValue());
   }

   /**
    * Specify the intial position and velocity of this joint.  This is used by gimbal, universal, and cylinder
    * joints to initialize their member pin joints.
    *
    * @param q_init intial position of this joint in radians.
    * @param qd_init intial velocity of this joint
    */
   public void setInitialState(double q_init, double qd_init)
   {
      q.set(q_init);
      qd.set(qd_init);
   }

   /**
    * Inserts the given position and velocity of this pin joint into the provided array.  Index zero contains
    * the position (angle) in radians, while index one contains the velocity.
    * @param state double[]
    */
   public void getState(double[] state)
   {
      state[0] = q.getDoubleValue();
      state[1] = qd.getDoubleValue();
   }

   /**
    * Sets the torque applied to this joint.  Usually these variables are accessed by name instead of going through
    * the joint.
    *
    * @param tau torque to be applied at this joint.
    */
   public void setTau(double tau)
   {
      if (Double.isNaN(tau))
      {
         throw new RuntimeException(getName() + " tau = NaN.");
      }

      this.tau.set(tau);
   }


   /**
    *    Add the torque given in parameter to this joint.
    *
    *    @param tau torque to be added to this joint.
    */

   public void addTau(double tau)
   {
      if (Double.isNaN(tau))
      {
         throw new RuntimeException("tau = NaN.");
      }

      this.tau.add(tau);
   }

   /**
    * Retrieve the current angle (position) of this joint.
    *
    * @return YoVariable representing the angle of this joint.
    */
   public DoubleYoVariable getQ()
   {
      return q;
   }

   /**
    * Retrieves the current velocity of this joint.
    *
    * @return YoVariable representing the current angle of this joint.
    */
   public DoubleYoVariable getQD()
   {
      return qd;
   }

   /**
    * Retrieves the current acceleration at this joint.
    *
    * @return YoVariable representing the current acceleration
    */
   public DoubleYoVariable getQDD()
   {
      return qdd;
   }

   /**
    * Retrieves the torque currently applied at this joint.
    *
    * @return YoVariable representing the currently applied torque.
    */
   public DoubleYoVariable getTau()
   {
      return tau;
   }

   public void setQ(double q)
   {
      if (Double.isNaN(q))
      {
         throw new RuntimeException("q = NaN.");
      }

      this.q.set(q);
   }

   public void setQd(double qd)
   {
      if (Double.isNaN(qd))
      {
         throw new RuntimeException("qd = NaN.");
      }

      this.qd.set(qd);
   }
   
   public void setQdd(double qdd)
   {
      if (Double.isNaN(qdd))
      {
         throw new RuntimeException("qdd = NaN.");
      }

      this.qdd.set(qdd);
   }

   /**
    * <p>Adds a set of limit stops to this joint.  This defines the allowed range of motion based on the provided
    * min and max angles.  Motion is constrained between these two values using a reaction torque calculated
    * using the given spring and damper constants.</p>
    *
    * @param q_min minimum allowed angle for this joint.
    * @param q_max maximum allowed angle for this joint.
    * @param k_limit spring constant used in torque calculations
    * @param b_limit damping constant used in torque calculations
    */
   public void setLimitStops(double q_min, double q_max, double k_limit, double b_limit)
   {
      if (tauJointLimit == null)
      {
         tauJointLimit = new DoubleYoVariable("tau_joint_limit_" + this.name, "PinJoint limit stop torque", registry);
      }

      this.q_min = q_min;
      this.q_max = q_max;
      
      if (q_min >= q_max)
         throw new RuntimeException("q_min must be less than q_max. q_min=" + q_min + ", q_max=" + q_max);
      
      this.k_limit = k_limit;
      this.b_limit = b_limit;
   }

   /**
    * Adds a velocity limit for this joint.  This is achieved through the application of a resistive torque calculated based on the
    * velocity and the provided damping function.
    *
    * @param qd_max maximum allowed velocity
    * @param b_vel_limit damping constant for torque calculations.
    */
   public void setVelocityLimits(double qd_max, double b_vel_limit)
   {
      if (tauVelocityLimit == null)
      {
         tauVelocityLimit = new DoubleYoVariable("tau_vel_limit_" + this.name, "PinJoint velocity limit torque", registry);
         this.b_vel_limit = new DoubleYoVariable("b_vel_limit_" + this.name, "PinJoint damping after maximum angular velocity is reached", registry);
         this.qd_max = new DoubleYoVariable("qd_max_" + this.name, "PinJoint maximum angular velocity", registry);


      }

      this.qd_max.set(qd_max);
      this.b_vel_limit.set(b_vel_limit);
   }


   public void setTorqueSpeedCurve(TorqueSpeedCurve torqueSpeedCurve)
   {
      this.torqueSpeedCurve = torqueSpeedCurve;
   }

   /**
    * Sets a threshold for maximum allowed torque at this joint.  If a torque larger than the specified value is desired
    * it will be cut to the given value.
    *
    * @param maxTorque torque limit.
    */
   public void setTorqueLimits(double maxTorque)
   {
      if (tau_max == null)
      {
         tau_max = new DoubleYoVariable("tau_max_" + this.name, "PinJoint maximum torque", registry);
      }

      this.tau_max.set(Math.abs(maxTorque));
   }

   /**
    * Specifies the overall damping constant for this joint.  If specified a torque base on this constant
    * and the current velocity will be applied.
    *
    * @param b_damp general damping constant for this joint
    */
   public void setDamping(double b_damp)
   {      
      if (tauDamping == null)
      {
         tauDamping = new DoubleYoVariable("tau_damp_" + this.name, "PinJoint damping torque", registry);
      }
      
      if (this.b_damp == null)
      {
         this.b_damp = new DoubleYoVariable("b_damp_" + this.name, "PinJoint damping parameter", registry);
         this.b_damp.set(b_damp);
      }
   }
   
   public void setStiction(double f_stiction)
   {
      if (tauDamping == null)
      {
         tauDamping = new DoubleYoVariable("tau_damp_" + this.name, "PinJoint damping torque", registry);
      }
      if (this.f_stiction == null)
      {
         this.f_stiction = new DoubleYoVariable("f_stiction_" + this.name, "PinJoint stiction force", registry);
         this.f_stiction.set(f_stiction);
      }
   }

   /**
    * Updates the transformation matrix tl based on the given rotation axis assuming a joint angle of zero.
    *
    * @param t1 Transform3D in which the transform is to be stored
    * @param u_i Vector3d representing the joint axis
    */
   protected void setPinTransform3D(RigidBodyTransform t1, Vector3d u_i)    // int rotAxis)
   {
      setPinTransform3D(t1, u_i, 0.0);    // rotAxis, 0.0);
   }

   /**
    * Updates the transformation matrix tl using the given joint axis and rotation angle.
    *
    * @param t1 Transform3D in which the transform is to be stored.
    * @param u_i Vector3d representing the joint axis
    * @param rotAng double specified rotation angle.
    */
   protected void setPinTransform3D(RigidBodyTransform t1, Vector3d u_i, double rotAng)
   {
      t1.setIdentity();
      axisAngle.set(u_i, rotAng);
      t1.setRotation(axisAngle);
   }

   /**
    * Initializes the YoVariables relevant for this joint.
    * @param jname the name of the joint
    * @param registry the YoVariableRegistry to which the YoVariables should be added.
    */
   protected void initializeYoVariables(String jname, YoVariableRegistry registry)
   {
      q = new DoubleYoVariable("q_" + jname, "PinJoint angle", registry);
      qd = new DoubleYoVariable("qd_" + jname, "PinJoint anglular velocity", registry);
      qdd = new DoubleYoVariable("qdd_" + jname, "PinJoint angular acceleration", registry);
      tau = new DoubleYoVariable("tau_" + jname, "PinJoint torque", registry);
   }
   
   public void setDampingParameterOnly(double b_damp) // Hack for Gazebo
   {
      if (this.b_damp != null) 
         this.b_damp.set(b_damp);
   }
   
   public void setStictionParameterOnly(double f_stiction) // Hack for Gazebo
   {
      if (this.f_stiction != null)
         this.f_stiction.set(f_stiction);
   }

   @Override
   public double getDamping()
   {
      if (b_damp == null) return 0.0;
      return b_damp.getDoubleValue();
   }

   @Override
   public double getTorqueLimit()
   {
      if(tau_max != null)
      {
         return tau_max.getDoubleValue();
      }
      else
      {
         return Double.POSITIVE_INFINITY;
      }
   }

   @Override
   public double getVelocityLimit()
   {
      if(qd_max != null)
      {
         return qd_max.getDoubleValue();
      }
      else
      {
         return Double.POSITIVE_INFINITY;
      }
   }
   
   @Override
   public double getJointUpperLimit()
   {
      return q_max;
   }
   
   @Override
   public double getJointLowerLimit()
   {
      return q_min;
   }
   
   @Override
   public double getJointStiction()
   {
      if (f_stiction == null) return 0.0;
      return f_stiction.getDoubleValue();
   }
}
