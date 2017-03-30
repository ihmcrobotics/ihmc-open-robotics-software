package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.physics.engine.featherstone.SliderJointPhysics;

/**
 * Title:        Simulation Construction Set<p>
 * Description:  Package for Simulating Dynamic Robots and Mechanisms.  A translational joint with a single degree of freedom.
 * This joint allows motion up and down the specified joint axis.  This is a fundamental joint type which is currently used in
 * the implementation of cylinder joint.<p>
 * Copyright:    Copyright (c) Jerry Pratt<p>
 * @author Jerry Pratt
 * @version Beta 1.0
 */

public class SliderJoint extends OneDegreeOfFreedomJoint
{
   private static final long serialVersionUID = 1364230363983913667L;
   private final YoVariableRegistry registry;
   protected DoubleYoVariable q, qd, qdd, tau;
   public DoubleYoVariable tauJointLimit, tauDamping;

   // private int axis;
   public Vector3D vTranslate = new Vector3D();

   public double q_min = Double.NEGATIVE_INFINITY, q_max = Double.POSITIVE_INFINITY, k_limit, b_limit, b_damp = 0.0, f_stiction = 0.0;

   /**
    * Constructs a new slider joint and adds it to the specified Robot.  There are three possible axis
    * of translation for this joint: X, Y and Z.
    *
    * @param jname name of this joint
    * @param offset Vector3d defining the offset from the robot origin to the joint
    * @param rob Robot to which this joint belongs
    * @param jaxis int representing the joint axis
    */
   public SliderJoint(String jname, Vector3D offset, Robot rob, Axis jaxis)
   {
      super(jname, offset, rob);
      physics = new SliderJointPhysics(this);

      registry = rob.getRobotsYoVariableRegistry();

      q = new DoubleYoVariable("q_" + jname, "SliderJoint position", registry);
      qd = new DoubleYoVariable("qd_" + jname, "SliderJoint linear velocity", registry);
      qdd = new DoubleYoVariable("qdd_" + jname, "SliderJoint linear acceleration", registry);
      tau = new DoubleYoVariable("tau_" + jname, "SliderJoint torque", registry);

      physics.u_i = new Vector3D();

      if (jaxis == Axis.X)
      {
         physics.u_i.setX(1.0);
      }
      else if (jaxis == Axis.Y)
      {
         physics.u_i.setY(1.0);
      }
      else if (jaxis == Axis.Z)
      {
         physics.u_i.setZ(1.0);
      }
      else
      {
         throw new RuntimeException("Undefined jaxis value!");
      }

      this.setSliderTransform3D(this.jointTransform3D, physics.u_i);    // jaxis);
   }

   /**
    * Creates a new slider joint and adds it to the specified robot.  This method allows the joint axis
    * to be defined by a vector, u_hat.
    *
    * @param jname name of this joint
    * @param offset Vector3d defining the offset from the robot origin to the joint
    * @param rob Robot to which this joint belongs
    * @param u_hat Vector3d defining the joint axis in world coordinates
    */
   public SliderJoint(String jname, Vector3D offset, Robot rob, Vector3D u_hat)
   {
      super(jname, offset, rob);
      physics = new SliderJointPhysics(this);

      registry = rob.getRobotsYoVariableRegistry();

      q = new DoubleYoVariable("q_" + jname, "Slider joint displacement", registry);
      qd = new DoubleYoVariable("qd_" + jname, "Slider joint linear velocity", registry);
      qdd = new DoubleYoVariable("qdd_" + jname, "Slider joint linear acceleration", registry);
      tau = new DoubleYoVariable("tau_" + jname, "Slider joint torque", registry);

      physics.u_i = new Vector3D(u_hat);
      physics.u_i.normalize();

      this.setSliderTransform3D(this.jointTransform3D, physics.u_i);
   }


   /**
    * This function updates the transform, velocity, and joint axis.  If specified
    * the graphics are also updated, however, this is nolonger the primary means of
    * graphics updates.
    */
   @Override
   protected void update()
   {
      this.setSliderTransform3D(this.jointTransform3D, physics.u_i, q.getDoubleValue());    // axis,q.val);
   }

   /**
    * Allows the definition of positional limit stops for this joint.  These limits are enforced via torques
    * applied based on the given spring and damping coefficients.
    *
    * @param q_min minimum position for this joint
    * @param q_max maximum position for this joint
    * @param k_limit spring constant used in torque calculations
    * @param b_limit damping constant used in torque calculations
    */
   public void setLimitStops(double q_min, double q_max, double k_limit, double b_limit)
   {
      if (tauJointLimit == null)
      {
         tauJointLimit = new DoubleYoVariable("tau_joint_limit_" + this.name, "SliderJoint limit stop torque", registry);
      }

      this.q_min = q_min;
      this.q_max = q_max;
      this.k_limit = k_limit;
      this.b_limit = b_limit;
   }

   public void setDampingParameterOnly(double b_damp)    // Hack for Gazebo
   {
      this.b_damp = b_damp;
   }

   /**
    * Sets a global velocity damping constant.  This will generate a damping torque at all times.
    *
    * @param b_damp new damping constant
    */
   @Override
   public void setDamping(double b_damp)
   {
      if (tauDamping == null)
      {
         tauDamping = new DoubleYoVariable("tau_damp_" + this.name, "SliderJoint damping torque", registry);
      }

      this.b_damp = b_damp;
   }

   public void setStiction(double f_stiction)
   {
      if (tauDamping == null)
      {
         tauDamping = new DoubleYoVariable("tau_damp_" + this.name, "SliderJoint damping torque", registry);
      }

      this.f_stiction = f_stiction;
   }

   /**
    * Sets the initial velocity and position for this joint.
    *
    * @param q_init initial position
    * @param qd_init initial velocity
    */
   public void setInitialState(double q_init, double qd_init)
   {
      setQ(q_init);
      setQd(qd_init);
   }

   @Override
   public void setQ(double q)
   {
      if (Double.isNaN(q))
         throw new RuntimeException("q = NaN.");

      this.q.set(q);
   }

   @Override
   public void setQd(double qd)
   {
      if (Double.isNaN(qd))
         throw new RuntimeException("qd = NaN.");

      this.qd.set(qd);
   }

   /**
    * Retrieves the current velocity and postion storing those values into the provided double array. The position is store at index 0 while the velocity
    * is stored at index 1.
    *
    * @param state array in which the state is to be stored
    */
   public void getState(double[] state)
   {
      state[0] = q.getDoubleValue();
      state[1] = qd.getDoubleValue();
   }

   /**
    * Sets the current torque at this joint.
    *
    * @param tau torque to be applied
    */
   @Override
   public void setTau(double tau)
   {
      if (Double.isNaN(tau))
      {
         throw new RuntimeException("tau = NaN.");
      }

      this.tau.set(tau);
   }

   /**
    * Retrieves the current position of this joint.
    *
    * @return YoVariable representing the current position
    */
   @Override
   public DoubleYoVariable getQYoVariable()
   {
      return q;
   }
   
   /**
    * Retrieves the current position of this joint.
    *
    * @return YoVariable representing the current position
    */
   @Override
   public double getQ()
   {
      return q.getDoubleValue();
   }

   /**
    * Retrieves the current velocity of this joint.
    *
    * @return YoVariable representing the current velocity
    */
   @Override
   public DoubleYoVariable getQDYoVariable()
   {
      return qd;
   }
   
   /**
    * Retrieves the current velocity of this joint.
    *
    * @return YoVariable representing the current velocity
    */
   @Override
   public double getQD()
   {
      return qd.getDoubleValue();
   }

   /**
    * Retrieves the current acceleration of this joint.
    *
    * @return YoVariable representing the current acceleration
    */
   @Override
   public DoubleYoVariable getQDDYoVariable()
   {
      return qdd;
   }
   
   /**
    * Retrieves the current acceleration of this joint.
    *
    * @return YoVariable representing the current acceleration
    */
   @Override
   public double getQDD()
   {
      return qdd.getDoubleValue();
   }

   /**
    * Retrieves the torque currently applied at this joint.
    *
    * @return YoVariable representing the currently applied torque
    */
   @Override
   public DoubleYoVariable getTauYoVariable()
   {
      return tau;
   }
   
   /**
    * Retrieves the torque currently applied at this joint.
    *
    * @return YoVariable representing the currently applied torque.
    */
   @Override
   public double getTau()
   {
      return tau.getDoubleValue();
   }

   /*
    *  public static Transform3D sliderTransform3D(int axis)
    * {
    *  return sliderTransform3D(axis,0.0);
    * }
    *
    * public static Transform3D sliderTransform3D(int axis, double transVal)
    * {
    *  Transform3D tTranslate = new Transform3D();
    *  setSliderTransform3D(tTranslate,axis,transVal);
    *
    *  return tTranslate;
    * }
    */

   /**
    * Updates the transformation matrix based on the provided joint axis.  This function assumes a joint translation
    * of zero.
    *
    * @param tTranslate Transform3D in which the calculated data will be stored.
    * @param u_i Vector3d representing the joint axis
    */
   protected void setSliderTransform3D(RigidBodyTransform tTranslate, Vector3D u_i)    // int axis)
   {
      setSliderTransform3D(tTranslate, u_i, 0.0);    // axis, u_i);//0.0);
   }

   /**
    * Updates the provided transformation matrix based on the given joint axis and joint translation.
    *
    * @param tTranslate Transform3D in which the data is to be stored
    * @param u_i Vector3d representing the joint axis
    * @param transVal distance translated along the joint axis
    */
   protected void setSliderTransform3D(RigidBodyTransform tTranslate, Vector3D u_i, double transVal)    // int axis, double transVal)
   {
      // double x=0.0,y=0.0,z=0.0;

      /*
       * if (axis == Axis.X) vTranslate.x=transVal;
       *    else if (axis == Axis.Y) vTranslate.y=transVal;
       *    else if (axis == Axis.Z) vTranslate.z=transVal;
       *    else return;
       */

      vTranslate.set(u_i);
      vTranslate.scale(transVal);

      // Transform3D tTranslate = new Transform3D();
      tTranslate.setIdentity();

      // vTranslate.x = x; vTranslate.y = y; vTranslate.z = z;
      tTranslate.setTranslation(vTranslate);

      // return tTranslate;
   }

   @Override
   public void setQdd(double qdd)
   {
   }

   @Override
   public double getDamping()
   {
      return b_damp;
   }

   @Override
   public double getTorqueLimit()
   {
      // Torque limit not implemented
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getVelocityLimit()
   {
      return Double.POSITIVE_INFINITY;
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
      return f_stiction;
   }

}
