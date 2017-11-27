package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariableList;
import us.ihmc.simulationconstructionset.physics.engine.featherstone.DummyOneDegreeOfFreedomJointPhysics;

public class DummyOneDegreeOfFreedomJoint extends OneDegreeOfFreedomJoint
{
   /**
    *
    */
   private static final long serialVersionUID = 1341493615657008348L;
   @SuppressWarnings("unused")
   private boolean hasLimitStops = false;
   @SuppressWarnings("unused")
   private double q_min, q_max, k_limit, b_limit, b_damp;

   @SuppressWarnings("unused")
   private boolean hasVelocityLimits = false;
   @SuppressWarnings("unused")
   private double qd_max, b_vel_limit;

   @SuppressWarnings("unused")
   private boolean hasTorqueLimits = false;
   @SuppressWarnings("unused")
   private double tau_max;

   private final YoDouble q, qd, qdd, tau;

   private YoVariableList jointVars;

   public DummyOneDegreeOfFreedomJoint(String jname, Vector3DReadOnly offset, Robot rob, Vector3DReadOnly u_hat)
   {
      super(jname, offset, rob);
      physics = new DummyOneDegreeOfFreedomJointPhysics(this);

      physics.u_i = new Vector3D();
      physics.u_i.set(u_hat);
      physics.u_i.normalize();

      YoVariableRegistry registry = rob.getRobotsYoVariableRegistry();
      q = new YoDouble("q_" + jname, "PinJoint angle", registry);
      qd = new YoDouble("qd_" + jname, "PinJoint anglular velocity", registry);
      qdd = new YoDouble("qdd_" + jname, "PinJoint angular acceleration", registry);
      tau = new YoDouble("tau_" + jname, "PinJoint torque", registry);

      this.setPinTransform3D(this.jointTransform3D, physics.u_i); // jaxis);
   }

   protected YoVariableList getJointVars()
   {
      return this.jointVars;
   }

   @Override
   protected void update()
   {
      this.jointTransform3D.setIdentity();
   }

   protected void setPinTransform3D(RigidBodyTransform t1, Vector3D u_i) // int rotAxis)
   {
      setPinTransform3D(t1, u_i, 0.0); // rotAxis, 0.0);
   }

   private AxisAngle axisAngle = new AxisAngle();

   protected void setPinTransform3D(RigidBodyTransform t1, Vector3D u_i, double rotAng)
   {
      t1.setIdentity();

      axisAngle.set(u_i, rotAng);
      t1.setRotation(axisAngle);

      // t1.setRotation();
   }

   @Override
   public YoDouble getQDDYoVariable()
   {
      return qdd;
   }

   @Override
   public YoDouble getQDYoVariable()
   {
      return qd;
   }

   @Override
   public YoDouble getQYoVariable()
   {
      return q;
   }

   @Override
   public void setQdd(double qdd)
   {
   }

   @Override
   public void setQd(double qd)
   {

   }

   @Override
   public void setQ(double q)
   {

   }

   @Override
   public void setTau(double tau)
   {
   }

   @Override
   public YoDouble getTauYoVariable()
   {
      return tau;
   }

   @Override
   public double getDamping()
   {
      return 0;
   }

   @Override
   public void setDamping(double b_damp)
   {

   }

   @Override
   public double getTorqueLimit()
   {
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
      return Double.POSITIVE_INFINITY;
   }
   
   @Override
   public double getJointLowerLimit()
   {
      return Double.NEGATIVE_INFINITY;
   }
   
   @Override
   public double getJointStiction()
   {
      return 0.0;
   }

   @Override
   public double getQDD()
   {
      return qdd.getDoubleValue();
   }

   @Override
   public double getQD()
   {
      return qd.getDoubleValue();
   }

   @Override
   public double getQ()
   {
      return q.getDoubleValue();
   }

   @Override
   public double getTau()
   {
      return tau.getDoubleValue();
   }
}
