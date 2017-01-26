package us.ihmc.simulationconstructionset;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.physics.engine.jerry.NullJointPhysics;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class NullJoint extends Joint
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

   private YoVariableList jointVars;

   public NullJoint(String jname, Vector3d offset, Robot rob)
   {
      super(jname, offset, rob, 0);
      physics = new NullJointPhysics(this);

      physics.u_i = new Vector3d(1.0, 0.0, 0.0);

      this.setPinTransform3D(this.jointTransform3D, physics.u_i);    // jaxis);
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

   protected void setPinTransform3D(RigidBodyTransform t1, Vector3d u_i)    // int rotAxis)
   {
      setPinTransform3D(t1, u_i, 0.0);    // rotAxis, 0.0);
   }

   private AxisAngle4d axisAngle = new AxisAngle4d();

   protected void setPinTransform3D(RigidBodyTransform t1, Vector3d u_i, double rotAng)
   {
      t1.setIdentity();

      axisAngle.set(u_i, rotAng);
      t1.setRotation(axisAngle);

      // t1.setRotation();
   }
}
