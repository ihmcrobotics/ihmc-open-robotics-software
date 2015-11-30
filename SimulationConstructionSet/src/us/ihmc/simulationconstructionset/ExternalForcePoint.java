package us.ihmc.simulationconstructionset;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class ExternalForcePoint extends KinematicPoint
{
   private static final long serialVersionUID = -7715587266631433612L;
   
   private final YoFrameVector force;
   private final YoFrameVector moment;
   private final YoFrameVector impulse;

   private Matrix3d R0_coll = new Matrix3d(), Rcoll_0 = new Matrix3d(), Rk_coll = new Matrix3d(), Rk_coll2 = new Matrix3d();
   private Vector3d u_coll = new Vector3d();
   private Matrix3d Ki_collision_total = new Matrix3d();

   private final Vector3d zAxis = new Vector3d(), yAxis = new Vector3d(), xAxis = new Vector3d();

   public boolean active = false;

   public ExternalForcePoint(String name, Robot robot)
   {
      this(name, null, robot.getRobotsYoVariableRegistry());
   }

   public ExternalForcePoint(String name, YoVariableRegistry registry)
   {
      this(name, null, registry);
   }

   public ExternalForcePoint(String name, Vector3d offset, Robot robot)
   {
      this(name, offset, robot.getRobotsYoVariableRegistry());
   }

   public ExternalForcePoint(String name, Vector3d offset, YoVariableRegistry registry)
   {
      super(name, offset, registry);
 
      force = new YoFrameVector(name + "_f", "", ReferenceFrame.getWorldFrame(), registry);
      moment = new YoFrameVector(name + "_m", "", ReferenceFrame.getWorldFrame(), registry);
      impulse = new YoFrameVector(name + "_p", "", ReferenceFrame.getWorldFrame(), registry);
   }

   public String toString()
   {
      return ("name: " + name + "x: " + getX() + ", y: " + getY() + ", z: " + getZ());
   }

   public boolean isForceZero()
   {
      return ((force.getX() == 0.0) && (force.getY() == 0.0) && (force.getZ() == 0.0) && (moment.getX() == 0.0) && (moment.getY() == 0.0) && (moment.getZ() == 0.0));
   }
   
   public void reset() 
   {
      super.reset();

      force.setToZero();
      impulse.setToZero();

      active = false;

      R0_coll.set(0);
      Rcoll_0.set(0);
      Rk_coll.set(0);
      Rk_coll2.set(0);
      u_coll.set(0,0,0);
      Ki_collision_total.set(0);

      zAxis.set(0,0,0);
      yAxis.set(0,0,0);
      xAxis.set(0,0,0);
   }


   public void resolveCollision(ExternalForcePoint externalForcePoint, Vector3d normal_world, double epsilon, double mu, Vector3d p_world)
   {
      computeRotationFromNormalVector(R0_coll, normal_world);
      Rcoll_0.set(R0_coll);
      Rcoll_0.transpose();

      u_coll.set(getXVelocity() - externalForcePoint.getXVelocity(), getYVelocity() - externalForcePoint.getYVelocity(),
                 getZVelocity() - externalForcePoint.getZVelocity());
      Rcoll_0.transform(u_coll);

      if (u_coll.z > 0.0)    // -0.001) // Moving slowly together or moving apart...
      {
         p_world.set(0.0, 0.0, 0.0);
         impulse.setToZero();
         externalForcePoint.impulse.setToZero();

         return;
      }

      Rk_coll.set(this.parentJoint.physics.Ri_0);
      Rk_coll.mul(R0_coll);
      Matrix3d Ki_collision = parentJoint.physics.computeKiCollision(offsetFromCOM, Rk_coll);

      Rk_coll2.set(externalForcePoint.parentJoint.physics.Ri_0);
      Rk_coll2.mul(R0_coll);
      Matrix3d Ki_collision2 = externalForcePoint.parentJoint.physics.computeKiCollision(externalForcePoint.offsetFromCOM, Rk_coll2);

      Ki_collision_total.add(Ki_collision, Ki_collision2);

      parentJoint.physics.integrateCollision(Ki_collision_total, u_coll, epsilon, mu, p_world);    // Returns the impulse in collision coordinates

      parentJoint.physics.applyImpulse(p_world);
      p_world.scale(-1.0);
      externalForcePoint.parentJoint.physics.applyImpulse(p_world);    // Equal and opposite impulses;

      // Rotate into world coordinates:
      R0_coll.transform(p_world);
      impulse.set(-p_world.x, -p_world.y, -p_world.z);

      externalForcePoint.impulse.set(p_world);

      // +++JEP.  After collision, recalculate velocities in case another collision occurs before velocities are calculated:
      parentJoint.rob.updateVelocities();
      externalForcePoint.parentJoint.rob.updateVelocities();
//      robot.updateVelocities();
   }

   public void resolveCollision(Vector3d vel_world, Vector3d normal_world, double epsilon, double mu, Vector3d p_world)
   {
      computeRotationFromNormalVector(R0_coll, normal_world);
      Rcoll_0.set(R0_coll);
      Rcoll_0.transpose();

      u_coll.set(getXVelocity() - vel_world.x, getYVelocity() - vel_world.y, getZVelocity() - vel_world.z);
      Rcoll_0.transform(u_coll);

      // System.out.println("normal_world: " + normal_world);
      // System.out.println("u_world: " + dx.val + ", " + dy.val + ", " + dz.val);
      // System.out.println("Rcoll_0" + Rcoll_0);

      if (u_coll.z > 0.0)    // -0.001) // Moving slowly together or moving apart...
      {
         p_world.set(0.0, 0.0, 0.0);
         impulse.setToZero();

         return;
      }

      // Rk_coll.set(Ri_0);

      /*
       * Rk_coll.set(R0_i);
       * Rk_coll.transpose();
       */
      Rk_coll.set(this.parentJoint.physics.Ri_0);

      Rk_coll.mul(R0_coll);

      parentJoint.physics.resolveCollision(offsetFromCOM, Rk_coll, u_coll, epsilon, mu, p_world);    // Returns the impulse in collision coordinates

      // Rotate into world coordinates:
      R0_coll.transform(p_world);
      impulse.set(p_world);

      // +++JEP.  After collision, recalculate velocities in case another collision occurs before velocities are calculated:
    parentJoint.rob.updateVelocities();
//      robot.updateVelocities();
   }


   public void resolveMicroCollision(double penetration_squared, Vector3d vel_world, Vector3d normal_world, double epsilon, double mu, Vector3d p_world)
   {
      computeRotationFromNormalVector(R0_coll, normal_world);
      Rcoll_0.set(R0_coll);
      Rcoll_0.transpose();

      u_coll.set(getXVelocity() - vel_world.x, getYVelocity() - vel_world.y, getZVelocity() - vel_world.z);
      Rcoll_0.transform(u_coll);

      if (u_coll.z > 0.0)    // Moving slowly together or moving apart...
      {
         p_world.set(0.0, 0.0, 0.0);

         return;
      }

      // Rk_coll.set(Ri_0);

      /*
       * Rk_coll.set(R0_i);
       * Rk_coll.transpose();
       */
      Rk_coll.set(this.parentJoint.physics.Ri_0);

      Rk_coll.mul(R0_coll);

      parentJoint.physics.resolveMicroCollision(penetration_squared, offsetFromCOM, Rk_coll, u_coll, epsilon, mu, p_world);    // Returns the impulse in collision coordinates

      // Rotate into world coordinates:
      R0_coll.transform(p_world);
      impulse.set(p_world);

      // +++JEP.  After collision, recalculate velocities in case another collision occurs before velocities are calculated:
      parentJoint.rob.updateVelocities();
//      robot.updateVelocities();
   }

   public void getForce(Vector3d vectorToPack)
   {
      force.get(vectorToPack);
   }
   
   public void getMoment(Vector3d vectorToPack)
   {
      moment.get(vectorToPack);
   }

   public void setForce(Vector3d force)
   {
      this.force.set(force);
   }

   public void setForce(double fx, double fy, double fz)
   {
      this.force.set(fx, fy, fz);
   }
   
   public void setMoment(Vector3d moment)
   {
      this.moment.set(moment);
   }

   public void setMoment(double mx, double my, double mz)
   {
      this.moment.set(mx, my, mz);
   }
   
   public void getImpulse(Vector3d vectorToPack)
   {
      impulse.get(vectorToPack);
   }

   public void setImpulse(Vector3d impulse)
   {
      this.impulse.set(impulse);
   }
   
   public void setImpulse(double px, double py, double pz)
   {
      this.impulse.set(px, py, pz);
   }
   
   public YoFrameVector getYoForce()
   {
      return force;
   }
   
   public YoFrameVector getYoMoment()
   {
	   return moment;
   }
   
   public YoFrameVector getYoImpulse()
   {
      return impulse;
   }

   private void computeRotationFromNormalVector(Matrix3d rot, Vector3d vec)
   {
      // Z axis points in direction of vec...
      zAxis.set(vec);
      zAxis.normalize();

      yAxis.set(0.0, 1.0, 0.0);

      // if (Math.abs(yAxis.dot(zAxis)) > 0.99){yAxis = new Vector3d(1.0,0.0,0.0);}
//    if (Math.abs(zAxis.y) > 0.99){yAxis = new Vector3d(1.0,0.0,0.0);}
      if (Math.abs(zAxis.y) > 0.99)
      {
         yAxis.set(1.0, 0.0, 0.0);
      }

      xAxis.cross(yAxis, zAxis);
      xAxis.normalize();

      yAxis.cross(zAxis, xAxis);

      rot.m00 = xAxis.x;
      rot.m01 = yAxis.x;
      rot.m02 = zAxis.x;
      rot.m10 = xAxis.y;
      rot.m11 = yAxis.y;
      rot.m12 = zAxis.y;
      rot.m20 = xAxis.z;
      rot.m21 = yAxis.z;
      rot.m22 = zAxis.z;

   }
}
