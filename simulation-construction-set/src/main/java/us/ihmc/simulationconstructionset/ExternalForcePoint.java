package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.simulationconstructionset.physics.engine.featherstone.JointPhysics;

public class ExternalForcePoint extends KinematicPoint
{
   private static final long serialVersionUID = -7715587266631433612L;

   // The force, impulse, and moment are all in world frame.
   private final YoFrameVector3D force;
   private final YoFrameVector3D moment;
   private final YoFrameVector3D impulse;

   private RotationMatrix R0_coll = new RotationMatrix(), Rcoll_0 = new RotationMatrix(), Rk_coll = new RotationMatrix(), Rk_coll2 = new RotationMatrix();
   private Vector3D u_coll = new Vector3D();
   private Matrix3D Ki_collision_total = new Matrix3D();

   private final Vector3D zAxis = new Vector3D(), yAxis = new Vector3D(), xAxis = new Vector3D();

   public boolean active = false;

   public ExternalForcePoint(String name, Robot robot)
   {
      this(name, null, robot.getRobotsYoVariableRegistry());
   }

   public ExternalForcePoint(String name, YoVariableRegistry registry)
   {
      this(name, null, registry);
   }

   /**
    * @param name
    * @param offset in world when all of the robot's joints are at zero
    * @param robot
    */
   public ExternalForcePoint(String name, Vector3DReadOnly offset, Robot robot)
   {
      this(name, offset, robot.getRobotsYoVariableRegistry());
   }

   /**
    * @param name
    * @param offset in world when all of the robot's joints are at zero
    * @param registry
    */
   public ExternalForcePoint(String name, Vector3DReadOnly offset, YoVariableRegistry registry)
   {
      super(name, offset, registry);

      force = new YoFrameVector3D(name + "_f", "", ReferenceFrame.getWorldFrame(), registry);
      moment = new YoFrameVector3D(name + "_m", "", ReferenceFrame.getWorldFrame(), registry);
      impulse = new YoFrameVector3D(name + "_p", "", ReferenceFrame.getWorldFrame(), registry);
   }

   @Override
   public String toString()
   {
      return ("name: " + getName() + "x: " + getX() + ", y: " + getY() + ", z: " + getZ());
   }

   public boolean isForceAndMomentZero()
   {
      return ((force.getX() == 0.0) && (force.getY() == 0.0) && (force.getZ() == 0.0) && (moment.getX() == 0.0) && (moment.getY() == 0.0) && (moment.getZ() == 0.0));
   }

   @Override
   public void reset()
   {
      super.reset();

      force.setToZero();
      impulse.setToZero();

      active = false;

      R0_coll.setToZero();
      Rcoll_0.setToZero();
      Rk_coll.setToZero();
      Rk_coll2.setToZero();
      u_coll.set(0,0,0);
      Ki_collision_total.setToZero();

      zAxis.set(0,0,0);
      yAxis.set(0,0,0);
      xAxis.set(0,0,0);
   }



   public boolean resolveMicroCollision(double penetrationSquared, ExternalForcePoint externalForcePointTwo, Vector3DReadOnly negative_normal, double epsilon, double mu, Vector3DBasics p_world)
   {
      //TODO: Duplicate code all over the place here. Clean this up once it works well. Test cases too!
      epsilon = epsilon + 1000000.0 * penetrationSquared;
      if (epsilon > 20.0)
         epsilon = 20.0;

//      System.out.println("epsilon1 = " + epsilon);
//      System.out.println("penetrationSquared1 = " + penetrationSquared);
      return resolveCollision(externalForcePointTwo, negative_normal, epsilon, mu, p_world);
   }


   private final Vector3D otherObjectVelocity = new Vector3D();

   public boolean resolveCollision(ExternalForcePoint externalForcePoint, Vector3DReadOnly collisionNormalInWorld, double epsilon, double mu, Vector3DBasics impulseInWorldToPack)
   {
//      System.out.println("Resolving collision with other object");
      otherObjectVelocity.set(externalForcePoint.getXVelocity(), externalForcePoint.getYVelocity(), externalForcePoint.getZVelocity());

      boolean movingTogether = computeRotationAndRelativeVelocity(collisionNormalInWorld, otherObjectVelocity, impulseInWorldToPack);
      if (!movingTogether)
      {
         externalForcePoint.impulse.setToZero();
         return false;
      }

      Rk_coll.set(this.parentJoint.physics.Ri_0);
      Rk_coll.multiply(R0_coll);
      Matrix3D Ki_collision = parentJoint.physics.computeKiCollision(tempVectorForOffsetFromCOM, Rk_coll);

      Rk_coll2.set(externalForcePoint.parentJoint.physics.Ri_0);
      Rk_coll2.multiply(R0_coll);
      Matrix3D Ki_collision2 = externalForcePoint.parentJoint.physics.computeKiCollision(externalForcePoint.tempVectorForOffsetFromCOM, Rk_coll2);

      Ki_collision_total.add(Ki_collision, Ki_collision2);
      parentJoint.physics.integrateCollision(Ki_collision_total, u_coll, epsilon, mu, impulseInWorldToPack);    // Returns the impulse in collision coordinates
      parentJoint.physics.applyImpulse(impulseInWorldToPack);

      impulseInWorldToPack.scale(-1.0);
      JointPhysics<?> jointPhysics = externalForcePoint.parentJoint.physics;

      jointPhysics.applyImpulse(impulseInWorldToPack);    // Equal and opposite impulses;

      // Rotate into world coordinates:
      R0_coll.transform(impulseInWorldToPack);
      impulse.set(-impulseInWorldToPack.getX(), -impulseInWorldToPack.getY(), -impulseInWorldToPack.getZ());
      externalForcePoint.impulse.set(impulseInWorldToPack);

      // +++JEP.  After collision, recalculate velocities in case another collision occurs before velocities are calculated:
      parentJoint.rob.updateVelocities();
      externalForcePoint.parentJoint.rob.updateVelocities();

      return true;
   }

   public boolean resolveCollision(Vector3DReadOnly velocityOfOtherObjectInWorld, Vector3DReadOnly collisionNormalInWorld, double epsilon, double mu, Vector3DBasics impulseInWorldToPack)
   {
//      System.out.println("Resolving normal collision with ground");
      boolean movingTogether = computeRotationAndRelativeVelocity(collisionNormalInWorld, velocityOfOtherObjectInWorld, impulseInWorldToPack);
      if (!movingTogether)
      {
         return false;
      }

      Rk_coll.set(this.parentJoint.physics.Ri_0);
      Rk_coll.multiply(R0_coll);

      parentJoint.physics.resolveCollision(tempVectorForOffsetFromCOM, Rk_coll, u_coll, epsilon, mu, impulseInWorldToPack);    // Returns the impulse in collision coordinates

      // Rotate into world coordinates:
      R0_coll.transform(impulseInWorldToPack);
      impulse.set(impulseInWorldToPack);

      // +++JEP.  After collision, recalculate velocities in case another collision occurs before velocities are calculated:
      parentJoint.rob.updateVelocities();

      return true;
   }


   public boolean resolveMicroCollision(double penetrationSquared, Vector3DReadOnly velocityOfOtherObjectInWorld, Vector3DReadOnly collisionNormalInWorld, double epsilon, double mu, Vector3DBasics impulseInWorldToPack)
   {
//      System.out.println("External force point: Resolving micro collision");
      boolean movingTogether = computeRotationAndRelativeVelocity(collisionNormalInWorld, velocityOfOtherObjectInWorld, impulseInWorldToPack);
      if (!movingTogether)
      {
//         System.out.println("Not moving together...");
         return false;
      }

      Rk_coll.set(this.parentJoint.physics.Ri_0);
      Rk_coll.multiply(R0_coll);

      parentJoint.physics.resolveMicroCollision(penetrationSquared, tempVectorForOffsetFromCOM, Rk_coll, u_coll, epsilon, mu, impulseInWorldToPack);    // Returns the impulse in collision coordinates

      // Rotate into world coordinates:
      R0_coll.transform(impulseInWorldToPack);
      impulse.set(impulseInWorldToPack);

      // +++JEP.  After collision, recalculate velocities in case another collision occurs before velocities are calculated:
      parentJoint.rob.updateVelocities();

      return true;
   }

   private boolean computeRotationAndRelativeVelocity(Vector3DReadOnly collisionNormalInWorld, Vector3DReadOnly otherObjectVelocityInWorld,  Vector3DBasics impulseInWorldToPack)
   {
      computeRotationFromNormalVector(R0_coll, collisionNormalInWorld);
      Rcoll_0.set(R0_coll);
      Rcoll_0.transpose();
      u_coll.set(getXVelocity() - otherObjectVelocityInWorld.getX(), getYVelocity() - otherObjectVelocityInWorld.getY(), getZVelocity() - otherObjectVelocityInWorld.getZ());
      Rcoll_0.transform(u_coll);

      if (u_coll.getZ() > 0.0)    // -0.001) // Moving slowly together or moving apart...
      {
         impulseInWorldToPack.set(0.0, 0.0, 0.0);
         impulse.setToZero();

         return false;
      }

      return true;
   }

   public void getForce(Vector3DBasics vectorToPack)
   {
      vectorToPack.set(force);
   }

   public void getMoment(Vector3DBasics vectorToPack)
   {
      vectorToPack.set(moment);
   }

   public void setForce(Vector3DReadOnly force)
   {
      this.force.set(force);
   }

   public void setForce(double fx, double fy, double fz)
   {
      this.force.set(fx, fy, fz);
   }

   public void setMoment(Vector3D moment)
   {
      this.moment.set(moment);
   }

   public void setMoment(double mx, double my, double mz)
   {
      this.moment.set(mx, my, mz);
   }

   public void getImpulse(Vector3D vectorToPack)
   {
      vectorToPack.set(impulse);
   }

   public void setImpulse(Vector3D impulse)
   {
      this.impulse.set(impulse);
   }

   public void setImpulse(double px, double py, double pz)
   {
      this.impulse.set(px, py, pz);
   }

   public YoFrameVector3D getYoForce()
   {
      return force;
   }

   public YoFrameVector3D getYoMoment()
   {
	   return moment;
   }

   public YoFrameVector3D getYoImpulse()
   {
      return impulse;
   }

   private void computeRotationFromNormalVector(RotationMatrix rot, Vector3DReadOnly vec)
   {
      // Z axis points in direction of vec...
      zAxis.set(vec);
      zAxis.normalize();

      yAxis.set(0.0, 1.0, 0.0);

      // if (Math.abs(yAxis.dot(zAxis)) > 0.99){yAxis = new Vector3d(1.0,0.0,0.0);}
//    if (Math.abs(zAxis.y) > 0.99){yAxis = new Vector3d(1.0,0.0,0.0);}
      if (Math.abs(zAxis.getY()) > 0.99)
      {
         yAxis.set(1.0, 0.0, 0.0);
      }

      xAxis.cross(yAxis, zAxis);
      xAxis.normalize();

      yAxis.cross(zAxis, xAxis);

      rot.setColumns(xAxis, yAxis, zAxis);
   }
}