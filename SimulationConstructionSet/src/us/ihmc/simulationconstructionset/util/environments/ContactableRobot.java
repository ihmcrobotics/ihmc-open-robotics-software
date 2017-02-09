package us.ihmc.simulationconstructionset.util.environments;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.Contactable;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public abstract class ContactableRobot extends Robot implements Contactable
{
   private final InternalSingleJointArticulatedContactable articulatedContactable;

   public ContactableRobot(String name)
   {
      super(name);
      articulatedContactable = new InternalSingleJointArticulatedContactable(name, this);
   }

   public abstract FloatingJoint getFloatingJoint();

   public abstract void setMass(double mass);

   public abstract void setMomentOfInertia(double Ixx, double Iyy, double Izz);

   private static class InternalSingleJointArticulatedContactable extends SingleJointArticulatedContactable
   {
      private final ContactableRobot contactableRobot;

      public InternalSingleJointArticulatedContactable(String name, ContactableRobot robot)
      {
         super(name, robot);
         this.contactableRobot = robot;
      }

      @Override
      public boolean isClose(Point3d pointInWorldToCheck)
      {
         return contactableRobot.isClose(pointInWorldToCheck);
      }

      @Override
      public boolean isPointOnOrInside(Point3d pointInWorldToCheck)
      {
         return contactableRobot.isPointOnOrInside(pointInWorldToCheck);
      }

      @Override
      public void closestIntersectionAndNormalAt(Point3d intersectionToPack, Vector3d normalToPack, Point3d pointInWorldToCheck)
      {
         contactableRobot.closestIntersectionAndNormalAt(intersectionToPack, normalToPack, pointInWorldToCheck);
      }

      @Override
      public Joint getJoint()
      {
         return contactableRobot.getFloatingJoint();
      }
   }

   public void createAvailableContactPoints(int groupIdentifier, int totalContactPointsAvailable, double forceVectorScale,
         boolean addDynamicGraphicForceVectorsForceVectors)
   {
      articulatedContactable.createAvailableContactPoints(groupIdentifier, totalContactPointsAvailable, forceVectorScale,
            addDynamicGraphicForceVectorsForceVectors);
   }

   @Override
   public int getAndLockAvailableContactPoint()
   {
      return articulatedContactable.getAndLockAvailableContactPoint();
   }

   @Override
   public void unlockContactPoint(GroundContactPoint groundContactPoint)
   {
      articulatedContactable.unlockContactPoint(groundContactPoint);
   }

   @Override
   public GroundContactPoint getLockedContactPoint(int contactPointIndex)
   {
      return articulatedContactable.getLockedContactPoint(contactPointIndex);
   }

   @Override
   public void updateContactPoints()
   {
      articulatedContactable.updateContactPoints();
   }

   public void setPosition(double x, double y, double z)
   {
      getFloatingJoint().setPosition(x, y, z);
   }

   public void setPosition(double[] position)
   {
      getFloatingJoint().setPosition(position[0], position[1], position[2]);
   }

   public void setPosition(Tuple3d position)
   {
      getFloatingJoint().setPosition(position);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      getFloatingJoint().setYawPitchRoll(yaw, pitch, roll);
   }

   public void setVelocity(double xd, double yd, double zd)
   {
      getFloatingJoint().setVelocity(xd, yd, zd);
   }

   public void setVelocity(Tuple3d velocity)
   {
      getFloatingJoint().setVelocity(velocity);
   }

   public void setPositionAndVelocity(double x, double y, double z, double dx, double dy, double dz)
   {
      getFloatingJoint().setPositionAndVelocity(x, y, z, dx, dy, dz);
   }

   public void setPositionAndVelocity(Point3d position, Vector3d velocity)
   {
      getFloatingJoint().setPositionAndVelocity(position, velocity);
   }

   public void setAngularVelocityInBody(Vector3d velocity)
   {
      getFloatingJoint().setAngularVelocityInBody(velocity);
   }

   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld)
   {
      getFloatingJoint().getTransformToWorld(transformToWorld);
   }

   public void getVelocity(Tuple3d velocityToPack)
   {
      getFloatingJoint().getVelocity(velocityToPack);
   }

   public void getPosition(Tuple3d positionToPack)
   {
      getFloatingJoint().getPosition(positionToPack);
   }
}
