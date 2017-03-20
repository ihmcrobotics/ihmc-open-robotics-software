package us.ihmc.simulationconstructionset.util.environments;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.Contactable;

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
      public boolean isClose(Point3D pointInWorldToCheck)
      {
         return contactableRobot.isClose(pointInWorldToCheck);
      }

      @Override
      public boolean isPointOnOrInside(Point3D pointInWorldToCheck)
      {
         return contactableRobot.isPointOnOrInside(pointInWorldToCheck);
      }

      @Override
      public void closestIntersectionAndNormalAt(Point3D intersectionToPack, Vector3D normalToPack, Point3D pointInWorldToCheck)
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
         boolean addYoGraphicForceVectorsForceVectors)
   {
      articulatedContactable.createAvailableContactPoints(groupIdentifier, totalContactPointsAvailable, forceVectorScale,
            addYoGraphicForceVectorsForceVectors);
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

   public void setPosition(Tuple3DBasics position)
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

   public void setVelocity(Tuple3DBasics velocity)
   {
      getFloatingJoint().setVelocity(velocity);
   }

   public void setPositionAndVelocity(double x, double y, double z, double dx, double dy, double dz)
   {
      getFloatingJoint().setPositionAndVelocity(x, y, z, dx, dy, dz);
   }

   public void setPositionAndVelocity(Point3D position, Vector3D velocity)
   {
      getFloatingJoint().setPositionAndVelocity(position, velocity);
   }

   public void setAngularVelocityInBody(Vector3D velocity)
   {
      getFloatingJoint().setAngularVelocityInBody(velocity);
   }

   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld)
   {
      getFloatingJoint().getTransformToWorld(transformToWorld);
   }

   public void getVelocity(Tuple3DBasics velocityToPack)
   {
      getFloatingJoint().getVelocity(velocityToPack);
   }

   public void getPosition(Tuple3DBasics positionToPack)
   {
      getFloatingJoint().getPosition(positionToPack);
   }
}
