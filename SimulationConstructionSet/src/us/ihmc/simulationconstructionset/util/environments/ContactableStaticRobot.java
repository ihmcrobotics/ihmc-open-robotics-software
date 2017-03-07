package us.ihmc.simulationconstructionset.util.environments;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.NullJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.Contactable;

public abstract class ContactableStaticRobot extends Robot implements Contactable
{
   private final InternalSingleJointArticulatedContactable articulatedContactable;

   public ContactableStaticRobot(String name)
   {
      super(name);
      articulatedContactable = new InternalSingleJointArticulatedContactable(name, this); 
   }
   
   public abstract NullJoint getNullJoint();

   private static class InternalSingleJointArticulatedContactable extends SingleJointArticulatedContactable
   {
      private final ContactableStaticRobot contactableRobot;

      public InternalSingleJointArticulatedContactable(String name, ContactableStaticRobot robot)
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
         return contactableRobot.getNullJoint();
      }
   }
   
   public void createAvailableContactPoints(int groupIdentifier, int totalContactPointsAvailable, double forceVectorScale, boolean addYoGraphicForceVectorsForceVectors)
   {
      articulatedContactable.createAvailableContactPoints(groupIdentifier, totalContactPointsAvailable, forceVectorScale, addYoGraphicForceVectorsForceVectors);
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

   public void getBodyTransformToWorld(RigidBodyTransform transformToWorld)
   {
      getNullJoint().getTransformToWorld(transformToWorld);
   }

}

