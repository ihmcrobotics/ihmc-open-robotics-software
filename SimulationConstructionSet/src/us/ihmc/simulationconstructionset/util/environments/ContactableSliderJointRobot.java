package us.ihmc.simulationconstructionset.util.environments;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.util.ground.Contactable;

public abstract class ContactableSliderJointRobot extends Robot implements Contactable
{
   private final InternalSingleJointArticulatedContactable articulatedContactable;

   public ContactableSliderJointRobot(String name)
   {
      super(name);
      articulatedContactable = new InternalSingleJointArticulatedContactable(name, this); 
   }
   
   public abstract SliderJoint getSliderJoint();
   
   public abstract void getBodyTransformToWorld(RigidBodyTransform transformToWorld);

   public abstract void setMass(double mass);

   public abstract void setMomentOfInertia(double Ixx, double Iyy, double Izz);

   private static class InternalSingleJointArticulatedContactable extends SingleJointArticulatedContactable
   {
      private final ContactableSliderJointRobot contactableRobot;

      public InternalSingleJointArticulatedContactable(String name, ContactableSliderJointRobot robot)
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
         return contactableRobot.getSliderJoint();
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

   public void setPosition(double position)
   {
      getSliderJoint().setQ(position);
   }

   public void setVelocity(double positionDerivative)
   {
      getSliderJoint().setQd(positionDerivative);
   }

   public void setPositionAndVelocity(double position, double positionDerivative)
   {
      setPosition(position);
      setVelocity(positionDerivative);
   }

   public double getVelocity()
   {
      return getSliderJoint().getQDYoVariable().getDoubleValue();
   }

   public double getPosition()
   {
      return getSliderJoint().getQYoVariable().getDoubleValue();
   }

}

