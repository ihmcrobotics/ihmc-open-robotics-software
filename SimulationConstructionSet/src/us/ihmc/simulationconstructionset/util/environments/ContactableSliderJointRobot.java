package us.ihmc.simulationconstructionset.util.environments;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.util.ground.Contactable;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public abstract class ContactableSliderJointRobot extends Robot implements Contactable
{
   private static final long serialVersionUID = -337148945439165270L;
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

      public boolean isClose(Point3d pointInWorldToCheck)
      {
         return contactableRobot.isClose(pointInWorldToCheck);
      }

      public boolean isPointOnOrInside(Point3d pointInWorldToCheck)
      {
         return contactableRobot.isPointOnOrInside(pointInWorldToCheck);
      }

      public void closestIntersectionAndNormalAt(Point3d intersectionToPack, Vector3d normalToPack, Point3d pointInWorldToCheck)
      {
         contactableRobot.closestIntersectionAndNormalAt(intersectionToPack, normalToPack, pointInWorldToCheck);
      }

      public Joint getJoint()
      {
         return contactableRobot.getSliderJoint();
      }
   }
   
   public void createAvailableContactPoints(int groupIdentifier, int totalContactPointsAvailable, double forceVectorScale, boolean addDynamicGraphicForceVectorsForceVectors)
   {
      articulatedContactable.createAvailableContactPoints(groupIdentifier, totalContactPointsAvailable, forceVectorScale, addDynamicGraphicForceVectorsForceVectors);
   }

   public int getAndLockAvailableContactPoint()
   {
      return articulatedContactable.getAndLockAvailableContactPoint();
   }

   public void unlockContactPoint(GroundContactPoint groundContactPoint)
   {
      articulatedContactable.unlockContactPoint(groundContactPoint);
   }

   public GroundContactPoint getLockedContactPoint(int contactPointIndex)
   {
      return articulatedContactable.getLockedContactPoint(contactPointIndex);
   }

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
      return getSliderJoint().getQD().getDoubleValue();
   }

   public double getPosition()
   {
      return getSliderJoint().getQ().getDoubleValue();
   }

}

