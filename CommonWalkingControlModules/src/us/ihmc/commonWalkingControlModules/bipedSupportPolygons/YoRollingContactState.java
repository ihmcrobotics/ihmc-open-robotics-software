package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.RigidBody;


public class YoRollingContactState implements PlaneContactState, ModifiableContactState
{
   private final RigidBody rigidBody;
   private final YoVariableRegistry registry;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame updatableContactFrame;
   private final RigidBodyTransform transformFromContactFrameToBodyFrame = new RigidBodyTransform();
   private final BooleanYoVariable inContact;
   private final DoubleYoVariable coefficientOfFriction;
   private final FrameVector contactNormalFrameVector;
   private final List<YoContactPoint> contactPoints = new ArrayList<YoContactPoint>();
   private final ContactableRollingBody contactableCylinderBody;
   private final int totalNumberOfContactPoints;

   // Class enabling to update the contact points of a contactable rolling body as it is rolling on the ground or on another contactable surface
   public YoRollingContactState(String namePrefix, ContactableRollingBody contactableCylinderBody, List<FramePoint> contactFramePoints, YoVariableRegistry parentRegistry)
   {
      // The rolling contactable body
      this.contactableCylinderBody = contactableCylinderBody;
      this.rigidBody = contactableCylinderBody.getRigidBody();
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.inContact = new BooleanYoVariable(namePrefix + "InContact", registry);
      this.coefficientOfFriction = new DoubleYoVariable(namePrefix + "CoefficientOfFriction", registry);
      this.updatableContactFrame = new ReferenceFrame(namePrefix + "ContactFrame", getFrameAfterParentJoint())
      {
         private static final long serialVersionUID = 6993243554111815201L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformFromContactFrameToBodyFrame);
         }
      };
      
      updateContactPoints();
      
      parentRegistry.addChild(registry);
      
      this.contactNormalFrameVector = new FrameVector(updatableContactFrame, 0.0, 0.0, 1.0);

      FramePoint tempFramePoint = new FramePoint(updatableContactFrame);
      
      for (int i = 0; i < contactFramePoints.size(); i++)
      {
         tempFramePoint.setIncludingFrame(contactFramePoints.get(i));
         tempFramePoint.changeFrame(updatableContactFrame);
         YoContactPoint contactPoint = new YoContactPoint(namePrefix, i, tempFramePoint, this, parentRegistry);
         contactPoints.add(contactPoint);
      }

      totalNumberOfContactPoints = contactPoints.size();
   }

   public void setContactPointsInContact(boolean[] inContact)
   {
      if (inContact.length != totalNumberOfContactPoints)
         throw new RuntimeException("Arrays should be of same length!");

      this.inContact.set(false);
      
      for (int i = 0; i < inContact.length; i++)
      {
         contactPoints.get(i).setInContact(inContact[i]);
         
         if (inContact[i])
         {
            this.inContact.set(true);
         }
      }
   }

   public void setContactPointInContact(int contactPointIndex, boolean inContact)
   {
      contactPoints.get(contactPointIndex).setInContact(inContact);

      if (inContact)
      {
         this.inContact.set(true);
      }
      else
      {
         this.inContact.set(false);
         for (int i = 0; i < totalNumberOfContactPoints; i++)
         {
            if (contactPoints.get(i).isInContact())
            {
               this.inContact.set(true);
            }
         }
      }
   }
   
   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      if (coefficientOfFriction < 0.0)
         throw new RuntimeException("Coefficient of friction is negative: " + coefficientOfFriction);
      
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   public void updateContactPoints()
   {
      // The contact reference frame is updated such as:
      // 1- it remains tangential to the contactable cylindrical body,
      // 2- it remains under the contactable cylindrical body (at the lowest height)
      RigidBodyTransform transformFromRigiBodyToWorld = getFrameAfterParentJoint().getTransformToDesiredFrame(worldFrame );
      Matrix3d rotationFromRigiBodyToWorld = new Matrix3d();
      transformFromRigiBodyToWorld.get(rotationFromRigiBodyToWorld);

      // Look for the angle theta that will position the reference contact frame. The contact points will be automatically positioned as they are expressed in that reference frame.
      // Ex. for the thigh: theta == 0 => back of the thigh, theta == PI/2 => left side of the thigh (whatever it is the left or right thigh)
      double theta = -Math.PI / 2.0 + Math.atan2(rotationFromRigiBodyToWorld.m20, rotationFromRigiBodyToWorld.m21);

      transformFromContactFrameToBodyFrame.setIdentity();
      transformFromContactFrameToBodyFrame.setEuler(theta, Math.PI / 2.0, 0.0);
      FramePoint originInBodyFrame = contactableCylinderBody.getCylinderOriginCopy();
      double cylinderRadius = contactableCylinderBody.getCylinderRadius();
      transformFromContactFrameToBodyFrame.setTranslation(-cylinderRadius  * Math.cos(theta) + originInBodyFrame .getX(), cylinderRadius * Math.sin(theta)
            + originInBodyFrame.getY(), originInBodyFrame.getZ());

      updatableContactFrame.update();
   }
   
   public List<FramePoint> getContactFramePointsInContactCopy()
   {
      List<FramePoint> ret = new ArrayList<FramePoint>(totalNumberOfContactPoints);

      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);

         if (contactPoint.isInContact())
         {
            FramePoint2d framePoint2d = new FramePoint2d();
            contactPoint.getPosition2d(framePoint2d);
            FramePoint framePoint = new FramePoint(framePoint2d);
            ret.add(framePoint);
         }
      }

      return ret;
   }

   public void getContactFramePointsInContact(List<FramePoint> contactPointListToPack)
   {
      int counter = 0;
      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);

         if (!contactPoint.isInContact())
            continue;
         
         if (counter >= contactPointListToPack.size())
            contactPointListToPack.add(new FramePoint());
         
         contactPoint.getPosition(contactPointListToPack.get(counter));
      }
      
      for (int i = contactPointListToPack.size() - 1; i >= counter; i--)
      {
         contactPointListToPack.remove(i);
      }
   }

   public List<FramePoint2d> getContactFramePoints2dInContactCopy()
   {
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>(totalNumberOfContactPoints);
      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);
         if (contactPoint.isInContact())
         {
            FramePoint2d e = new FramePoint2d();
            contactPoint.getPosition2d(e);
            ret.add(e);
         }
      }

      return ret;
   }

   public ReferenceFrame getFrameAfterParentJoint()
   {
      return rigidBody.getParentJoint().getFrameAfterJoint();
   }

   public ReferenceFrame getPlaneFrame()
   {
      return updatableContactFrame;
   }

   public boolean inContact()
   {
      return inContact.getBooleanValue();
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction.getDoubleValue();
   }

   public int getNumberOfContactPointsInContact()
   {
      int numberOfContactPointsInContact = 0;
      
      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         if (contactPoints.get(i).isInContact())
            numberOfContactPointsInContact++;
      }
      
      return numberOfContactPointsInContact;
   }

   public FrameVector getContactNormalFrameVectorCopy()
   {
      return new FrameVector(contactNormalFrameVector);
   }

   public void getContactNormalFrameVector(FrameVector frameVectorToPack)
   {
	   frameVectorToPack.setIncludingFrame(contactNormalFrameVector);
   }
   
   public void clear()
   {
      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         contactPoints.get(i).setInContact(false);
      }

      inContact.set(false);
   }
   
   public void setFullyConstrained()
   {
      for (int i = 0; i < totalNumberOfContactPoints; i++)
      {
         contactPoints.get(i).setInContact(true);
      }
      
      inContact.set(true);
   }

   public List<YoContactPoint> getContactPoints()
   {
      return contactPoints;
   }

   public int getTotalNumberOfContactPoints()
   {
      return totalNumberOfContactPoints;
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }
}
