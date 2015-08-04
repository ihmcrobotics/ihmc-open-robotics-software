package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class NonFlatGroundPlaneContactState implements PlaneContactState
{
   private final RigidBody rigidBody;
   private final ArrayList<FramePoint> contactFramePoints;
   private final ArrayList<FramePoint2d> contactFramePoints2d;
   private final List<ContactPoint> contactPoints;
   private final ReferenceFrame planeFrame;
   private final double coefficientOfFriction;
   private final FrameVector contactNormalFrameVector;
   private final int totalNumberOfContactPoints;
   private boolean inContact;

   public NonFlatGroundPlaneContactState(double footLength, double footWidth, Point3d midfootLocation, Vector3d normalToContactPlane,
           double coefficientOfFriction, RigidBody rigidBody)
   {
      this.rigidBody = rigidBody;
      contactFramePoints = new ArrayList<FramePoint>();
      contactFramePoints2d = new ArrayList<FramePoint2d>();
      planeFrame = ReferenceFrame.constructReferenceFrameFromPointAndZAxis("planeFrame", new FramePoint(ReferenceFrame.getWorldFrame(), midfootLocation),
              new FrameVector(ReferenceFrame.getWorldFrame(), normalToContactPlane));

      Point3d frontLeft = new Point3d(midfootLocation);
      frontLeft.setX(frontLeft.getX() + footLength / 2.0);
      frontLeft.setY(frontLeft.getY() + footWidth / 2.0);

      Point3d frontRight = new Point3d(midfootLocation);
      frontRight.setX(frontRight.getX() + footLength / 2.0);
      frontRight.setY(frontRight.getY() - footWidth / 2.0);

      Point3d backLeft = new Point3d(midfootLocation);
      backLeft.setX(backLeft.getX() - footLength / 2.0);
      backLeft.setY(backLeft.getY() + footWidth / 2.0);

      Point3d backRight = new Point3d(midfootLocation);
      backRight.setX(backRight.getX() - footLength / 2.0);
      backRight.setY(backRight.getY() - footWidth / 2.0);

      contactFramePoints.add(new FramePoint(planeFrame, frontLeft));
      contactFramePoints.add(new FramePoint(planeFrame, frontRight));
      contactFramePoints.add(new FramePoint(planeFrame, backRight));
      contactFramePoints.add(new FramePoint(planeFrame, backLeft));

      contactFramePoints2d.add(new FramePoint2d(planeFrame, projectToXY(frontLeft)));
      contactFramePoints2d.add(new FramePoint2d(planeFrame, projectToXY(frontRight)));
      contactFramePoints2d.add(new FramePoint2d(planeFrame, projectToXY(backRight)));
      contactFramePoints2d.add(new FramePoint2d(planeFrame, projectToXY(backLeft)));

      this.coefficientOfFriction = coefficientOfFriction;
      
      this.contactNormalFrameVector = new FrameVector(planeFrame, 0.0, 0.0, 1.0);

      contactPoints = new ArrayList<ContactPoint>();
      for (int i = 0; i < contactFramePoints2d.size(); i++)
      {
         ContactPoint contactPoint = new ContactPoint(contactFramePoints2d.get(i), this);
         contactPoint.setInContact(true);
         contactPoints.add(contactPoint);
      }
      
      inContact = true;
      
      totalNumberOfContactPoints = contactPoints.size();
   }

   private Point2d projectToXY(Point3d point)
   {
      return new Point2d(point.getX(), point.getY());
   }

   public List<FramePoint> getContactFramePointsInContactCopy()
   {
      List<FramePoint> ret = new ArrayList<FramePoint>();
      
      for (int i = 0; i < contactFramePoints.size(); i++)
      {
         ContactPoint contactPoint = contactPoints.get(i);
         if (!contactPoint.isInContact())
            continue;
         
         FramePoint e = new FramePoint();
         contactPoint.getPosition(e);
         ret.add(e);
      }
      return ret;
   }

   public void getContactFramePointsInContact(List<FramePoint> contactPointListToPack)
   {
      int counter = 0;
      for (int i = 0; i < contactFramePoints.size(); i++)
      {
         ContactPoint contactPoint = contactPoints.get(i);
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
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>();
      
      for (int i = 0; i < contactFramePoints2d.size(); i++)
      {
         ContactPoint contactPoint = contactPoints.get(i);
         if (!contactPoint.isInContact())
            continue;
         
         FramePoint2d e = new FramePoint2d();
         contactPoint.getPosition2d(e);
         ret.add(e);
      }
      return ret;
   }

   public ReferenceFrame getFrameAfterParentJoint()
   {
      return planeFrame;
   }

   public boolean inContact()
   {
      return inContact;
   }

   public ReferenceFrame getPlaneFrame()
   {
      return planeFrame;
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }

   public int getNumberOfContactPointsInContact()
   {
      return contactFramePoints.size();
   }

   public FrameVector getContactNormalFrameVectorCopy()
   {
      return new FrameVector(contactNormalFrameVector);
   }

   public void getContactNormalFrameVector(FrameVector frameVectorToPack)
   {
	   frameVectorToPack.setIncludingFrame(contactNormalFrameVector);
   }
   
   public List<ContactPoint> getContactPoints()
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
