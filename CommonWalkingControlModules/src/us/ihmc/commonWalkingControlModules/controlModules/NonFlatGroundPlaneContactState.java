package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

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

   public List<FramePoint> getCopyOfContactFramePointsInContact()
   {
      return contactFramePoints;
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

   public List<FramePoint2d> getCopyOfContactFramePoints2dInContact()
   {
      return contactFramePoints2d;
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
	   frameVectorToPack.setAndChangeFrame(contactNormalFrameVector);
   }
   
   public void setRhoContactRegularization(double wRho)
   {
   }

   public double getRhoContactRegularization()
   {
      return DEFAULT_WRHO;
   }

   public void resetContactRegularization()
   {
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
