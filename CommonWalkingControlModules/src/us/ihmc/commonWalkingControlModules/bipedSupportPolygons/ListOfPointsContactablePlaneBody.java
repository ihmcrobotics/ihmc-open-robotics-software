package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Tuple2d;

import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;

public class ListOfPointsContactablePlaneBody implements ContactablePlaneBody
{
   private final RigidBody rigidBody;
   private final ReferenceFrame soleFrame;
   private final List<Point2d> contactPoints = new ArrayList<Point2d>();
   private final int totalNumberOfContactPoints;

   public ListOfPointsContactablePlaneBody(RigidBody rigidBody, ReferenceFrame soleFrame, List<Point2d> contactPointsInSoleFrame)
   {
      this.rigidBody = rigidBody;
      this.soleFrame = soleFrame;
      
      for(Point2d contactPoint : contactPointsInSoleFrame)
      {
         this.contactPoints.add(new Point2d(contactPoint));
      }
      
      totalNumberOfContactPoints = contactPoints.size();
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }
   
   public String getName()
   {
      return rigidBody.getName();
   }

   public List<FramePoint> getContactPointsCopy()
   {
      List<FramePoint> ret = new ArrayList<FramePoint>(contactPoints.size());
      for (int i = 0; i < contactPoints.size(); i++)
      {
         Tuple2d point = contactPoints.get(i);
         ret.add(new FramePoint(soleFrame, point.getX(), point.getY(), 0.0));
      }
   
      return ret;
   }

   public ReferenceFrame getFrameAfterParentJoint()
   {
      return rigidBody.getParentJoint().getFrameAfterJoint();
   }

   public FrameConvexPolygon2d getContactPolygonCopy()
   {
      return new FrameConvexPolygon2d(soleFrame, contactPoints);
   }

   public ReferenceFrame getSoleFrame()
   {
      return soleFrame;
   }

   public List<FramePoint2d> getContactPoints2d()
   {
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>(contactPoints.size());
      for (int i = 0; i < contactPoints.size(); i++)
      {
         Tuple2d point = contactPoints.get(i);
         ret.add(new FramePoint2d(soleFrame, point));
      }
   
      return ret;
   }

   public boolean inContact()
   {
      throw new RuntimeException();
   }

   public int getTotalNumberOfContactPoints()
   {
      return totalNumberOfContactPoints;
   }

}