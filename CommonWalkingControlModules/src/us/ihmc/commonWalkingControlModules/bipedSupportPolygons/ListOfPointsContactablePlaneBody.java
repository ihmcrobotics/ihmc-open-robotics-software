package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

public class ListOfPointsContactablePlaneBody implements ContactablePlaneBody
{
   private final RigidBody rigidBody;
   private final ReferenceFrame soleFrame;
   private final List<Point2d> contactPoints = new ArrayList<Point2d>();

   public ListOfPointsContactablePlaneBody(RigidBody rigidBody, ReferenceFrame soleFrame, List<Point2d> contactPointsInSoleFrame)
   {
      this.rigidBody = rigidBody;
      this.soleFrame = soleFrame;
      this.contactPoints.addAll(contactPointsInSoleFrame);
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }
   
   public String getName()
   {
      return rigidBody.getName();
   }

   public List<FramePoint> getContactPoints()
   {
      List<FramePoint> ret = new ArrayList<FramePoint>(contactPoints.size());
      for (Point2d point : contactPoints)
      {
         ret.add(new FramePoint(soleFrame, point.getX(), point.getY(), 0.0));
      }
   
      return ret;
   }

   public ReferenceFrame getBodyFrame()
   {
      return rigidBody.getParentJoint().getFrameAfterJoint();
   }

   public FrameConvexPolygon2d getContactPolygon()
   {
      return new FrameConvexPolygon2d(soleFrame, contactPoints);
   }

   public ReferenceFrame getPlaneFrame()
   {
      return soleFrame;
   }

   public List<FramePoint2d> getContactPoints2d()
   {
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>(contactPoints.size());
      for (Point2d point : contactPoints)
      {
         ret.add(new FramePoint2d(soleFrame, point));
      }
   
      return ret;
   }

   public boolean inContact()
   {
      throw new RuntimeException();
   }

}