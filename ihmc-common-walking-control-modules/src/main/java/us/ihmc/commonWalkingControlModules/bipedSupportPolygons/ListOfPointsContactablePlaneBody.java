package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;

public class ListOfPointsContactablePlaneBody implements ContactablePlaneBody
{
   private final RigidBodyBasics rigidBody;
   private final ReferenceFrame soleFrame;
   private final List<Point2D> contactPoints = new ArrayList<Point2D>();
   private final List<FramePoint2D> frameContactPoints = new ArrayList<FramePoint2D>();
   private final int totalNumberOfContactPoints;

   public ListOfPointsContactablePlaneBody(RigidBodyBasics rigidBody, ReferenceFrame soleFrame, List<Point2D> contactPointsInSoleFrame)
   {
      this.rigidBody = rigidBody;
      this.soleFrame = soleFrame;

      for (Point2D contactPoint : contactPointsInSoleFrame)
      {
         Point2D point = new Point2D(contactPoint);
         this.contactPoints.add(point);
         frameContactPoints.add(new FramePoint2D(soleFrame, point));
      }
      

      totalNumberOfContactPoints = contactPoints.size();
   }

   @Override
   public RigidBodyBasics getRigidBody()
   {
      return rigidBody;
   }

   @Override
   public String getName()
   {
      return rigidBody.getName();
   }

   @Override
   public List<FramePoint3D> getContactPointsCopy()
   {
      List<FramePoint3D> ret = new ArrayList<FramePoint3D>(contactPoints.size());
      for (int i = 0; i < contactPoints.size(); i++)
      {
         Tuple2DBasics point = contactPoints.get(i);
         ret.add(new FramePoint3D(soleFrame, point.getX(), point.getY(), 0.0));
      }

      return ret;
   }

   @Override
   public MovingReferenceFrame getFrameAfterParentJoint()
   {
      return rigidBody.getParentJoint().getFrameAfterJoint();
   }

   public FrameConvexPolygon2D getContactPolygonCopy()
   {
      return new FrameConvexPolygon2D(soleFrame, Vertex2DSupplier.asVertex2DSupplier(contactPoints));
   }

   @Override
   public ReferenceFrame getSoleFrame()
   {
      return soleFrame;
   }

   @Override
   public List<FramePoint2D> getContactPoints2d()
   {
      return frameContactPoints;
   }

   @Override
   public int getTotalNumberOfContactPoints()
   {
      return totalNumberOfContactPoints;
   }

   @Override
   public void setSoleFrameTransformFromParentJoint(RigidBodyTransform transform)
   {
      throw new UnsupportedOperationException();
   }

}