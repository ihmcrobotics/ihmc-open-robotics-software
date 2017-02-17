package us.ihmc.robotics.geometry.frameObjects;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.AbstractFrameObject;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.WaypointToStringTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameEuclideanWaypoint extends AbstractFrameObject<FrameEuclideanWaypoint, EuclideanWaypoint>
      implements EuclideanWaypointInterface<FrameEuclideanWaypoint>
{
   private final EuclideanWaypoint geometryObject;
   
   public FrameEuclideanWaypoint()
   {
      super(new EuclideanWaypoint());
      geometryObject = getGeometryObject();
   }

   public FrameEuclideanWaypoint(ReferenceFrame referenceFrame)
   {
      this();
      this.referenceFrame = referenceFrame;
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      geometryObject.setPosition(position);
   }

   public void setPosition(FramePoint position)
   {
      checkReferenceFrameMatch(position);
      geometryObject.setPosition(position.getPoint());
   }

   @Override
   public void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      geometryObject.setLinearVelocity(linearVelocity);
   }

   public void setLinearVelocity(FrameVector linearVelocity)
   {
      checkReferenceFrameMatch(linearVelocity);
      geometryObject.setLinearVelocity(linearVelocity.getVector());
   }

   public void set(Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      geometryObject.set(position, linearVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      setToZero(referenceFrame);
      geometryObject.set(position, linearVelocity);
   }

   public void set(FramePoint position, FrameVector linearVelocity)
   {
      checkReferenceFrameMatch(position);
      checkReferenceFrameMatch(linearVelocity);
      geometryObject.set(position.getPoint(), linearVelocity.getVector());
   }

   public void setIncludingFrame(FramePoint position, FrameVector linearVelocity)
   {
      position.checkReferenceFrameMatch(linearVelocity);
      setToZero(position.getReferenceFrame());
      geometryObject.set(position.getPoint(), linearVelocity.getVector());
   }

   public void set(EuclideanWaypointInterface<?> euclideanWaypoint)
   {
      geometryObject.set(euclideanWaypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, EuclideanWaypointInterface<?> euclideanWaypoint)
   {
      setToZero(referenceFrame);
      geometryObject.set(euclideanWaypoint);
   }

   @Override
   public void setPositionToZero()
   {
      geometryObject.setPositionToZero();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      geometryObject.setLinearVelocityToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      geometryObject.setPositionToNaN();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      geometryObject.setLinearVelocityToNaN();
   }

   public double positionDistance(FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      checkReferenceFrameMatch(frameEuclideanWaypoint);
      return geometryObject.positionDistance(frameEuclideanWaypoint.geometryObject);
   }

   @Override
   public void getPosition(Point3DBasics positionToPack)
   {
      geometryObject.getPosition(positionToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      checkReferenceFrameMatch(positionToPack);
      geometryObject.getPosition(positionToPack.getPoint());
   }

   public void getPositionIncludingFrame(FramePoint positionToPack)
   {
      positionToPack.setToZero(getReferenceFrame());
      geometryObject.getPosition(positionToPack.getPoint());
   }

   @Override
   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      geometryObject.getLinearVelocity(linearVelocityToPack);
   }

   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      checkReferenceFrameMatch(linearVelocityToPack);
      geometryObject.getLinearVelocity(linearVelocityToPack.getVector());
   }

   public void getLinearVelocityIncludingFrame(FrameVector linearVelocityToPack)
   {
      linearVelocityToPack.setToZero(getReferenceFrame());
      geometryObject.getLinearVelocity(linearVelocityToPack.getVector());
   }

   public void get(Point3DBasics positionToPack, Vector3DBasics linearVelocityToPack)
   {
      geometryObject.get(positionToPack, linearVelocityToPack);
   }

   public void get(FramePoint positionToPack, FrameVector linearVelocityToPack)
   {
      getPosition(positionToPack);
      getLinearVelocity(linearVelocityToPack);
   }

   public void getIncludingFrame(FramePoint positionToPack, FrameVector linearVelocityToPack)
   {
      getPositionIncludingFrame(positionToPack);
      getLinearVelocityIncludingFrame(linearVelocityToPack);
   }

   public void get(EuclideanWaypointInterface<?> euclideanWaypoint)
   {
      euclideanWaypoint.setPosition(geometryObject.getPosition());
      euclideanWaypoint.setLinearVelocity(geometryObject.getLinearVelocity());
   }

   @Override
   public String toString()
   {
      return WaypointToStringTools.waypointToString(this);
   }
}
