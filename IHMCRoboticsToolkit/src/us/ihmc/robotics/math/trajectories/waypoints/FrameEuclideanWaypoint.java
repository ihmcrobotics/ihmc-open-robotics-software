package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.AbstractFrameObject;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameEuclideanWaypoint extends AbstractFrameObject<FrameEuclideanWaypoint, SimpleEuclideanWaypoint>
      implements EuclideanWaypointInterface<FrameEuclideanWaypoint>
{
   private final SimpleEuclideanWaypoint geometryObject;
   
   public FrameEuclideanWaypoint()
   {
      super(new SimpleEuclideanWaypoint());
      geometryObject = getGeometryObject();
   }

   @Override
   public void setPosition(Point3d position)
   {
      geometryObject.setPosition(position);
   }

   public void setPosition(FramePoint position)
   {
      checkReferenceFrameMatch(position);
      geometryObject.setPosition(position.getPoint());
   }

   @Override
   public void setLinearVelocity(Vector3d linearVelocity)
   {
      geometryObject.setLinearVelocity(linearVelocity);
   }

   public void setLinearVelocity(FrameVector linearVelocity)
   {
      checkReferenceFrameMatch(linearVelocity);
      geometryObject.setLinearVelocity(linearVelocity.getVector());
   }

   public void set(Point3d position, Vector3d linearVelocity)
   {
      geometryObject.set(position, linearVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Point3d position, Vector3d linearVelocity)
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
   public void getPosition(Point3d positionToPack)
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
   public void getLinearVelocity(Vector3d linearVelocityToPack)
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

   public void get(Point3d positionToPack, Vector3d linearVelocityToPack)
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
