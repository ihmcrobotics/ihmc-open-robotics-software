package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.geometry.yoFrameObjects.YoFrameEuclideanWaypoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameEuclideanTrajectoryPoint
      extends YoFrameTrajectoryPoint<YoFrameEuclideanTrajectoryPoint, FrameEuclideanTrajectoryPoint, SimpleEuclideanTrajectoryPoint>
      implements EuclideanTrajectoryPointInterface<YoFrameEuclideanTrajectoryPoint>
{
   private final YoFramePoint position;
   private final YoFrameVector linearVelocity;

   public YoFrameEuclideanTrajectoryPoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame... referenceFrames)
   {
      super(new FrameEuclideanTrajectoryPoint(), namePrefix, nameSuffix, registry, referenceFrames);
      position = YoFrameEuclideanWaypoint.createYoPosition(this, namePrefix, nameSuffix, registry);
      linearVelocity = YoFrameEuclideanWaypoint.createYoLinearVelocity(this, namePrefix, nameSuffix, registry);
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      this.position.set(position);
   }

   public void setPosition(FramePoint position)
   {
      this.position.set(position);
   }

   @Override
   public void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   public void setLinearVelocity(FrameVector linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   public void set(EuclideanTrajectoryPointInterface<?> euclideanTrajectoryPoint)
   {
      frameWaypoint.setToZero(getReferenceFrame());
      frameWaypoint.set(euclideanTrajectoryPoint);
      getYoValuesFromFrameWaypoint();
   }

   public void set(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      this.time.set(time);
      this.position.set(position);
      this.linearVelocity.set(linearVelocity);
   }

   public void set(double time, FramePoint position, FrameVector linearVelocity)
   {
      this.time.set(time);
      this.position.set(position);
      this.linearVelocity.set(linearVelocity);
   }

   public void set(double time, YoFramePoint position, YoFrameVector linearVelocity)
   {
      this.time.set(time);
      this.position.set(position);
      this.linearVelocity.set(linearVelocity);
   }

   @Override
   public void setPositionToZero()
   {
      position.setToZero();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      linearVelocity.setToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      position.setToNaN();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      linearVelocity.setToNaN();
   }

   @Override
   public void setToNaN()
   {
      super.setToNaN();
      setTimeToNaN();
      setPositionToNaN();
      setLinearVelocityToNaN();
   }

   @Override
   public void setToNaN(ReferenceFrame referenceFrame)
   {
      super.setToNaN(referenceFrame);
      setToNaN();
   }

   @Override
   public double positionDistance(YoFrameEuclideanTrajectoryPoint other)
   {
      putYoValuesIntoFrameWaypoint();
      other.putYoValuesIntoFrameWaypoint();
      return frameWaypoint.positionDistance(other.frameWaypoint);
   }

   @Override
   public void getPosition(Point3DBasics positionToPack)
   {
      position.get(positionToPack);
   }

   @Override
   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      linearVelocity.get(linearVelocityToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      position.getFrameTuple(positionToPack);
   }

   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      linearVelocity.getFrameTuple(linearVelocityToPack);
   }

   public void getPositionIncludingFrame(FramePoint positionToPack)
   {
      position.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getLinearVelocityIncludingFrame(FrameVector linearVelocityToPack)
   {
      linearVelocity.getFrameTupleIncludingFrame(linearVelocityToPack);
   }

   public void getPosition(YoFramePoint positionToPack)
   {
      positionToPack.set(position);
   }

   public void getLinearVelocity(YoFrameVector linearVelocityToPack)
   {
      linearVelocityToPack.set(linearVelocity);
   }

   /**
    * Return the original position held by this trajectory point.
    */
   public YoFramePoint getPosition()
   {
      return position;
   }

   /**
    * Return the original linearVelocity held by this trajectory point.
    */
   public YoFrameVector getLinearVelocity()
   {
      return linearVelocity;
   }

   @Override
   protected void getYoValuesFromFrameWaypoint()
   {
      SimpleEuclideanTrajectoryPoint simpleTrajectoryPoint = frameWaypoint.getGeometryObject();
      EuclideanWaypoint euclideanWaypoint = simpleTrajectoryPoint.getEuclideanWaypoint();
      
      time.set(simpleTrajectoryPoint.getTime());
      position.set(euclideanWaypoint.getPosition());
      linearVelocity.set(euclideanWaypoint.getLinearVelocity());
   }

   @Override
   protected void putYoValuesIntoFrameWaypoint()
   {
      frameWaypoint.setToZero(getReferenceFrame());

      frameWaypoint.setTime(time.getDoubleValue());
      frameWaypoint.setPosition(position.getFrameTuple());
      frameWaypoint.setLinearVelocity(linearVelocity.getFrameTuple());
   }

   @Override
   public String toString()
   {
      putYoValuesIntoFrameWaypoint();
      return frameWaypoint.toString();
   }
}
