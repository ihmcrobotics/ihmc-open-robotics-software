package us.ihmc.robotics.math.trajectories.waypoints;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameEuclideanTrajectoryPoint
      extends YoFrameTrajectoryPoint<SimpleEuclideanTrajectoryPoint, FrameEuclideanTrajectoryPoint, YoFrameEuclideanTrajectoryPoint>
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
   public void setPosition(Point3d position)
   {
      this.position.set(position);
   }

   @Override
   public void setLinearVelocity(Vector3d linearVelocity)
   {
      this.linearVelocity.set(linearVelocity);
   }

   public void set(EuclideanTrajectoryPointInterface<?> euclideanTrajectoryPoint)
   {
      frameWaypoint.setToZero(getReferenceFrame());
      frameWaypoint.set(euclideanTrajectoryPoint);
      getYoValuesFromFrameTrajectoryPoint();
   }

   public void set(double time, Point3d position, Vector3d linearVelocity)
   {
      setTime(time);
      this.position.set(position);
      this.linearVelocity.set(linearVelocity);
   }

   public void set(double time, FramePoint position, FrameVector linearVelocity)
   {
      setTime(time);
      this.position.set(position);
      this.linearVelocity.set(linearVelocity);
   }

   public void set(DoubleYoVariable time, YoFramePoint position, YoFrameVector linearVelocity)
   {
      setTime(time.getDoubleValue());
      this.position.set(position);
      this.linearVelocity.set(linearVelocity);
   }

   @Override
   public void setPositionToZero()
   {
      frameWaypoint.setPositionToZero();
      getYoValuesFromFrameTrajectoryPoint();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      frameWaypoint.setLinearVelocityToZero();
      getYoValuesFromFrameTrajectoryPoint();
   }

   @Override
   public void setPositionToNaN()
   {
      frameWaypoint.setPositionToNaN();
      getYoValuesFromFrameTrajectoryPoint();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      frameWaypoint.setLinearVelocityToNaN();
      getYoValuesFromFrameTrajectoryPoint();
   }

   @Override
   public double positionDistance(YoFrameEuclideanTrajectoryPoint other)
   {
      return frameWaypoint.positionDistance(other.frameWaypoint);
   }

   @Override
   public void getPosition(Point3d positionToPack)
   {
      position.get(positionToPack);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
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
   protected void getYoValuesFromFrameTrajectoryPoint()
   {
      SimpleEuclideanTrajectoryPoint simpleWaypoint = frameWaypoint.getSimpleWaypoint();
      position.set(simpleWaypoint.getPosition());
      linearVelocity.set(simpleWaypoint.getLinearVelocity());
   }

   @Override
   protected void putYoValuesIntoFrameTrajectoryPoint()
   {
      SimpleEuclideanTrajectoryPoint simpleWaypoint = frameWaypoint.getSimpleWaypoint();
      position.get(simpleWaypoint.getPosition());
      linearVelocity.get(simpleWaypoint.getLinearVelocity());
   }
}
