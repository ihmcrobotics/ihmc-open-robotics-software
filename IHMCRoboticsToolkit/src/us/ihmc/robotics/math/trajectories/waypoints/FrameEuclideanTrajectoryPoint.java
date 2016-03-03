package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameEuclideanTrajectoryPoint extends FrameTrajectoryPoint<FrameEuclideanTrajectoryPoint, SimpleEuclideanTrajectoryPoint>
      implements EuclideanTrajectoryPointInterface<FrameEuclideanTrajectoryPoint>
{
   private final SimpleEuclideanTrajectoryPoint geometryObject;
   
   public FrameEuclideanTrajectoryPoint()
   {
      super(new SimpleEuclideanTrajectoryPoint());
      geometryObject = getGeometryObject();
   }

   public FrameEuclideanTrajectoryPoint(ReferenceFrame referenceFrame)
   {
      this();
      setToZero(referenceFrame);
   }

   public FrameEuclideanTrajectoryPoint(double time, FramePoint position, FrameVector linearVelocity)
   {
      this();
      setIncludingFrame(time, position, linearVelocity);
   }

   public FrameEuclideanTrajectoryPoint(ReferenceFrame referenceFrame, EuclideanTrajectoryPointInterface<?> euclideanTrajectoryPointInterface)
   {
      this();
      setIncludingFrame(referenceFrame, euclideanTrajectoryPointInterface);
   }

   public FrameEuclideanTrajectoryPoint(FrameEuclideanTrajectoryPoint other)
   {
      this();
      setIncludingFrame(other);
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

   public void set(double time, Point3d position, Vector3d linearVelocity)
   {
      geometryObject.set(time, position, linearVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, Point3d position, Vector3d linearVelocity)
   {
      setToZero(referenceFrame);
      geometryObject.set(time, position, linearVelocity);
   }

   public void set(double time, FramePoint position, FrameVector linearVelocity)
   {
      checkReferenceFrameMatch(position);
      checkReferenceFrameMatch(linearVelocity);
      geometryObject.set(time, position.getPoint(), linearVelocity.getVector());
   }

   public void setIncludingFrame(double time, FramePoint position, FrameVector linearVelocity)
   {
      position.checkReferenceFrameMatch(linearVelocity);
      setToZero(position.getReferenceFrame());
      geometryObject.set(time, position.getPoint(), linearVelocity.getVector());
   }

   public void set(double time, EuclideanWaypointInterface<?> euclideanWaypoint)
   {
      geometryObject.set(time, euclideanWaypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, EuclideanWaypointInterface<?> euclideanWaypoint)
   {
      setToZero(referenceFrame);
      geometryObject.set(time, euclideanWaypoint);
   }

   public void set(EuclideanTrajectoryPointInterface<?> euclideanTrajectoryPoint)
   {
      geometryObject.set(euclideanTrajectoryPoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, EuclideanTrajectoryPointInterface<?> euclideanTrajectoryPoint)
   {
      setToZero(referenceFrame);
      geometryObject.set(euclideanTrajectoryPoint);
   }

   public void set(double time, FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      checkReferenceFrameMatch(frameEuclideanWaypoint);
      setTime(time);
      frameEuclideanWaypoint.get(geometryObject);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      setToZero(referenceFrame);
      setTime(time);
      frameEuclideanWaypoint.get(geometryObject);
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

   public double positionDistance(FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint)
   {
      checkReferenceFrameMatch(frameEuclideanTrajectoryPoint);
      return geometryObject.positionDistance(frameEuclideanTrajectoryPoint.geometryObject);
   }

   @Override
   public void getPosition(Point3d positionToPack)
   {
      geometryObject.getPosition(positionToPack);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      geometryObject.getLinearVelocity(linearVelocityToPack);
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

   public double get(YoFramePoint positionToPack, YoFrameVector linearVelocityToPack)
   {
      checkReferenceFrameMatch(positionToPack);
      checkReferenceFrameMatch(linearVelocityToPack);
      positionToPack.set(geometryObject.getPosition());
      linearVelocityToPack.set(geometryObject.getLinearVelocity());
      return getTime();
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeToString = "time = " + doubleFormat.format(getTime());
      return "Euclidean trajectory point: (" + timeToString + ", " + geometryObject + ")";
   }
}
