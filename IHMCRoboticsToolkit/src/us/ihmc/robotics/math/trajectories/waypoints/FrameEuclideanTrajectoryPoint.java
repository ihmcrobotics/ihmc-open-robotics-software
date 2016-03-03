package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameEuclideanTrajectoryPoint extends FrameTrajectoryPoint<FrameEuclideanTrajectoryPoint, SimpleEuclideanTrajectoryPoint>
      implements EuclideanTrajectoryPointInterface<FrameEuclideanTrajectoryPoint>
{
   public FrameEuclideanTrajectoryPoint()
   {
      super(new SimpleEuclideanTrajectoryPoint());
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
      simpleWaypoint.setPosition(position);
   }

   public void setPosition(FramePoint position)
   {
      checkReferenceFrameMatch(position);
      simpleWaypoint.setPosition(position.getPoint());
   }

   @Override
   public void setLinearVelocity(Vector3d linearVelocity)
   {
      simpleWaypoint.setLinearVelocity(linearVelocity);
   }

   public void setLinearVelocity(FrameVector linearVelocity)
   {
      checkReferenceFrameMatch(linearVelocity);
      simpleWaypoint.setLinearVelocity(linearVelocity.getVector());
   }

   public void set(double time, Point3d position, Vector3d linearVelocity)
   {
      simpleWaypoint.set(time, position, linearVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, Point3d position, Vector3d linearVelocity)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(time, position, linearVelocity);
   }

   public void set(double time, FramePoint position, FrameVector linearVelocity)
   {
      checkReferenceFrameMatch(position);
      checkReferenceFrameMatch(linearVelocity);
      simpleWaypoint.set(time, position.getPoint(), linearVelocity.getVector());
   }

   public void setIncludingFrame(double time, FramePoint position, FrameVector linearVelocity)
   {
      position.checkReferenceFrameMatch(linearVelocity);
      setToZero(position.getReferenceFrame());
      simpleWaypoint.set(time, position.getPoint(), linearVelocity.getVector());
   }

   public void set(double time, EuclideanWaypointInterface<?> euclideanWaypoint)
   {
      simpleWaypoint.set(time, euclideanWaypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, EuclideanWaypointInterface<?> euclideanWaypoint)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(time, euclideanWaypoint);
   }

   public void set(EuclideanTrajectoryPointInterface<?> euclideanTrajectoryPoint)
   {
      simpleWaypoint.set(euclideanTrajectoryPoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, EuclideanTrajectoryPointInterface<?> euclideanTrajectoryPoint)
   {
      setToZero(referenceFrame);
      simpleWaypoint.set(euclideanTrajectoryPoint);
   }

   public void set(double time, FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      checkReferenceFrameMatch(frameEuclideanWaypoint);
      setTime(time);
      frameEuclideanWaypoint.get(simpleWaypoint);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      setToZero(referenceFrame);
      setTime(time);
      frameEuclideanWaypoint.get(simpleWaypoint);
   }

   @Override
   public void setPositionToZero()
   {
      simpleWaypoint.setPositionToZero();
   }

   @Override
   public void setLinearVelocityToZero()
   {
      simpleWaypoint.setLinearVelocityToZero();
   }

   @Override
   public void setPositionToNaN()
   {
      simpleWaypoint.setPositionToNaN();
   }

   @Override
   public void setLinearVelocityToNaN()
   {
      simpleWaypoint.setLinearVelocityToNaN();
   }

   public double positionDistance(FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint)
   {
      checkReferenceFrameMatch(frameEuclideanTrajectoryPoint);
      return simpleWaypoint.positionDistance(frameEuclideanTrajectoryPoint.simpleWaypoint);
   }

   @Override
   public void getPosition(Point3d positionToPack)
   {
      simpleWaypoint.getPosition(positionToPack);
   }

   @Override
   public void getLinearVelocity(Vector3d linearVelocityToPack)
   {
      simpleWaypoint.getLinearVelocity(linearVelocityToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      checkReferenceFrameMatch(positionToPack);
      simpleWaypoint.getPosition(positionToPack.getPoint());
   }

   public void getPositionIncludingFrame(FramePoint positionToPack)
   {
      positionToPack.setToZero(getReferenceFrame());
      simpleWaypoint.getPosition(positionToPack.getPoint());
   }

   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      checkReferenceFrameMatch(linearVelocityToPack);
      simpleWaypoint.getLinearVelocity(linearVelocityToPack.getVector());
   }

   public void getLinearVelocityIncludingFrame(FrameVector linearVelocityToPack)
   {
      linearVelocityToPack.setToZero(getReferenceFrame());
      simpleWaypoint.getLinearVelocity(linearVelocityToPack.getVector());
   }

   public double get(YoFramePoint positionToPack, YoFrameVector linearVelocityToPack)
   {
      checkReferenceFrameMatch(positionToPack);
      checkReferenceFrameMatch(linearVelocityToPack);
      positionToPack.set(simpleWaypoint.getPosition());
      linearVelocityToPack.set(simpleWaypoint.getLinearVelocity());
      return getTime();
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeToString = "time = " + doubleFormat.format(getTime());
      return "Euclidean trajectory point: (" + timeToString + ", " + simpleWaypoint + ")";
   }
}
