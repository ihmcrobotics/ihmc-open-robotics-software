package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.frameObjects.FrameEuclideanWaypoint;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
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

   public void set(double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      geometryObject.set(time, position, linearVelocity);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double time, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
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

   public void setIncludingFrame(double time, FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      setToZero(frameEuclideanWaypoint.getReferenceFrame());
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

   public void getEuclideanWaypoint(EuclideanWaypoint euclideanWaypointToPack)
   {
      geometryObject.get(euclideanWaypointToPack);
   }
   
   public void getFrameEuclideanWaypoint(FrameEuclideanWaypoint frameEuclideanWaypoint)
   {
      checkReferenceFrameMatch(frameEuclideanWaypoint);

      Point3DReadOnly position = geometryObject.getPosition();
      Vector3DReadOnly linearVelocity = geometryObject.getLinearVelocity();
 
      frameEuclideanWaypoint.set(position, linearVelocity);
   }

   public double positionDistance(FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint)
   {
      checkReferenceFrameMatch(frameEuclideanTrajectoryPoint);
      return geometryObject.positionDistance(frameEuclideanTrajectoryPoint.geometryObject);
   }

   @Override
   public void getPosition(Point3DBasics positionToPack)
   {
      geometryObject.getPosition(positionToPack);
   }

   @Override
   public void getLinearVelocity(Vector3DBasics linearVelocityToPack)
   {
      geometryObject.getLinearVelocity(linearVelocityToPack);
   }

   public void getPosition(FramePoint positionToPack)
   {
      checkReferenceFrameMatch(positionToPack);
      geometryObject.getPosition(positionToPack.getPoint());
   }

   public FramePoint getPositionCopy()
   {
      FramePoint positionCopy = new FramePoint(getReferenceFrame());
      getPosition(positionCopy);
      return positionCopy;
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

   public FrameVector getLinearVelocityCopy()
   {
      FrameVector linearVelocityCopy = new FrameVector(getReferenceFrame());
      getLinearVelocity(linearVelocityCopy);
      return linearVelocityCopy;
   }

   public void getLinearVelocityIncludingFrame(FrameVector linearVelocityToPack)
   {
      linearVelocityToPack.setToZero(getReferenceFrame());
      geometryObject.getLinearVelocity(linearVelocityToPack.getVector());
   }

   public double get(Point3D positionToPack, Vector3D linearVelocityToPack)
   {
      getPosition(positionToPack);
      getLinearVelocity(linearVelocityToPack);
      return getTime();
   }

   public double get(FramePoint positionToPack, FrameVector linearVelocityToPack)
   {
      getPosition(positionToPack);
      getLinearVelocity(linearVelocityToPack);
      return getTime();
   }

   public double getIncludingFrame(FramePoint positionToPack, FrameVector linearVelocityToPack)
   {
      getPositionIncludingFrame(positionToPack);
      getLinearVelocityIncludingFrame(linearVelocityToPack);
      return getTime();
   }

   public double getPositionX()
   {
      return geometryObject.getPositionX();
   }

   public double getPositionY()
   {
      return geometryObject.getPositionY();
   }

   public double getPositionZ()
   {
      return geometryObject.getPositionZ();
   }

   public double getLinearVelocityX()
   {
      return geometryObject.getLinearVelocityX();
   }

   public double getLinearVelocityY()
   {
      return geometryObject.getLinearVelocityY();
   }

   public double getLinearVelocityZ()
   {
      return geometryObject.getLinearVelocityZ();
   }

   @Override
   public String toString()
   {
      NumberFormat doubleFormat = new DecimalFormat(" 0.00;-0.00");
      String timeToString = "time = " + doubleFormat.format(getTime());
      return "Euclidean trajectory point: (" + timeToString + ", " + geometryObject + ")";
   }
}
