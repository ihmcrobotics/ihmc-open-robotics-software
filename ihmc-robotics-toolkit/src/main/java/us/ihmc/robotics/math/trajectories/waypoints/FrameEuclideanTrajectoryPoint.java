package us.ihmc.robotics.math.trajectories.waypoints;

import java.text.DecimalFormat;
import java.text.NumberFormat;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.frameObjects.FrameEuclideanWaypoint;
import us.ihmc.robotics.geometry.interfaces.EuclideanWaypointInterface;
import us.ihmc.robotics.geometry.transformables.EuclideanWaypoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;

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

   public FrameEuclideanTrajectoryPoint(double time, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
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

   public void setPosition(FramePoint3DReadOnly position)
   {
      checkReferenceFrameMatch(position);
      geometryObject.setPosition(position);
   }

   @Override
   public void setLinearVelocity(Vector3DReadOnly linearVelocity)
   {
      geometryObject.setLinearVelocity(linearVelocity);
   }

   public void setLinearVelocity(FrameVector3DReadOnly linearVelocity)
   {
      checkReferenceFrameMatch(linearVelocity);
      geometryObject.setLinearVelocity(linearVelocity);
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

   public void set(double time, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      checkReferenceFrameMatch(position);
      checkReferenceFrameMatch(linearVelocity);
      geometryObject.set(time, position, linearVelocity);
   }

   public void setIncludingFrame(double time, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      position.checkReferenceFrameMatch(linearVelocity);
      setToZero(position.getReferenceFrame());
      geometryObject.set(time, position, linearVelocity);
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

   public void getPosition(FramePoint3D positionToPack)
   {
      checkReferenceFrameMatch(positionToPack);
      geometryObject.getPosition(positionToPack);
   }

   public FramePoint3D getPositionCopy()
   {
      FramePoint3D positionCopy = new FramePoint3D(getReferenceFrame());
      getPosition(positionCopy);
      return positionCopy;
   }

   public void getPositionIncludingFrame(FramePoint3D positionToPack)
   {
      positionToPack.setToZero(getReferenceFrame());
      geometryObject.getPosition(positionToPack);
   }

   public void getLinearVelocity(FrameVector3D linearVelocityToPack)
   {
      checkReferenceFrameMatch(linearVelocityToPack);
      geometryObject.getLinearVelocity(linearVelocityToPack);
   }

   public FrameVector3D getLinearVelocityCopy()
   {
      FrameVector3D linearVelocityCopy = new FrameVector3D(getReferenceFrame());
      getLinearVelocity(linearVelocityCopy);
      return linearVelocityCopy;
   }

   public void getLinearVelocityIncludingFrame(FrameVector3D linearVelocityToPack)
   {
      linearVelocityToPack.setToZero(getReferenceFrame());
      geometryObject.getLinearVelocity(linearVelocityToPack);
   }

   public double get(Point3D positionToPack, Vector3D linearVelocityToPack)
   {
      getPosition(positionToPack);
      getLinearVelocity(linearVelocityToPack);
      return getTime();
   }

   public double get(FramePoint3D positionToPack, FrameVector3D linearVelocityToPack)
   {
      getPosition(positionToPack);
      getLinearVelocity(linearVelocityToPack);
      return getTime();
   }

   public double getIncludingFrame(FramePoint3D positionToPack, FrameVector3D linearVelocityToPack)
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
      return "Euclidean trajectory point: (" + timeToString + ", " + geometryObject + ")-" + getReferenceFrame().toString();
   }
}
