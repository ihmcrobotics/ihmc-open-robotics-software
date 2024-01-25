package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class GroundPlaneEstimator
{
   private final static int MAX_GROUND_PLANE_POINTS = 100;
   private final Plane3D groundPlane = new Plane3D();
   private final Point3D groundPlanePoint = new Point3D();
   private final ArrayList<Point3DReadOnly> groundPlanePoints = new ArrayList<>(MAX_GROUND_PLANE_POINTS);
   private final LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   private final FramePose3D groundPlanePose = new FramePose3D();
   private final PoseReferenceFrame groundPlaneFrame = new PoseReferenceFrame("groundPlaneFrame", ReferenceFrame.getWorldFrame());


   /**
    * @return pitch angle of ground plane in World Frame
    */
   public double getPitch()
   {
      // reproducing for speed
      return Math.atan2(groundPlane.getNormal().getX(), groundPlane.getNormal().getZ());
//      return getPitch(0.0);
   }

   /**
    * @return pitch angle of ground plane in World Frame
    */
   public double getRoll()
   {
      // reproducing for speed
      return Math.atan2(-groundPlane.getNormal().getY(), groundPlane.getNormal().getZ());
//      return getRoll(0.0);
   }

   /**
    * @param yaw : angle Of ZUp frame relative to World Frame
    * @return pitch angle of ground plane in ZUp Frame
    */
   public double getPitch(double yaw)
   {
      return Math.atan2(Math.cos(yaw) * groundPlane.getNormal().getX() + Math.sin(yaw) * groundPlane.getNormal().getY(), groundPlane.getNormal().getZ());
   }

   /**
    * @param yaw : angle Of ZUp frame relative to World Frame
    * @return roll angle of ground plane in ZUp Frame
    */
   public double getRoll(double yaw)
   {
      return Math.atan2(Math.sin(yaw) * groundPlane.getNormal().getX() - Math.cos(yaw) * groundPlane.getNormal().getY(), groundPlane.getNormal().getZ());
   }

   /**
    * @param plane3d : ground plane in World Frame
    */
   public void getPlane(Plane3D plane3d)
   {
      plane3d.set(groundPlane);
   }

   /**
    * @param point : ground plane point in World Frame
    */
   public void getPlanePoint(FramePoint3D point)
   {
      point.changeFrame(ReferenceFrame.getWorldFrame());
      point.set(groundPlane.getPoint());
   }

   /**
    * @param normal : ground plane normal in World Frame
    */
   public void getPlaneNormal(FrameVector3D normal)
   {
      normal.changeFrame(ReferenceFrame.getWorldFrame());
      normal.set(groundPlane.getNormal());
   }

   /**
    * @param point : point to be vertically projected onto ground plane
    */
   public void projectZ(FramePoint3DBasics point)
   {
      point.changeFrame(ReferenceFrame.getWorldFrame());
      point.setZ(groundPlane.getZOnPlane(point.getX(), point.getY()));
   }

   /**
    * @param point : point to be orthogonally projected onto ground plane
    */
   public void projectOrthogonal(FramePoint3DBasics point)
   {
      point.changeFrame(ReferenceFrame.getWorldFrame());
      groundPlane.orthogonalProjection(point);
   }

   /**
    * Clear the list of ground contact points.
    */
   public void clearContactPoints()
   {
      groundPlanePoints.clear();
   }

   /**
    * Add a point to the list of ground contact points.
    * @param contactPoint : ground contact point in world frame
    */
   public void addContactPoint(Point3DReadOnly contactPoint)
   {
      groundPlanePoints.add(contactPoint);
   }

   /**
    * Estimate the ground plane given the current list of ground contact points.
    */
   public void compute()
   {
      planeFitter.fitPlaneToPoints(groundPlanePoints, groundPlane);
      groundPlanePoint.set(groundPlane.getPoint());

      groundPlanePose.getPosition().set(groundPlane.getPoint());
      groundPlanePose.getOrientation().setYawPitchRoll(0.0, getPitch(), getRoll());
      groundPlaneFrame.setPoseAndUpdate(groundPlanePose);
   }

   /**
    * Set the list of ground contact points and compute the ground plane.
    * @param contactPoints : list of ground contact points
    */
   public void compute(List<FramePoint3D> contactPoints)
   {
      int nPoints = Math.min(contactPoints.size(), MAX_GROUND_PLANE_POINTS);
      groundPlanePoints.clear();
      for (int i = 0; i < nPoints; i++)
      {
         contactPoints.get(i).changeFrame(ReferenceFrame.getWorldFrame());
         groundPlanePoints.add(contactPoints.get(i));
      }
      compute();
   }

   /**
    * Set the list of ground contact points and compute the ground plane.
    * @param contactPoints : quadrant dependent list of contact points
    */
   public void compute(QuadrantDependentList<FramePoint3D> contactPoints)
   {
      groundPlanePoints.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactPoints.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         groundPlanePoints.add(contactPoints.get(robotQuadrant));
      }
      compute();
   }

   public PoseReferenceFrame getGroundPlaneFrame()
   {
      return groundPlaneFrame;
   }
}
