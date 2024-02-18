package us.ihmc.robotics.geometry;

import java.util.List;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class GroundPlaneEstimator
{
   private final static int MAX_GROUND_PLANE_POINTS = 100;
   private final Plane3D groundPlane = new Plane3D();
   private final RecyclingArrayList<FramePoint3D> groundPlanePoints = new RecyclingArrayList<>(FramePoint3D.class);
   private final LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();

   private final FramePose3D groundPlanePose = new FramePose3D();
   private final PoseReferenceFrame groundPlaneFrame = new PoseReferenceFrame("groundPlaneFrame", ReferenceFrame.getWorldFrame());


   /**
    * @return pitch angle of ground plane in World Frame
    */
   public double getPitch()
   {
      return Math.atan2(groundPlane.getNormal().getX(), groundPlane.getNormal().getZ());
   }

   /**
    * @return roll angle of ground plane in World Frame
    */
   public double getRoll()
   {
      return Math.atan2(-groundPlane.getNormal().getY(), groundPlane.getNormal().getZ());
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
    * @param plane3dToPack : ground plane to pack in World Frame
    */
   public void getPlane(Plane3D plane3dToPack)
   {
      plane3dToPack.set(groundPlane);
   }

   /**
    * returns ground plane
    */
   public Plane3DReadOnly getPlane()
   {
      return groundPlane;
   }

   /**
    * @param pointToPack : ground plane point to pack
    */
   public void getPlanePoint(FixedFramePoint3DBasics pointToPack)
   {
      pointToPack.setMatchingFrame(ReferenceFrame.getWorldFrame(), groundPlane.getPoint());
   }

   /**
    * returns ground plane point in World Frame
    */
   public Point3DReadOnly getPlanePoint()
   {
      return groundPlane.getPoint();
   }

   /**
    * @param normalToPack : ground plane normal to pack
    */
   public void getPlaneNormal(FixedFrameVector3DBasics normalToPack)
   {
      normalToPack.setMatchingFrame(ReferenceFrame.getWorldFrame(), groundPlane.getNormal());
   }

   /**
    * returns ground plane normal in World Frame
    */
   public Vector3DReadOnly getPlaneNormal()
   {
      return groundPlane.getNormal();
   }

   /**
    * @param pointToPack : point to be vertically projected onto ground plane
    */
   public void projectZ(FramePoint3DBasics pointToPack)
   {
      projectZ(pointToPack, groundPlane);
   }

   public static void projectZ(FramePoint3DBasics pointToPack, Plane3DReadOnly groundPlane)
   {
      ReferenceFrame originalFrame = pointToPack.getReferenceFrame();
      pointToPack.changeFrame(ReferenceFrame.getWorldFrame());
      pointToPack.setZ(groundPlane.getZOnPlane(pointToPack.getX(), pointToPack.getY()));
      pointToPack.changeFrame(originalFrame);
   }

   /**
    * @param pointToPack : point to be orthogonally projected onto ground plane
    */
   public void projectOrthogonal(FramePoint3DBasics pointToPack)
   {
      ReferenceFrame originalFrame = pointToPack.getReferenceFrame();
      pointToPack.changeFrame(ReferenceFrame.getWorldFrame());
      groundPlane.orthogonalProjection(pointToPack);
      pointToPack.changeFrame(originalFrame);
   }

   /**
    * Clear the list of ground contact points.
    */
   public void clearContactPoints()
   {
      groundPlanePoints.clear();
   }

   /**
    * Add a point to the list of points to fit ground plane to.
    * @param contactPoint : ground point
    */
   public void addContactPoint(FramePoint3DReadOnly contactPoint)
   {
      groundPlanePoints.add().setMatchingFrame(contactPoint);
   }

   /**
    * Estimate the ground plane given the current list of ground contact points.
    */
   public void compute()
   {
      planeFitter.fitPlaneToPoints(groundPlanePoints, groundPlane);

      groundPlanePose.getPosition().set(groundPlane.getPoint());
      groundPlanePose.getOrientation().setYawPitchRoll(0.0, getPitch(), getRoll());
      groundPlaneFrame.setPoseAndUpdate(groundPlanePose);
   }

   /**
    * Set the list of ground contact points and compute the ground plane.
    * @param contactPoints : list of ground contact points
    */
   public void compute(List<? extends FramePoint3DReadOnly> contactPoints)
   {
      groundPlanePoints.clear();

      int nPoints = Math.min(contactPoints.size(), MAX_GROUND_PLANE_POINTS);
      for (int i = 0; i < nPoints; i++)
         addContactPoint(contactPoints.get(i));

      compute();
   }

   /**
    * Set the list of ground contact points and compute the ground plane.
    * @param contactPoints : quadrant dependent list of contact points
    */
   public void compute(QuadrantDependentList<? extends FramePoint3DReadOnly> contactPoints)
   {
      groundPlanePoints.clear();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         addContactPoint(contactPoints.get(robotQuadrant));

      compute();
   }

   public PoseReferenceFrame getGroundPlaneFrame()
   {
      return groundPlaneFrame;
   }
}
