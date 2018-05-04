package us.ihmc.quadrupedRobotics.estimator;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class GroundPlaneEstimator
{
   public final static int MAX_GROUND_PLANE_POINTS = 100;
   private final Plane3D groundPlane = new Plane3D();
   private final Vector3D groundPlaneNormal = new Vector3D();
   private final Point3D groundPlanePoint = new Point3D();
   private final ArrayList<Point3DReadOnly> groundPlanePoints = new ArrayList<>(MAX_GROUND_PLANE_POINTS);
   private final LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFramePoint3D yoGroundPlanePoint = new YoFramePoint3D("groundPlanePoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D yoGroundPlaneNormal = new YoFrameVector3D("groundPlaneNormal", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameYawPitchRoll yoGroundPlaneOrientation = new YoFrameYawPitchRoll("groundPlaneOrientation", ReferenceFrame.getWorldFrame(), registry);

   public GroundPlaneEstimator()
   {
      this(null, null);
   }

   public GroundPlaneEstimator(YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this(parentRegistry, graphicsListRegistry, YoAppearance.Glass());
   }

   public GroundPlaneEstimator(YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry, AppearanceDefinition groundPlaneAppearance)
   {

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

      if (graphicsListRegistry != null)
      {
         Graphics3DObject groundPlaneGraphic = new Graphics3DObject();
         groundPlaneGraphic.addCylinder(0.005, 0.5, groundPlaneAppearance);
         YoGraphicShape yoGroundPlaneGraphic = new YoGraphicShape("groundPlaneEstimate", groundPlaneGraphic, yoGroundPlanePoint, yoGroundPlaneOrientation, 1.0);
         graphicsListRegistry.registerYoGraphic("groundPlaneEstimate", yoGroundPlaneGraphic);
      }
   }

   /**
    * @return pitch angle of ground plane in World Frame
    */
   public double getPitch()
   {
      return getPitch(0);
   }

   /**
    * @return pitch angle of ground plane in World Frame
    */
   public double getRoll()
   {
      return getRoll(0);
   }

   /**
    * @param yaw : angle Of ZUp frame relative to World Frame
    * @return pitch angle of ground plane in ZUp Frame
    */
   public double getPitch(double yaw)
   {
      groundPlane.getNormal(groundPlaneNormal);
      return Math.atan2(Math.cos(yaw) * groundPlaneNormal.getX() - Math.sin(yaw) * groundPlaneNormal.getY(), groundPlaneNormal.getZ());
   }

   /**
    * @param yaw : angle Of ZUp frame relative to World Frame
    * @return roll angle of ground plane in ZUp Frame
    */
   public double getRoll(double yaw)
   {
      groundPlane.getNormal(groundPlaneNormal);
      return Math.atan2(Math.sin(yaw) * groundPlaneNormal.getX() + Math.cos(yaw) * groundPlaneNormal.getY(), groundPlaneNormal.getZ());
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
      groundPlane.getPoint(point);
   }

   /**
    * @param normal : ground plane normal in World Frame
    */
   public void getPlaneNormal(FrameVector3D normal)
   {
      normal.changeFrame(ReferenceFrame.getWorldFrame());
      groundPlane.getNormal(normal);
   }

   /**
    * @param point : point to be vertically projected onto ground plane
    */
   public void projectZ(FramePoint3D point)
   {
      point.changeFrame(ReferenceFrame.getWorldFrame());
      point.setZ(groundPlane.getZOnPlane(point.getX(), point.getY()));
   }

   /**
    * @param point : point in world frame to be vertically projected onto ground plane
    */
   public void projectZ(Point3D point)
   {
      point.setZ(groundPlane.getZOnPlane(point.getX(), point.getY()));
   }

   /**
    * @param point : point to be orthogonally projected onto ground plane
    */
   public void projectOrthogonal(FramePoint3D point)
   {
      point.changeFrame(ReferenceFrame.getWorldFrame());
      groundPlane.orthogonalProjection(point);
   }

   /**
    * @param point : point in world frame to be orthogonally projected onto ground plane
    */
   public void projectOrthogonal(Point3D point)
   {
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
      groundPlane.getNormal(groundPlaneNormal);
      yoGroundPlaneNormal.set(groundPlaneNormal);
      groundPlane.getPoint(groundPlanePoint);
      yoGroundPlanePoint.set(groundPlanePoint);
      yoGroundPlaneOrientation.setYawPitchRoll(0.0, getPitch(), getRoll());
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
}
