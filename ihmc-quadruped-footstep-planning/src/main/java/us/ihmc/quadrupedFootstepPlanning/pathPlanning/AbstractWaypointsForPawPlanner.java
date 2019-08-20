package us.ihmc.quadrupedFootstepPlanning.pathPlanning;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawPlanningResult;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawPlannerGoal;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;

public abstract class AbstractWaypointsForPawPlanner implements WaypointsForPawPlanner
{
   protected final static boolean debug = false;
   private static final double defaultFallbackRegionSize = 0.3;



   protected final FramePose3D bodyStartPose = new FramePose3D();
   protected final FramePose3D bodyGoalPose = new FramePose3D();

   protected final List<Point3D> waypoints = new ArrayList<>();

   protected final YoEnum<PawPlanningResult> yoResult;

   protected PlanarRegionsList planarRegionsList;

   private double fallbackRegionSize = defaultFallbackRegionSize;


   public AbstractWaypointsForPawPlanner(String prefix, YoVariableRegistry registry)
   {
      yoResult = new YoEnum<>(prefix + "PathPlanningResult", registry, PawPlanningResult.class);
   }

   @Override
   public void setInitialBodyPose(FramePose3DReadOnly initialPose)
   {
      bodyStartPose.setToZero(ReferenceFrame.getWorldFrame());
      bodyStartPose.setPosition(initialPose.getX(), initialPose.getY(), 0.0);
      bodyStartPose.setOrientationYawPitchRoll(initialPose.getYaw(), 0.0, 0.0);
   }

   @Override
   public void setGoal(PawPlannerGoal goal)
   {
      FramePose3DReadOnly goalPose = goal.getTargetPose();
      bodyGoalPose.setIncludingFrame(goalPose);
   }

   @Override
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = new PlanarRegionsList(planarRegionsList);
   }

   @Override
   public void setFallbackRegionSize(double size)
   {
      this.fallbackRegionSize = size;
   }

   public void computeBestEffortPlan(double horizonLength)
   {
      Vector2D goalDirection = new Vector2D(bodyGoalPose.getPosition());
      goalDirection.sub(bodyStartPose.getX(), bodyStartPose.getY());
      goalDirection.scale(horizonLength / goalDirection.length());
      Point3D waypoint = new Point3D(bodyStartPose.getPosition());
      waypoint.add(goalDirection.getX(), goalDirection.getY(), 0.0);
      waypoints.add(waypoint);
   }

   @Override
   public List<Point3D> getWaypoints()
   {
      return waypoints;
   }

   protected void addPlanarRegionAtHeight(Point3DReadOnly point, int id)
   {
      addPlanarRegionAtHeight(point.getX(), point.getY(), point.getZ(), id);
   }

   protected void addPlanarRegionAtHeight(double xLocation, double yLocation, double zLocation, int id)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(fallbackRegionSize, fallbackRegionSize);
      polygon.addVertex(-fallbackRegionSize, fallbackRegionSize);
      polygon.addVertex(fallbackRegionSize, -fallbackRegionSize);
      polygon.addVertex(-fallbackRegionSize, -fallbackRegionSize);
      polygon.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(new AxisAngle(), new Vector3D(xLocation, yLocation, zLocation)), polygon);
      planarRegion.setRegionId(id);
      planarRegionsList.addPlanarRegion(planarRegion);
   }

   @Override
   public FramePose3DReadOnly getInitialBodyPose()
   {
      return bodyStartPose;
   }

   @Override
   public FramePose3DReadOnly getGoalBodyPose()
   {
      return bodyGoalPose;
   }

}
