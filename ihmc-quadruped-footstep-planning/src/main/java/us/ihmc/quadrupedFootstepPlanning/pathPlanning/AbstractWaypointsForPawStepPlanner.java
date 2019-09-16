package us.ihmc.quadrupedFootstepPlanning.pathPlanning;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanningResult;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerGoal;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;

public abstract class AbstractWaypointsForPawStepPlanner implements WaypointsForPawStepPlanner
{
   protected final static boolean debug = false;
   private static final double defaultFallbackRegionSize = 0.3;



   protected final FramePose3D bodyStartPose = new FramePose3D();
   protected final FramePose3D bodyGoalPose = new FramePose3D();

   protected final List<Pose3DReadOnly> waypoints = new ArrayList<>();

   protected final YoEnum<PawStepPlanningResult> yoResult;

   protected PlanarRegionsList planarRegionsList;

   private double fallbackRegionSize = defaultFallbackRegionSize;


   public AbstractWaypointsForPawStepPlanner(String prefix, YoVariableRegistry registry)
   {
      yoResult = new YoEnum<>(prefix + "PathPlanningResult", registry, PawStepPlanningResult.class);
   }

   @Override
   public void setInitialBodyPose(FramePose3DReadOnly initialPose)
   {
      bodyStartPose.setToZero(ReferenceFrame.getWorldFrame());
      bodyStartPose.setPosition(initialPose.getX(), initialPose.getY(), 0.0);
      bodyStartPose.setOrientationYawPitchRoll(initialPose.getYaw(), 0.0, 0.0);
   }

   @Override
   public void setGoal(PawStepPlannerGoal goal)
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
      Pose3D waypoint = new Pose3D();
      waypoint.getPosition().set(bodyStartPose.getPosition());
      waypoint.getPosition().add(goalDirection.getX(), goalDirection.getY(), 0.0);
      waypoint.getOrientation().setYawPitchRoll(BodyPathPlannerTools.calculateHeading(goalDirection), 0.0, 0.0);
      waypoints.add(waypoint);
   }

   @Override
   public List<Pose3DReadOnly> getWaypoints()
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
