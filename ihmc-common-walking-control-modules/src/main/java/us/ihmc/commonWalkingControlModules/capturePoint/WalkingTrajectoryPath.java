package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.lists.YoPreallocatedList;
import us.ihmc.robotics.math.trajectories.generators.MultiCubicSpline1DSolver;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkingTrajectoryPath
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String namePrefix = "walkingTrajectoryPath";

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final MultiCubicSpline1DSolver solvers[] = {new MultiCubicSpline1DSolver(), new MultiCubicSpline1DSolver(), new MultiCubicSpline1DSolver()};

   private final YoPreallocatedList<WaypointData> waypoints;

   private final List<Footstep> footsteps = new ArrayList<>();
   private final List<FootstepTiming> footstepTimings = new ArrayList<>();

   private final DoubleProvider time;
   private final YoDouble startTime = new YoDouble(namePrefix + "StartTime", registry);
   private final YoDouble totalDuration = new YoDouble(namePrefix + "TotalDuration", registry);
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   private final MultipleWaypointsPositionTrajectoryGenerator positionTrajectory = new MultipleWaypointsPositionTrajectoryGenerator(namePrefix,
                                                                                                                                    10,
                                                                                                                                    worldFrame,
                                                                                                                                    registry);
   private final YoBoolean reset = new YoBoolean(namePrefix + "Reset", registry);
   private final YoFramePose3D lastPose = new YoFramePose3D(namePrefix + "LastPose", worldFrame, registry);
   private final YoFrameVector3D lastLinearVelocity = new YoFrameVector3D(namePrefix + "LastLinearVelocity", worldFrame, registry);
   private final YoDouble lastYawRate = new YoDouble(namePrefix + "LastYawRate", registry);

   public WalkingTrajectoryPath(HighLevelHumanoidControllerToolbox controllerToolbox, YoRegistry parentRegistry)
   {
      time = controllerToolbox.getYoTime();
      soleFrames = controllerToolbox.getReferenceFrames().getSoleFrames();

      YoGraphicsList yoGraphicList;
      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      if (yoGraphicsListRegistry != null)
         yoGraphicList = new YoGraphicsList(namePrefix);
      else
         yoGraphicList = null;
      waypoints = new YoPreallocatedList<>(WaypointData.class,
                                           SupplierBuilder.indexedSupplier(i -> new WaypointData(namePrefix, Integer.toString(i), registry, yoGraphicList)),
                                           "walkingPath",
                                           registry,
                                           10);
      if (yoGraphicList != null)
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicList);

      reset();
      clearFootsteps();

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      reset.set(true);
   }

   public void clearFootsteps()
   {
      for (int i = 0; i < waypoints.size(); i++)
      {
         waypoints.get(i).clear();
      }
      waypoints.clear();
      footsteps.clear();
      footstepTimings.clear();
   }

   public void addFootstep(Footstep footstep, FootstepTiming footstepTiming)
   {
      footsteps.add(footstep);
      footstepTimings.add(footstepTiming);
   }

   private final SideDependentList<Pose3D> footPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private final DMatrixRMaj solution = new DMatrixRMaj(10, 1);

   public void initialize()
   {
      startTime.set(time.getValue());

      for (RobotSide robotSide : RobotSide.values)
         footPoses.get(robotSide).set(soleFrames.get(robotSide).getTransformToRoot());

      WaypointData waypoint = waypoints.add();
      waypoint.time.set(0.0);

      if (reset.getValue())
      {
         reset.set(false);
         computeAverage(footPoses, waypoint.pose);
         waypoint.linearVelocity.setToZero();
      }
      else
      {
         waypoint.pose.set(lastPose);
         waypoint.linearVelocity.set(lastLinearVelocity);
      }
      computeAverage(footPoses, waypoint.pose);

      if (!footsteps.isEmpty())
      {
         WaypointData previousWaypoint = waypoint;

         for (int i = 0; i < footsteps.size(); i++)
         {
            waypoint = waypoints.add();
            Footstep footstep = footsteps.get(i);
            footPoses.get(footstep.getRobotSide()).set(footstep.getFootstepPose());
            computeAverage(footPoses, waypoint.pose);
            waypoint.time.set(previousWaypoint.time.getValue() + footstepTimings.get(i).getStepTime());
            previousWaypoint = waypoint;
         }

         waypoints.getLast().linearVelocity.setToZero();
         totalDuration.set(previousWaypoint.time.getValue());
      }
   }

   private static void computeAverage(SideDependentList<? extends Pose3DReadOnly> input, Pose3DBasics output)
   {
      Pose3DReadOnly leftPose = input.get(RobotSide.LEFT);
      Pose3DReadOnly rightPose = input.get(RobotSide.RIGHT);
      output.getPosition().interpolate(leftPose.getPosition(), rightPose.getPosition(), 0.5);
      output.getOrientation().setToYawOrientation(0.5 * (leftPose.getYaw() + rightPose.getYaw()));
   }

   public void computeTrajectory()
   {
      if (!footsteps.isEmpty())
      {
         for (Axis3D axis : Axis3D.values)
         {
            WaypointData firstWaypoint = waypoints.getFirst();
            WaypointData lastWaypoint = waypoints.getLast();

            lastWaypoint.linearVelocity.setToZero();
            MultiCubicSpline1DSolver solver = solvers[axis.ordinal()];
            solver.clearWaypoints();
            solver.clearWeights();
            solver.setEndpoints(firstWaypoint.getPosition(axis), firstWaypoint.getLinearVelocity(axis), lastWaypoint.getPosition(axis), 0.0);

            for (int i = 1; i < waypoints.size() - 1; i++)
            {
               WaypointData waypoint = waypoints.get(i);
               solver.addWaypoint(waypoint.getPosition(axis), waypoint.time.getValue() / totalDuration.getValue());
            }

            solver.solve(solution);

            for (int i = 1; i < waypoints.size() - 1; i++)
            {
               WaypointData waypoint = waypoints.get(i);
               waypoint.setLinearVelocity(axis, solver.computeWaypointVelocityFromSolution(i - 1, solution) / totalDuration.getValue());
            }
         }
      }

      positionTrajectory.clear();

      for (int i = 0; i < waypoints.size(); i++)
      {
         WaypointData waypoint = waypoints.get(i);
         positionTrajectory.appendWaypoint(waypoint.time.getValue(), waypoint.pose.getPosition(), waypoint.linearVelocity);
      }

      positionTrajectory.initialize();

      double currentTime = time.getValue() - startTime.getValue();
      positionTrajectory.compute(currentTime);

      lastPose.getPosition().set(positionTrajectory.getPosition());
      lastLinearVelocity.set(positionTrajectory.getVelocity());
   }

   public static class WaypointData
   {
      private final String name;
      private final YoDouble time;
      private final YoFramePose3D pose;
      private final YoFrameVector3D linearVelocity;
      private final YoDouble yawRate;

      private final YoGraphicCoordinateSystem poseViz;

      private WaypointData(String namePrefix, String nameSuffix, YoRegistry registry, YoGraphicsList yoGraphicsList)
      {
         name = namePrefix + nameSuffix;
         time = new YoDouble(namePrefix + "Time" + nameSuffix, registry);
         pose = new YoFramePose3D(namePrefix + "Pose" + nameSuffix, worldFrame, registry);
         linearVelocity = new YoFrameVector3D(namePrefix + "LinearVelocity" + nameSuffix, worldFrame, registry);
         yawRate = new YoDouble(namePrefix + "YawRate" + nameSuffix, registry);

         if (yoGraphicsList != null)
         {
            poseViz = new YoGraphicCoordinateSystem(namePrefix + "PoseViz" + nameSuffix, pose, 0.3, YoAppearance.Orange());
            yoGraphicsList.add(poseViz);
         }
         else
         {
            poseViz = null;
         }

         clear();
      }

      public void clear()
      {
         time.setToNaN();
         pose.setToNaN();
         linearVelocity.setToNaN();
         yawRate.setToNaN();
      }

      public double getPosition(Axis3D axis)
      {
         return pose.getPosition().getElement(axis);
      }

      public double getLinearVelocity(Axis3D axis)
      {
         return linearVelocity.getElement(axis);
      }

      public void setLinearVelocity(Axis3D axis, double value)
      {
         linearVelocity.setElement(axis, value);
      }

      @Override
      public String toString()
      {
         return name + ", time: " + time.getValue() + ", pos: " + EuclidCoreIOTools.getTuple3DString(pose.getPosition()) + ", lin. vel.: "
               + EuclidCoreIOTools.getTuple3DString(linearVelocity);
      }
   }
}
