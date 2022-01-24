package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
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
import us.ihmc.robotics.math.trajectories.core.Polynomial;
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
   private final Polynomial[] polynomials = {new Polynomial(12), new Polynomial(12), new Polynomial(12)};

   private final YoPreallocatedList<WaypointData> waypoints;

   private final List<Footstep> footsteps = new ArrayList<>();
   private final List<FootstepTiming> footstepTimings = new ArrayList<>();

   private final DoubleProvider time;
   private final YoDouble startTime = new YoDouble(namePrefix + "StartTime", registry);
   private final YoDouble totalDuration = new YoDouble(namePrefix + "TotalDuration", registry);
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   private final YoBoolean reset = new YoBoolean(namePrefix + "Reset", registry);
   private final YoFramePose3D currentPose = new YoFramePose3D(namePrefix + "CurrentPose", worldFrame, registry);
   private final YoFrameVector3D currentLinearVelocity = new YoFrameVector3D(namePrefix + "CurrentLinearVelocity", worldFrame, registry);
   private final YoDouble currentYawRate = new YoDouble(namePrefix + "CurrentYawRate", registry);
   private final YoFramePose3D lastPose = new YoFramePose3D(namePrefix + "LastPose", worldFrame, registry);
   private final YoFrameVector3D lastLinearVelocity = new YoFrameVector3D(namePrefix + "LastLinearVelocity", worldFrame, registry);
   private final YoDouble lastYawRate = new YoDouble(namePrefix + "LastYawRate", registry);

   private final FeetManager feetManager;

   public WalkingTrajectoryPath(FeetManager feetManager, HighLevelHumanoidControllerToolbox controllerToolbox, YoRegistry parentRegistry)
   {
      this.feetManager = feetManager;
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
      else
      {
         waypoints.getFirst().linearVelocity.setToZero();
         totalDuration.set(0.0);
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
      updateFirstWaypoint();
      updatePolynomials();

      double currentTime = MathTools.clamp(time.getValue() - startTime.getValue(), 0.0, totalDuration.getValue());

      for (Axis3D axis : Axis3D.values)
      {
         polynomials[axis.ordinal()].compute(currentTime);
         currentPose.getPosition().setElement(axis, polynomials[axis.ordinal()].getValue());
         currentLinearVelocity.setElement(axis, polynomials[axis.ordinal()].getVelocity());
      }

      lastPose.getPosition().set(currentPose.getPosition());
      lastLinearVelocity.set(currentLinearVelocity);
   }

   private void updateFirstWaypoint()
   {
      
   }

   private void updatePolynomials()
   {
      if (footsteps.isEmpty())
      {
         for (Axis3D axis : Axis3D.values)
         {
            Polynomial polynomial = polynomials[axis.ordinal()];
            polynomial.setConstant(waypoints.getFirst().getPosition(axis));
         }
      }
      else
      {
         for (Axis3D axis : Axis3D.values)
         {
            // With Polynomial
            Polynomial polynomial = polynomials[axis.ordinal()];
            polynomial.setTime(0, totalDuration.getValue());
            polynomial.reshape(waypoints.size() + 2);
            int row = 0;
            polynomial.setPositionRow(row++, 0.0, waypoints.getFirst().getPosition(axis));
            polynomial.setVelocityRow(row++, 0.0, waypoints.getFirst().getLinearVelocity(axis));

            for (int i = 1; i < waypoints.size(); i++)
            {
               polynomial.setPositionRow(row++, waypoints.get(i).time.getValue(), waypoints.get(i).getPosition(axis));
            }
            polynomial.setVelocityRow(row++, totalDuration.getValue(), 0.0);
            polynomial.setIsConstraintMatrixUpToDate(true);
            polynomial.initialize();

            for (int i = 1; i < waypoints.size() - 1; i++)
            {
               WaypointData waypoint = waypoints.get(i);
               polynomial.compute(waypoint.time.getValue());
               waypoint.setLinearVelocity(axis, polynomial.getVelocity());
            }
         }
      }
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
