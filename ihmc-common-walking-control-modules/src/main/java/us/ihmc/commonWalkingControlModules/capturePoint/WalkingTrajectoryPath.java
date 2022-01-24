package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.lists.YoPreallocatedList;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
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
   private final Polynomial[] linearPolynomials = {new Polynomial(12), new Polynomial(12), new Polynomial(12)};
   private final Polynomial yawPolynomial = new Polynomial(12);

   private final YoPreallocatedList<WaypointData> waypoints;

   private final List<Footstep> footsteps = new ArrayList<>();
   private final List<FootstepTiming> footstepTimings = new ArrayList<>();

   private final DoubleProvider time;
   private final YoDouble startTime = new YoDouble(namePrefix + "StartTime", registry);
   private final YoDouble totalDuration = new YoDouble(namePrefix + "TotalDuration", registry);
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   private final YoBoolean reset = new YoBoolean(namePrefix + "Reset", registry);
   private final YoFramePoint3D currentPosition = new YoFramePoint3D(namePrefix + "CurrentPosition", worldFrame, registry);
   private final YoDouble currentYaw = new YoDouble(namePrefix + "CurrentYaw", registry);
   private final YoFrameVector3D currentLinearVelocity = new YoFrameVector3D(namePrefix + "CurrentLinearVelocity", worldFrame, registry);
   private final YoDouble currentYawRate = new YoDouble(namePrefix + "CurrentYawRate", registry);
   private final YoFramePoint3D lastPosition = new YoFramePoint3D(namePrefix + "LastPosition", worldFrame, registry);
   private final YoDouble lastYaw = new YoDouble(namePrefix + "LastYaw", registry);
   private final YoFrameVector3D lastLinearVelocity = new YoFrameVector3D(namePrefix + "LastLinearVelocity", worldFrame, registry);
   private final YoDouble lastYawRate = new YoDouble(namePrefix + "LastYawRate", registry);

   private final MovingReferenceFrame walkingTrajectoryPathFrame = new MovingReferenceFrame("walkingTrajectoryPathFrame", worldFrame, true)
   {
      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.getTranslation().set(currentPosition);
         transformToParent.getRotation().setToYawOrientation(currentYaw.getValue());
      }

      @Override
      protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
      {
         twistRelativeToParentToPack.setToZero(walkingTrajectoryPathFrame, worldFrame, walkingTrajectoryPathFrame);
         twistRelativeToParentToPack.getLinearPart().setMatchingFrame(currentLinearVelocity);
         twistRelativeToParentToPack.getAngularPart().set(0.0, 0.0, currentYawRate.getValue());
      }
   };

   public WalkingTrajectoryPath(DoubleProvider time,
                                SideDependentList<MovingReferenceFrame> soleFrames,
                                YoGraphicsListRegistry yoGraphicsListRegistry,
                                YoRegistry parentRegistry)
   {
      this.time = time;
      this.soleFrames = soleFrames;

      YoGraphicsList yoGraphicList;
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
         waypoint.yaw.set(computeAverage(footPoses, waypoint.position));
         waypoint.linearVelocity.setToZero();
         waypoint.yawRate.set(0.0);
      }
      else
      {
         waypoint.position.set(lastPosition);
         waypoint.yaw.set(lastYaw.getValue());
         waypoint.linearVelocity.set(lastLinearVelocity);
         waypoint.yawRate.set(lastYawRate.getValue());
      }

      waypoint.updateViz();

      if (!footsteps.isEmpty())
      {
         WaypointData previousWaypoint = waypoint;

         for (int i = 0; i < footsteps.size(); i++)
         {
            waypoint = waypoints.add();
            Footstep footstep = footsteps.get(i);
            footPoses.get(footstep.getRobotSide()).set(footstep.getFootstepPose());
            double yaw = computeAverage(footPoses, waypoint.position);
            waypoint.yaw.set(previousWaypoint.yaw.getValue() + AngleTools.computeAngleDifferenceMinusPiToPi(yaw, previousWaypoint.yaw.getValue()));
            waypoint.time.set(previousWaypoint.time.getValue() + footstepTimings.get(i).getStepTime());
            waypoint.updateViz();
            previousWaypoint = waypoint;
         }

         waypoints.getLast().linearVelocity.setToZero();
         waypoints.getLast().yawRate.set(0.0);
         totalDuration.set(previousWaypoint.time.getValue());
      }
      else
      {
         waypoints.getFirst().linearVelocity.setToZero();
         totalDuration.set(0.0);
      }
   }

   private static double computeAverage(SideDependentList<? extends Pose3DReadOnly> input, Point3DBasics output)
   {
      Pose3DReadOnly leftPose = input.get(RobotSide.LEFT);
      Pose3DReadOnly rightPose = input.get(RobotSide.RIGHT);
      output.interpolate(leftPose.getPosition(), rightPose.getPosition(), 0.5);
      return AngleTools.computeAngleAverage(leftPose.getYaw(), rightPose.getYaw());
   }

   public void computeTrajectory()
   {
      updateFirstWaypoint();
      updatePolynomials();

      double currentTime = MathTools.clamp(time.getValue() - startTime.getValue(), 0.0, totalDuration.getValue());

      for (Axis3D axis : Axis3D.values)
      {
         linearPolynomials[axis.ordinal()].compute(currentTime);
         currentPosition.setElement(axis, linearPolynomials[axis.ordinal()].getValue());
         currentLinearVelocity.setElement(axis, linearPolynomials[axis.ordinal()].getVelocity());
      }

      yawPolynomial.compute(currentTime);
      currentYaw.set(AngleTools.trimAngleMinusPiToPi(yawPolynomial.getValue()));
      currentYawRate.set(yawPolynomial.getVelocity());

      lastPosition.set(currentPosition);
      lastYaw.set(currentYaw.getValue());
      lastLinearVelocity.set(currentLinearVelocity);
      lastYawRate.set(currentYawRate.getValue());

      walkingTrajectoryPathFrame.update();
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
            Polynomial polynomial = linearPolynomials[axis.ordinal()];
            polynomial.setConstant(waypoints.getFirst().getPosition(axis));
         }

         // Yaw part
         yawPolynomial.setConstant(waypoints.getFirst().yaw.getValue());
      }
      else
      {
         for (Axis3D axis : Axis3D.values)
         {
            Polynomial polynomial = linearPolynomials[axis.ordinal()];
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

         // Yaw part
         yawPolynomial.setTime(0, totalDuration.getValue());
         yawPolynomial.reshape(waypoints.size() + 2);
         int row = 0;
         yawPolynomial.setPositionRow(row++, 0.0, waypoints.getFirst().yaw.getValue());
         yawPolynomial.setVelocityRow(row++, 0.0, waypoints.getFirst().yawRate.getValue());

         for (int i = 1; i < waypoints.size(); i++)
         {
            yawPolynomial.setPositionRow(row++, waypoints.get(i).time.getValue(), waypoints.get(i).yaw.getValue());
         }
         yawPolynomial.setVelocityRow(row++, totalDuration.getValue(), 0.0);
         yawPolynomial.setIsConstraintMatrixUpToDate(true);
         yawPolynomial.initialize();

         for (int i = 1; i < waypoints.size() - 1; i++)
         {
            WaypointData waypoint = waypoints.get(i);
            yawPolynomial.compute(waypoint.time.getValue());
            waypoint.yawRate.set(yawPolynomial.getVelocity());
         }
      }
   }

   public MovingReferenceFrame getWalkingTrajectoryPathFrame()
   {
      return walkingTrajectoryPathFrame;
   }

   public static class WaypointData
   {
      private final String name;
      private final YoDouble time;
      private final YoFramePoint3D position;
      private final YoDouble yaw;
      private final YoFrameVector3D linearVelocity;
      private final YoDouble yawRate;

      private final YoFrameVector3D headingViz;
      private final YoGraphicVector waypointViz;

      private WaypointData(String namePrefix, String nameSuffix, YoRegistry registry, YoGraphicsList yoGraphicsList)
      {
         name = namePrefix + nameSuffix;
         time = new YoDouble(namePrefix + "Time" + nameSuffix, registry);
         position = new YoFramePoint3D(namePrefix + "Position" + nameSuffix, worldFrame, registry);
         yaw = new YoDouble(namePrefix + "Yaw" + nameSuffix, registry);
         linearVelocity = new YoFrameVector3D(namePrefix + "LinearVelocity" + nameSuffix, worldFrame, registry);
         yawRate = new YoDouble(namePrefix + "YawRate" + nameSuffix, registry);

         if (yoGraphicsList != null)
         {
            headingViz = new YoFrameVector3D(namePrefix + "HeadingViz" + nameSuffix, worldFrame, registry);
            waypointViz = new YoGraphicVector(namePrefix + "WaypointViz" + nameSuffix, position, headingViz, 0.3, YoAppearance.Orange());
            yoGraphicsList.add(waypointViz);
         }
         else
         {
            headingViz = null;
            waypointViz = null;
         }

         clear();
      }

      public void clear()
      {
         time.setToNaN();
         position.setToNaN();
         linearVelocity.setToNaN();
         yawRate.setToNaN();
      }

      public void updateViz()
      {
         if (headingViz != null)
            RotationMatrixTools.applyYawRotation(yaw.getValue(), Axis3D.X, headingViz);
      }

      public double getPosition(Axis3D axis)
      {
         return position.getElement(axis);
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
         return name + ", time: " + time.getValue() + ", pos: " + EuclidCoreIOTools.getTuple3DString(position) + ", yaw: " + yaw.getValue() + ", lin. vel.: "
               + EuclidCoreIOTools.getTuple3DString(linearVelocity) + ", yaw rate: " + yawRate.getValue();
      }
   }
}
