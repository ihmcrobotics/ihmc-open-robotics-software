package us.ihmc.commonWalkingControlModules.capturePoint;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.lists.YoPreallocatedList;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.math.trajectories.generators.MultiCubicSpline1DSolver;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class WalkingTrajectoryPath
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final String WALKING_TRAJECTORY_PATH_FRAME_NAME = "walkingTrajectoryPathFrame";
   public static final String WALKING_TRAJECTORY_FRAME_NAMEID = worldFrame.getNameId() + ReferenceFrame.SEPARATOR + WALKING_TRAJECTORY_PATH_FRAME_NAME;
   public static final int WALKING_TRAJECTORY_FRAME_ID = WALKING_TRAJECTORY_FRAME_NAMEID.hashCode();

   private static final int MAX_NUMBER_OF_FOOTSTEPS = 4;

   private final String namePrefix = "walkingTrajectoryPath";

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final TrajectoryManager trajectoryManager = new MultiCubicSplineBasedTrajectoryManager();

   private final YoPreallocatedList<WaypointData> waypoints;

   private final PreallocatedList<Footstep> footsteps = new PreallocatedList<>(Footstep.class, Footstep::new, MAX_NUMBER_OF_FOOTSTEPS);
   private final PreallocatedList<FootstepTiming> footstepTimings = new PreallocatedList<>(FootstepTiming.class, FootstepTiming::new, MAX_NUMBER_OF_FOOTSTEPS);

   private final DoubleProvider time;
   private final YoDouble startTime = new YoDouble(namePrefix + "StartTime", registry);
   private final YoDouble totalDuration = new YoDouble(namePrefix + "TotalDuration", registry);
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   private final YoBoolean reset = new YoBoolean(namePrefix + "Reset", registry);
   private final YoFramePoint3D currentPosition = new YoFramePoint3D(namePrefix + "CurrentPosition", worldFrame, registry);
   private final YoDouble currentYaw = new YoDouble(namePrefix + "CurrentYaw", registry);
   private final YoFrameVector3D currentLinearVelocity = new YoFrameVector3D(namePrefix + "CurrentLinearVelocity", worldFrame, registry);
   private final YoDouble currentYawRate = new YoDouble(namePrefix + "CurrentYawRate", registry);

   private double initialSupportFootYaw;
   private final FramePoint3D firstWaypointInSupportFootFrame = new FramePoint3D();

   private final BagOfBalls trajectoryPositionViz;
   private final YoFrameVector3D currentZUpViz;
   private final YoFrameVector3D currentHeadingViz;
   private final YoBoolean isInDoubleSupport = new YoBoolean(namePrefix + "DoubleSupport", registry);
   private final YoEnum<RobotSide> supportSide = new YoEnum<>(namePrefix + "SupportSide", registry, RobotSide.class, true);

   private final MovingReferenceFrame walkingTrajectoryPathFrame = new MovingReferenceFrame(WALKING_TRAJECTORY_PATH_FRAME_NAME, worldFrame, true)
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

   private final DoubleParameter filterBreakFrequency;

   private final double dt;
   private final ExecutionTimer timer = new ExecutionTimer(namePrefix + "Timer", registry);

   public WalkingTrajectoryPath(DoubleProvider time,
                                double updateDT,
                                SideDependentList<MovingReferenceFrame> soleFrames,
                                YoGraphicsListRegistry yoGraphicsListRegistry,
                                YoRegistry parentRegistry)
   {
      this.time = time;
      this.dt = updateDT;
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
                                           MAX_NUMBER_OF_FOOTSTEPS + 1);
      if (yoGraphicList != null)
      {
         currentZUpViz = new YoFrameVector3D(namePrefix + "CurrentZUp", worldFrame, registry);
         currentHeadingViz = new YoFrameVector3D(namePrefix + "CurrentHeading", worldFrame, registry);
         yoGraphicList.add(new YoGraphicVector(namePrefix + "CurrentHeadingViz", currentPosition, currentHeadingViz, 0.35, YoAppearance.Blue()));
         yoGraphicList.add(new YoGraphicVector(namePrefix + "CurrentZUpViz", currentPosition, currentZUpViz, 0.25, YoAppearance.Blue()));
         trajectoryPositionViz = new BagOfBalls(100, 0.005, "walkingPathViz", YoAppearance.Red(), registry, yoGraphicsListRegistry);
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicList);
      }
      else
      {
         currentZUpViz = null;
         currentHeadingViz = null;
         trajectoryPositionViz = null;
      }

      filterBreakFrequency = new DoubleParameter(namePrefix + "FilterBreakFrequency", registry, 0.5);

      reset();
      clearWaypoints();
      clearFootsteps();

      parentRegistry.addChild(registry);
   }

   private void clearWaypoints()
   {
      for (int i = 0; i < waypoints.size(); i++)
      {
         waypoints.get(i).clear();
      }
      waypoints.clear();
   }

   public void reset()
   {
      reset.set(true);
   }

   public void clearFootsteps()
   {
      footsteps.clear();
      footstepTimings.clear();
   }

   public void addFootsteps(WalkingMessageHandler walkingMessageHandler)
   {
      for (int i = 0; i < walkingMessageHandler.getCurrentNumberOfFootsteps(); i++)
      {
         if (footsteps.remaining() == 0)
            break;

         walkingMessageHandler.peekFootstep(i, footsteps.add());
         walkingMessageHandler.peekTiming(i, footstepTimings.add());
      }
   }

   public void addFootstep(Footstep footstep, FootstepTiming footstepTiming)
   {
      if (footsteps.remaining() == 0)
         return;

      footsteps.add().set(footstep);
      footstepTimings.add().set(footstepTiming);
   }

   private final SideDependentList<Pose3D> supportFootPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private final SideDependentList<Pose3D> tempFootPoses = new SideDependentList<>(new Pose3D(), new Pose3D());

   public void initializeDoubleSupport()
   {
      initializeInternal(true, null);
   }

   public void initializeSingleSupport(RobotSide supportSide)
   {
      initializeInternal(false, supportSide);
   }

   private void initializeInternal(boolean isInDoubleSupport, RobotSide supportSide)
   {
      startTime.set(time.getValue());
      this.isInDoubleSupport.set(isInDoubleSupport);
      this.supportSide.set(supportSide);

      if (isInDoubleSupport)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            supportFootPoses.get(robotSide).set(soleFrames.get(robotSide).getTransformToRoot());
         }
      }
      else
      {
         supportFootPoses.get(supportSide).set(soleFrames.get(supportSide).getTransformToRoot());
      }

      clearWaypoints();

      WaypointData waypoint = waypoints.add();
      waypoint.time.set(0.0);

      if (reset.getValue())
      {
         reset.set(false);
         waypoint.setYaw(computeAverage(supportFootPoses, waypoint.position));
         waypoint.linearVelocity.setToZero();
         waypoint.setYawRate(0.0);
      }
      else
      {
         waypoint.position.set(currentPosition);
         waypoint.setYaw(currentYaw.getValue());
         waypoint.linearVelocity.set(currentLinearVelocity);
         waypoint.setYawRate(currentYawRate.getValue());
      }

      waypoint.updateViz();

      if (!isInDoubleSupport)
      {
         initialSupportFootYaw = soleFrames.get(supportSide).getTransformToRoot().getRotation().getYaw();
         firstWaypointInSupportFootFrame.setIncludingFrame(waypoint.position);
         firstWaypointInSupportFootFrame.changeFrame(soleFrames.get(supportSide));

         if (!footsteps.isEmpty())
         { // We already did the transfer, we're zeroing it out to shift the first waypoint time.
            footstepTimings.getFirst().setTransferTime(0.0);
         }
      }

      updateFootstepsInternal();
   }

   public void updateFootsteps(ConstraintType leftFootConstraintType, ConstraintType rightFootConstraintType)
   {
      updateSupportFootPoses(leftFootConstraintType, rightFootConstraintType);
      updateFootstepsInternal();
   }

   private void updateFootstepsInternal()
   {
      if (!footsteps.isEmpty())
      {
         WaypointData waypoint = waypoints.getFirst();
         WaypointData previousWaypoint = waypoint;

         for (RobotSide robotSide : RobotSide.values)
         {
            tempFootPoses.get(robotSide).set(supportFootPoses.get(robotSide));
         }

         for (int i = 0; i < footsteps.size(); i++)
         {
            int waypointIndex = i + 1;
            waypoint = waypoints.size() == waypointIndex ? waypoints.add() : waypoints.get(waypointIndex);
            Footstep footstep = footsteps.get(i);
            tempFootPoses.get(footstep.getRobotSide()).set(footstep.getFootstepPose());
            double yaw = computeAverage(tempFootPoses, waypoint.position);
            waypoint.setYaw(previousWaypoint.getYaw() + AngleTools.computeAngleDifferenceMinusPiToPi(yaw, previousWaypoint.getYaw()));
            waypoint.time.set(previousWaypoint.time.getValue() + footstepTimings.get(i).getStepTime());
            waypoint.updateViz();
            previousWaypoint = waypoint;
         }

         waypoints.getLast().linearVelocity.setToZero();
         waypoints.getLast().setYawRate(0.0);
         totalDuration.set(previousWaypoint.time.getValue());
      }
      else
      {
         waypoints.getFirst().linearVelocity.setToZero();
         totalDuration.set(0.0);
      }
   }

   public void computeTrajectory(ConstraintType leftFootConstraintType, ConstraintType rightFootConstraintType)
   {
      timer.startMeasurement();

      updateSupportFootPoses(leftFootConstraintType, rightFootConstraintType);
      updateWaypoints();
      updatePolynomials();

      double currentTime = MathTools.clamp(time.getValue() - startTime.getValue(), 0.0, totalDuration.getValue());

      if (footsteps.isEmpty())
      {
         WaypointData firstWaypoint = waypoints.getFirst();
         currentPosition.set(firstWaypoint.position);
         currentLinearVelocity.set(firstWaypoint.linearVelocity);
         currentYaw.set(AngleTools.trimAngleMinusPiToPi(firstWaypoint.getYaw()));
         currentYawRate.set(firstWaypoint.getYawRate());
      }
      else
      {
         trajectoryManager.computePosition(currentTime, currentPosition);
         trajectoryManager.computeLinearVelocity(currentTime, currentLinearVelocity);
         currentYaw.set(AngleTools.trimAngleMinusPiToPi(trajectoryManager.computeYaw(currentTime)));
         currentYawRate.set(trajectoryManager.computeYawRate(currentTime));
      }

      walkingTrajectoryPathFrame.update();

      updateViz();

      timer.stopMeasurement();
   }

   public void updateSupportFootPoses(ConstraintType leftFootConstraintType, ConstraintType rightFootConstraintType)
   {
      if (leftFootConstraintType == ConstraintType.FULL)
         supportFootPoses.get(RobotSide.LEFT).set(soleFrames.get(RobotSide.LEFT).getTransformToRoot());
      if (rightFootConstraintType == ConstraintType.FULL)
         supportFootPoses.get(RobotSide.RIGHT).set(soleFrames.get(RobotSide.RIGHT).getTransformToRoot());
   }

   private final Point3D tempBallPosition = new Point3D();

   private void updateViz()
   {
      if (trajectoryPositionViz == null)
         return;

      RotationMatrixTools.applyYawRotation(currentYaw.getValue(), Axis3D.X, currentHeadingViz);
      currentZUpViz.set(Axis3D.Z);

      for (int i = 0; i < trajectoryPositionViz.getNumberOfBalls(); i++)
      {
         double t = ((double) i) / (trajectoryPositionViz.getNumberOfBalls() - 1.0) * totalDuration.getValue();

         if (footsteps.isEmpty())
            tempBallPosition.set(waypoints.getFirst().position);
         else
            trajectoryManager.computePosition(t, tempBallPosition);
         trajectoryPositionViz.setBall(tempBallPosition, i);
      }
   }

   private final Point3D newWaypointPosition = new Point3D();

   private void updateWaypoints()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         tempFootPoses.get(robotSide).set(supportFootPoses.get(robotSide));
      }

      double filterAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filterBreakFrequency.getValue(), dt);

      WaypointData firstWaypoint = waypoints.getFirst();

      if (isInDoubleSupport.getValue())
      {
         double newYaw = computeAverage(tempFootPoses, newWaypointPosition);
         updateFilteredWaypoint(firstWaypoint, newWaypointPosition, newYaw, filterAlpha);
      }
      else
      {
         MovingReferenceFrame supportFootFrame = soleFrames.get(supportSide.getValue());
         newWaypointPosition.set(firstWaypointInSupportFootFrame);
         supportFootFrame.transformFromThisToDesiredFrame(worldFrame, newWaypointPosition);
         double firstWaypointYaw = firstWaypoint.getYaw() - initialSupportFootYaw;
         double newYaw = firstWaypointYaw + supportFootFrame.getTransformToRoot().getRotation().getYaw();
         updateFilteredWaypoint(firstWaypoint, newWaypointPosition, newYaw, filterAlpha);
      }

      if (!footsteps.isEmpty())
      {
         Footstep firstFootstep = footsteps.get(0);
         tempFootPoses.get(firstFootstep.getRobotSide()).set(firstFootstep.getFootstepPose());
         double newYaw = computeAverage(tempFootPoses, newWaypointPosition);
         WaypointData secondWaypoint = waypoints.get(1);
         updateFilteredWaypoint(secondWaypoint, newWaypointPosition, newYaw, filterAlpha);
         secondWaypoint.setYaw(firstWaypoint.getYaw() + AngleTools.computeAngleDifferenceMinusPiToPi(secondWaypoint.getYaw(), firstWaypoint.getYaw()));
      }
   }

   public static void updateFilteredWaypoint(WaypointData waypoint, Point3DReadOnly newPosition, double newYaw, double alpha)
   {
      waypoint.position.interpolate(newPosition, waypoint.position, alpha);
      waypoint.setYaw(AngleTools.interpolateAngle(newYaw, waypoint.getYaw(), alpha));
   }

   private void updatePolynomials()
   {
      if (!footsteps.isEmpty())
      {
         trajectoryManager.initialize(waypoints);
         for (int i = 1; i < waypoints.size() - 1; i++)
         {
            WaypointData waypoint = waypoints.get(i);
            trajectoryManager.computeLinearVelocity(waypoint.time.getValue(), waypoint.linearVelocity);
            waypoint.setYawRate(trajectoryManager.computeYawRate(waypoint.time.getValue()));
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

      private final YoFrameVector3D zUpViz;
      private final YoFrameVector3D headingViz;
      private final YoGraphicVector waypointZUpViz;
      private final YoGraphicVector waypointHeadingViz;

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
            zUpViz = new YoFrameVector3D(namePrefix + "ZUpViz" + nameSuffix, worldFrame, registry);
            headingViz = new YoFrameVector3D(namePrefix + "HeadingViz" + nameSuffix, worldFrame, registry);
            waypointZUpViz = new YoGraphicVector(nameSuffix + "WaypointZUpViz" + nameSuffix, position, zUpViz, 0.1, YoAppearance.Orange());
            waypointHeadingViz = new YoGraphicVector(namePrefix + "WaypointHeadingViz" + nameSuffix, position, headingViz, 0.3, YoAppearance.Orange());
            yoGraphicsList.add(waypointZUpViz);
            yoGraphicsList.add(waypointHeadingViz);
         }
         else
         {
            zUpViz = null;
            headingViz = null;
            waypointZUpViz = null;
            waypointHeadingViz = null;
         }

         clear();
      }

      public void clear()
      {
         time.setToNaN();
         position.setToNaN();
         yaw.setToNaN();
         linearVelocity.setToNaN();
         yawRate.setToNaN();
      }

      public void updateViz()
      {
         if (headingViz != null)
         {
            zUpViz.set(Axis3D.Z);
            RotationMatrixTools.applyYawRotation(getYaw(), Axis3D.X, headingViz);
         }
      }

      public double getPosition(Axis3D axis)
      {
         return position.getElement(axis);
      }

      public double getYaw()
      {
         return yaw.getValue();
      }

      public double getLinearVelocity(Axis3D axis)
      {
         return linearVelocity.getElement(axis);
      }

      public double getYawRate()
      {
         return yawRate.getValue();
      }

      public void setYaw(double yaw)
      {
         this.yaw.set(yaw);
      }

      public void setLinearVelocity(Axis3D axis, double value)
      {
         linearVelocity.setElement(axis, value);
      }

      public void setYawRate(double yawRate)
      {
         this.yawRate.set(yawRate);
      }

      @Override
      public String toString()
      {
         return name + ", time: " + time.getValue() + ", pos: " + EuclidCoreIOTools.getTuple3DString(position) + ", yaw: " + yaw.getValue() + ", lin. vel.: "
               + EuclidCoreIOTools.getTuple3DString(linearVelocity) + ", yaw rate: " + yawRate.getValue();
      }
   }

   private static interface TrajectoryManager
   {
      void initialize(YoPreallocatedList<WaypointData> waypoints);

      void computePosition(double time, Tuple3DBasics positionToPack);

      double computeYaw(double time);

      void computeLinearVelocity(double time, Tuple3DBasics velocityToPack);

      double computeYawRate(double time);
   }

   static class MultiCubicSplineBasedTrajectoryManager implements TrajectoryManager
   {
      private final DMatrixRMaj[] linearSolutions = {new DMatrixRMaj(1, 1), new DMatrixRMaj(1, 1), new DMatrixRMaj(1, 1)};
      private final DMatrixRMaj yawSolution = new DMatrixRMaj(1, 1);
      private final MultiCubicSpline1DSolver[] linearSolvers = {new MultiCubicSpline1DSolver(), new MultiCubicSpline1DSolver(), new MultiCubicSpline1DSolver()};
      private final MultiCubicSpline1DSolver yawSolver = new MultiCubicSpline1DSolver();
      private double totalDuration;

      @Override
      public void initialize(YoPreallocatedList<WaypointData> waypoints)
      {
         WaypointData firstWaypoint = waypoints.getFirst();
         WaypointData lastWaypoint = waypoints.getLast();
         totalDuration = lastWaypoint.time.getValue();

         for (Axis3D axis : Axis3D.values)
         {
            MultiCubicSpline1DSolver linearSolver = linearSolvers[axis.ordinal()];
            linearSolver.clearWaypoints();
            linearSolver.clearWeights();
            linearSolver.setEndpoints(firstWaypoint.getPosition(axis),
                                      firstWaypoint.getLinearVelocity(axis) * totalDuration,
                                      lastWaypoint.getPosition(axis),
                                      lastWaypoint.getLinearVelocity(axis) * totalDuration);

            for (int i = 1; i < waypoints.size() - 1; i++)
            {
               WaypointData waypoint = waypoints.get(i);
               linearSolver.addWaypoint(waypoint.getPosition(axis), waypoint.time.getValue() / totalDuration);
            }

            linearSolver.solve(linearSolutions[axis.ordinal()]);
         }

         yawSolver.clearWaypoints();
         yawSolver.clearWeights();
         yawSolver.setEndpoints(firstWaypoint.yaw.getValue(),
                                firstWaypoint.yawRate.getValue() * totalDuration,
                                lastWaypoint.yaw.getValue(),
                                lastWaypoint.yawRate.getValue() * totalDuration);

         for (int i = 1; i < waypoints.size() - 1; i++)
         {
            WaypointData waypoint = waypoints.get(i);
            yawSolver.addWaypoint(waypoint.yaw.getValue(), waypoint.time.getValue() / totalDuration);
         }

         yawSolver.solve(yawSolution);
      }

      @Override
      public void computePosition(double time, Tuple3DBasics positionToPack)
      {
         positionToPack.setX(linearSolvers[0].computePosition(time / totalDuration, linearSolutions[0]));
         positionToPack.setY(linearSolvers[1].computePosition(time / totalDuration, linearSolutions[1]));
         positionToPack.setZ(linearSolvers[2].computePosition(time / totalDuration, linearSolutions[2]));
      }

      @Override
      public double computeYaw(double time)
      {
         return yawSolver.computePosition(time / totalDuration, yawSolution);
      }

      @Override
      public void computeLinearVelocity(double time, Tuple3DBasics velocityToPack)
      {
         velocityToPack.setX(linearSolvers[0].computeVelocity(time / totalDuration, linearSolutions[0]));
         velocityToPack.setY(linearSolvers[1].computeVelocity(time / totalDuration, linearSolutions[1]));
         velocityToPack.setZ(linearSolvers[2].computeVelocity(time / totalDuration, linearSolutions[2]));
         velocityToPack.scale(1.0 / totalDuration);
      }

      @Override
      public double computeYawRate(double time)
      {
         return yawSolver.computeVelocity(time / totalDuration, yawSolution) / totalDuration;
      }
   }

   static class PolynomialBasedTrajectoryManager implements TrajectoryManager
   {
      private final Polynomial[] linearPolynomials = {new Polynomial(MAX_NUMBER_OF_FOOTSTEPS + 1 + 2), new Polynomial(MAX_NUMBER_OF_FOOTSTEPS + 1 + 2),
            new Polynomial(MAX_NUMBER_OF_FOOTSTEPS + 1 + 2)};
      private final Polynomial yawPolynomial = new Polynomial(MAX_NUMBER_OF_FOOTSTEPS + 1 + 2);
      private double totalDuration;

      @Override
      public void initialize(YoPreallocatedList<WaypointData> waypoints)
      {
         WaypointData lastWaypoint = waypoints.getLast();
         totalDuration = lastWaypoint.time.getValue();

         if (waypoints.size() == 1)
         {
            for (Axis3D axis : Axis3D.values)
            {
               Polynomial polynomial = linearPolynomials[axis.ordinal()];
               polynomial.setConstant(waypoints.getFirst().getPosition(axis));
            }

            yawPolynomial.setConstant(waypoints.getFirst().yaw.getValue());
         }
         else
         {
            for (Axis3D axis : Axis3D.values)
            {
               Polynomial polynomial = linearPolynomials[axis.ordinal()];
               polynomial.setTime(0, totalDuration);
               polynomial.reshape(waypoints.size() + 2);
               int row = 0;

               polynomial.setPositionRow(row++, 0.0, waypoints.getFirst().getPosition(axis));
               polynomial.setVelocityRow(row++, 0.0, waypoints.getFirst().getLinearVelocity(axis));

               for (int i = 1; i < waypoints.size(); i++)
               {
                  polynomial.setPositionRow(row++, waypoints.get(i).time.getValue(), waypoints.get(i).getPosition(axis));
               }
               polynomial.setVelocityRow(row++, totalDuration, 0.0);
               polynomial.setIsConstraintMatrixUpToDate(true);
               polynomial.initialize();

               for (int i = 1; i < waypoints.size() - 1; i++)
               {
                  WaypointData waypoint = waypoints.get(i);
                  polynomial.compute(waypoint.time.getValue());
                  waypoint.setLinearVelocity(axis, polynomial.getVelocity());
               }
            }

            yawPolynomial.setTime(0, totalDuration);
            yawPolynomial.reshape(waypoints.size() + 2);
            int row = 0;

            yawPolynomial.setPositionRow(row++, 0.0, waypoints.getFirst().yaw.getValue());
            yawPolynomial.setVelocityRow(row++, 0.0, waypoints.getFirst().yawRate.getValue());

            for (int i = 1; i < waypoints.size(); i++)
            {
               yawPolynomial.setPositionRow(row++, waypoints.get(i).time.getValue(), waypoints.get(i).yaw.getValue());
            }
            yawPolynomial.setVelocityRow(row++, totalDuration, 0.0);
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

      @Override
      public void computePosition(double time, Tuple3DBasics positionToPack)
      {
         for (Axis3D axis : Axis3D.values)
            linearPolynomials[axis.ordinal()].compute(time);

         positionToPack.setX(linearPolynomials[0].getValue());
         positionToPack.setY(linearPolynomials[1].getValue());
         positionToPack.setZ(linearPolynomials[2].getValue());
      }

      @Override
      public double computeYaw(double time)
      {
         yawPolynomial.compute(time);
         return yawPolynomial.getValue();
      }

      @Override
      public void computeLinearVelocity(double time, Tuple3DBasics velocityToPack)
      {
         for (Axis3D axis : Axis3D.values)
            linearPolynomials[axis.ordinal()].compute(time);

         velocityToPack.setX(linearPolynomials[0].getVelocity());
         velocityToPack.setY(linearPolynomials[1].getVelocity());
         velocityToPack.setZ(linearPolynomials[2].getVelocity());
      }

      @Override
      public double computeYawRate(double time)
      {
         yawPolynomial.compute(time);
         return yawPolynomial.getVelocity();
      }
   }

   private static double computeAverage(SideDependentList<? extends Pose3DReadOnly> input, Point3DBasics output)
   {
      return computeAverage(input.get(RobotSide.LEFT), input.get(RobotSide.RIGHT), output);
   }

   private static double computeAverage(Pose3DReadOnly firstPose, Pose3DReadOnly secondPose, Point3DBasics output)
   {
      output.interpolate(firstPose.getPosition(), secondPose.getPosition(), 0.5);
      return AngleTools.computeAngleAverage(firstPose.getYaw(), secondPose.getYaw());
   }
}
