package us.ihmc.commonWalkingControlModules.referenceFrames;

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
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.lists.YoPreallocatedList;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.frames.YoMatrix;
import us.ihmc.robotics.math.trajectories.generators.MultiCubicSpline1DSolver;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * This class sets up a trajectory that aims at providing a reference frame that follows the robot
 * as it walks, while following a smooth trajectory.
 * <p>
 * The reference frame satisfies the following:
 * <ul>
 * <li>It is a z-up frame.
 * <li>When the robot is standing, its position and yaw angle are computed as the average of the
 * soles position and yaw angles.
 * <li>It can be used in trajectory commands by providing {@value #WALKING_TRAJECTORY_FRAME_ID} as
 * the reference frame ID.
 * </ul>
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class WalkingTrajectoryPath implements SCS2YoGraphicHolder
{
   private static final boolean VISUALIZE = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final String WALKING_TRAJECTORY_PATH_FRAME_NAME = "walkingTrajectoryPathFrame";
   public static final String WALKING_TRAJECTORY_FRAME_NAMEID = worldFrame.getNameId() + ReferenceFrame.SEPARATOR + WALKING_TRAJECTORY_PATH_FRAME_NAME;
   public static final int WALKING_TRAJECTORY_FRAME_ID = WALKING_TRAJECTORY_PATH_FRAME_NAME.hashCode();

   private static final int MAX_NUMBER_OF_FOOTSTEPS = 2;

   private final String namePrefix = "walkingTrajectoryPath";

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final TrajectoryManager trajectoryManager;

   private final YoPreallocatedList<WaypointData> waypoints;

   private boolean dirtyFootsteps = false;
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

   private final YoFrameVector3D initialLinearVelocity = new YoFrameVector3D(namePrefix + "InitialLinearVelocity", worldFrame, registry);
   private final YoDouble initialYawRate = new YoDouble(namePrefix + "InitialYawRate", registry);

   private double initialSupportFootYaw;
   private final FramePoint3D firstWaypointInSupportFootFrame = new FramePoint3D();

   private final BagOfBalls trajectoryPositionViz;
   private final BagOfBalls waypointsWTPViz;
   private final YoFrameVector3D currentZUpViz;
   private final YoFrameVector3D currentHeadingViz;
   private final YoBoolean isInDoubleSupport = new YoBoolean(namePrefix + "DoubleSupport", registry);
   private final YoEnum<RobotSide> supportSide = new YoEnum<>(namePrefix + "SupportSide", registry, RobotSide.class, true);
   private final YoBoolean isLastWaypointOpen = new YoBoolean(namePrefix + "IsLastWaypointOpen", registry);

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
      dt = updateDT;
      this.soleFrames = soleFrames;

      trajectoryManager = new TrajectoryManager(registry);
      YoGraphicsList yoGraphicList;

      if (yoGraphicsListRegistry != null)
      {
         if (VISUALIZE)
         {
            yoGraphicList = new YoGraphicsList(namePrefix);

            currentZUpViz = new YoFrameVector3D(namePrefix + "CurrentZUp", worldFrame, registry);
            currentHeadingViz = new YoFrameVector3D(namePrefix + "CurrentHeading", worldFrame, registry);
            yoGraphicList.add(new YoGraphicVector(namePrefix + "CurrentHeadingViz", currentPosition, currentHeadingViz, 0.35, YoAppearance.Blue()));
            yoGraphicList.add(new YoGraphicVector(namePrefix + "CurrentZUpViz", currentPosition, currentZUpViz, 0.25, YoAppearance.Blue()));
            trajectoryPositionViz = new BagOfBalls(100, 0.005, "walkingPathViz", YoAppearance.Red(), registry, yoGraphicsListRegistry);
            waypointsWTPViz = new BagOfBalls(MAX_NUMBER_OF_FOOTSTEPS+1, 0.005, "waypointWTPViz", YoAppearance.Black(), registry, yoGraphicsListRegistry);
            yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicList);
         }
         else
         {
            yoGraphicList = null;

            currentZUpViz = null;
            currentHeadingViz = null;
            trajectoryPositionViz = null;
            waypointsWTPViz = null;
         }
      }
      else
      {
         yoGraphicList = null;

         currentZUpViz = null;
         currentHeadingViz = null;
         trajectoryPositionViz = null;
         waypointsWTPViz = null;
      }

      waypoints = new YoPreallocatedList<>(WaypointData.class,
                                           SupplierBuilder.indexedSupplier(i -> new WaypointData(namePrefix, Integer.toString(i), registry, yoGraphicList)),
                                           "walkingPath",
                                           registry,
                                           MAX_NUMBER_OF_FOOTSTEPS + 1);

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
      dirtyFootsteps = true;
      footsteps.clear();
      footstepTimings.clear();
   }

   double finalTransferDuration = 0.0;
   public void addFootsteps(WalkingMessageHandler walkingMessageHandler)
   {
      dirtyFootsteps = true;

      setLastWaypointOpen(footsteps.remaining() < walkingMessageHandler.getCurrentNumberOfFootsteps());

      for (int i = 0; i < walkingMessageHandler.getCurrentNumberOfFootsteps(); i++)
      {
         if (footsteps.remaining() == 0)
            break;

         walkingMessageHandler.peekFootstep(i, footsteps.add());
         walkingMessageHandler.peekTiming(i, footstepTimings.add());
      }
      finalTransferDuration = walkingMessageHandler.getFinalTransferTime();
   }

   public void addFootstep(Footstep footstep, FootstepTiming footstepTiming)
   {
      if (footsteps.remaining() == 0)
         return;

      dirtyFootsteps = true;
      footsteps.add().set(footstep);
      footstepTimings.add().set(footstepTiming);
   }

   /**
    * Sets whether the last waypoint is open or not.
    * <p>
    * The last waypoint should be open when the last footstep added to the trajectory is not the last
    * footstep of the current walking sequence. When the last waypoint is open, the final velocity of
    * the trajectory is left unconstrained. When the last waypoint is closed, the final velocity is
    * constrained to 0, such as to plan for the robot coming to a stop.
    * </p>
    */
   public void setLastWaypointOpen(boolean isOpen)
   {
      isLastWaypointOpen.set(isOpen);
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
         initialLinearVelocity.setToZero();
         initialYawRate.set(0.0);
      }
      else
      {
         waypoint.position.set(currentPosition);
         waypoint.setYaw(currentYaw.getValue());
         initialLinearVelocity.set(currentLinearVelocity);
         initialYawRate.set(currentYawRate.getValue());
      }

      waypoint.updateViz();

      if (!isInDoubleSupport)
      {
         initialSupportFootYaw = soleFrames.get(supportSide).getTransformToRoot().getRotation().getYaw();
         firstWaypointInSupportFootFrame.setIncludingFrame(waypoint.position);
         firstWaypointInSupportFootFrame.changeFrame(soleFrames.get(supportSide));
      }

      updateFootstepsInternal();
   }

   public void updateTrajectory(ConstraintType leftFootConstraintType, ConstraintType rightFootConstraintType)
   {
      timer.startMeasurement();

      updateSupportFootPoses(leftFootConstraintType, rightFootConstraintType);
      updateFootstepsInternal();
      updateWaypoints();

      double currentTime = MathTools.clamp(time.getValue() - startTime.getValue(), 0.0, totalDuration.getValue());

      if (footsteps.isEmpty())
      {
         WaypointData firstWaypoint = waypoints.getFirst();
         currentPosition.set(firstWaypoint.position);
         currentLinearVelocity.set(initialLinearVelocity);
         currentYaw.set(AngleTools.trimAngleMinusPiToPi(firstWaypoint.getYaw()));
         currentYawRate.set(initialYawRate.getValue());
      }
      else
      {
         trajectoryManager.initialize(initialLinearVelocity, initialYawRate.getValue(), waypoints, isLastWaypointOpen.getValue());
         trajectoryManager.computePosition(currentTime, currentPosition);
         trajectoryManager.computeLinearVelocity(currentTime, currentLinearVelocity);
         currentYaw.set(AngleTools.trimAngleMinusPiToPi(trajectoryManager.computeYaw(currentTime)));
         currentYawRate.set(trajectoryManager.computeYawRate(currentTime));
      }

      walkingTrajectoryPathFrame.update();

      updateViz();

      timer.stopMeasurement();
   }

   private void updateSupportFootPoses(ConstraintType leftFootConstraintType, ConstraintType rightFootConstraintType)
   {
      if (leftFootConstraintType == ConstraintType.FULL)
         supportFootPoses.get(RobotSide.LEFT).set(soleFrames.get(RobotSide.LEFT).getTransformToRoot());
      if (rightFootConstraintType == ConstraintType.FULL)
         supportFootPoses.get(RobotSide.RIGHT).set(soleFrames.get(RobotSide.RIGHT).getTransformToRoot());
   }

   private void updateFootstepsInternal()
   {
      if (!dirtyFootsteps)
         return;

      dirtyFootsteps = false;

      if (!footsteps.isEmpty())
      {
         WaypointData waypoint = waypoints.getFirst();
         WaypointData previousWaypoint = waypoint;

         for (RobotSide robotSide : RobotSide.values)
         {
            tempFootPoses.get(robotSide).set(supportFootPoses.get(robotSide));
         }

         if (!isInDoubleSupport.getValue())
         {// We already did the first transfer, we're zeroing it out to shift the first waypoint time.
            footstepTimings.getFirst().setTransferTime(0.0);
         }

         for (int i = 0; i < footsteps.size(); i++)
         {
            int waypointIndex = i + 1;
            waypoint = waypoints.size() == waypointIndex ? waypoints.add() : waypoints.get(waypointIndex);
            Footstep footstep = footsteps.get(i);
            tempFootPoses.get(footstep.getRobotSide()).set(footstep.getFootstepPose());
            double yaw = computeAverage(tempFootPoses, waypoint.position);
            waypoint.setYaw(previousWaypoint.getYaw() + AngleTools.computeAngleDifferenceMinusPiToPi(yaw, previousWaypoint.getYaw()));
            double waypointDuration = footstepTimings.get(i).getSwingTime() + 0.5 * footstepTimings.get(i).getTransferTime();
            if (i == 0)
               waypointDuration += 0.5 * footstepTimings.get(i).getTransferTime();
            if (i == footsteps.size() - 1)
               waypointDuration += finalTransferDuration; // add the final transfer duration.
            else
               waypointDuration += 0.5 * footstepTimings.get(i + 1).getTransferTime();
            waypoint.time.set(previousWaypoint.time.getValue() + waypointDuration);
            waypoint.updateViz();
            previousWaypoint = waypoint;
         }

         totalDuration.set(previousWaypoint.time.getValue());
      }
      else
      {
         initialLinearVelocity.setToZero();
         initialYawRate.set(0.0);
         totalDuration.set(0.0);
      }
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

      waypointsWTPViz.hideAll();
      for (int i = 0; i < waypoints.size(); i++)
      {
         waypointsWTPViz.setBall(waypoints.get(i).position, i);
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

   public MovingReferenceFrame getWalkingTrajectoryPathFrame()
   {
      return walkingTrajectoryPathFrame;
   }

   private static class WaypointData
   {
      private final String name;
      private final YoDouble time;
      private final YoFramePoint3D position;
      private final YoDouble yaw;

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

         if (yoGraphicsList != null)
         {
            zUpViz = new YoFrameVector3D(namePrefix + "ZUpViz" + nameSuffix, worldFrame, registry);
            headingViz = new YoFrameVector3D(namePrefix + "HeadingViz" + nameSuffix, worldFrame, registry);
            waypointZUpViz = new YoGraphicVector(nameSuffix + "WaypointZUpViz" + nameSuffix, position, zUpViz, 1.0, YoAppearance.Orange());
            waypointHeadingViz = new YoGraphicVector(namePrefix + "WaypointHeadingViz" + nameSuffix, position, headingViz, 1.0, YoAppearance.Black());
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

      public void setYaw(double yaw)
      {
         this.yaw.set(yaw);
      }

      @Override
      public String toString()
      {
         return name + ", time: " + time.getValue() + ", pos: " + EuclidCoreIOTools.getTuple3DString(position) + ", yaw: " + yaw.getValue();
      }
   }

   private static class TrajectoryManager
   {
      private final DMatrixRMaj[] linearSolutions = {new DMatrixRMaj(1, 1), new DMatrixRMaj(1, 1), new DMatrixRMaj(1, 1)};
      private final DMatrixRMaj yawSolution = new DMatrixRMaj(1, 1);
      private final YoMatrix someMatrix;
      private final MultiCubicSpline1DSolver[] linearSolvers = {new MultiCubicSpline1DSolver(), new MultiCubicSpline1DSolver(), new MultiCubicSpline1DSolver()};
      private final MultiCubicSpline1DSolver yawSolver = new MultiCubicSpline1DSolver();
      private double totalDuration;

      public TrajectoryManager(YoRegistry registry)
      {
         someMatrix = new YoMatrix("SolutionX", 8, 1, registry);
      }

      public void initialize(Vector3DReadOnly initialLinearVelocity,
                             double initialYawRate,
                             YoPreallocatedList<WaypointData> waypoints,
                             boolean isLastWaypointOpen)
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
                                      initialLinearVelocity.getElement(axis) * totalDuration,
                                      lastWaypoint.getPosition(axis),
                                      0.0);
            if (isLastWaypointOpen)
               linearSolver.setEndpointWeights(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0);

            for (int i = 1; i < waypoints.size() - 1; i++)
            {
               WaypointData waypoint = waypoints.get(i);
               linearSolver.addWaypoint(waypoint.getPosition(axis), waypoint.time.getValue() / totalDuration);
            }

            linearSolver.solve(linearSolutions[axis.ordinal()]);
         }
         someMatrix.set(linearSolutions[Axis3D.X.ordinal()]);

         yawSolver.clearWaypoints();
         yawSolver.clearWeights();
         yawSolver.setEndpoints(firstWaypoint.yaw.getValue(), initialYawRate * totalDuration, lastWaypoint.yaw.getValue(), 0.0);
         if (isLastWaypointOpen)
            yawSolver.setEndpointWeights(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0);

         for (int i = 1; i < waypoints.size() - 1; i++)
         {
            WaypointData waypoint = waypoints.get(i);
            yawSolver.addWaypoint(waypoint.yaw.getValue(), waypoint.time.getValue() / totalDuration);
         }

         yawSolver.solve(yawSolution);
      }

      public void computePosition(double time, Tuple3DBasics positionToPack)
      {
         positionToPack.setX(linearSolvers[0].computePosition(time / totalDuration, linearSolutions[0]));
         positionToPack.setY(linearSolvers[1].computePosition(time / totalDuration, linearSolutions[1]));
         positionToPack.setZ(linearSolvers[2].computePosition(time / totalDuration, linearSolutions[2]));
      }

      public double computeYaw(double time)
      {
         return yawSolver.computePosition(time / totalDuration, yawSolution);
      }

      public void computeLinearVelocity(double time, Tuple3DBasics velocityToPack)
      {
         velocityToPack.setX(linearSolvers[0].computeVelocity(time / totalDuration, linearSolutions[0]));
         velocityToPack.setY(linearSolvers[1].computeVelocity(time / totalDuration, linearSolutions[1]));
         velocityToPack.setZ(linearSolvers[2].computeVelocity(time / totalDuration, linearSolutions[2]));
         velocityToPack.scale(1.0 / totalDuration);
      }

      public double computeYawRate(double time)
      {
         return yawSolver.computeVelocity(time / totalDuration, yawSolution) / totalDuration;
      }
   }

   private static void updateFilteredWaypoint(WaypointData waypoint, Point3DReadOnly newPosition, double newYaw, double alpha)
   {
      waypoint.position.interpolate(newPosition, waypoint.position, alpha);
      waypoint.setYaw(AngleTools.interpolateAngle(newYaw, waypoint.getYaw(), alpha));
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

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      if (currentZUpViz == null)
         return null;

      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D("CurrentZUp", currentPosition, currentZUpViz, 1.0, ColorDefinitions.Blue()));
//      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D("CurrentHeading", currentPosition, currentHeadingViz, 0.35, ColorDefinitions.Blue()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPointcloud3D("walkingPath", trajectoryPositionViz.getPositions(), 0.02, ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPointcloud3D("waypointWTP", waypointsWTPViz.getPositions(), 0.04, ColorDefinitions.Black()));
      return group;
   }
}
