package us.ihmc.commonWalkingControlModules.capturePoint;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
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
import us.ihmc.robotics.math.trajectories.generators.MultiCubicSpline1DSolver;
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
   private final TrajectoryManager trajectoryManager = new TrajectoryManager();

   private final YoPreallocatedList<WaypointData> waypoints;

   private final RecyclingArrayList<Footstep> footsteps = new RecyclingArrayList<>(Footstep::new);
   private final RecyclingArrayList<FootstepTiming> footstepTimings = new RecyclingArrayList<>(FootstepTiming::new);

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

   private final BagOfBalls trajectoryPositionViz;
   private final YoFrameVector3D currentZUpViz;
   private final YoFrameVector3D currentHeadingViz;

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

   public void addFootsteps(WalkingMessageHandler walkingMessageHandler)
   {
      for (int i = 0; i < walkingMessageHandler.getCurrentNumberOfFootsteps(); i++)
      {
         walkingMessageHandler.peekFootstep(i, footsteps.add());
         walkingMessageHandler.peekTiming(i, footstepTimings.add());
      }
   }

   public void addFootstep(Footstep footstep, FootstepTiming footstepTiming)
   {
      footsteps.add().set(footstep);
      footstepTimings.add().set(footstepTiming);
   }

   private final SideDependentList<Pose3D> supportFootPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private final SideDependentList<Pose3D> tempFootPoses = new SideDependentList<>(new Pose3D(), new Pose3D());

   public void initialize()
   {
      startTime.set(time.getValue());

      for (RobotSide robotSide : RobotSide.values)
      {
         supportFootPoses.get(robotSide).set(soleFrames.get(robotSide).getTransformToRoot());
      }

      WaypointData waypoint = waypoints.add();
      waypoint.time.set(0.0);

      if (reset.getValue())
      {
         reset.set(false);
         waypoint.yaw.set(computeAverage(supportFootPoses, waypoint.position));
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

         for (RobotSide robotSide : RobotSide.values)
         {
            tempFootPoses.get(robotSide).set(supportFootPoses.get(robotSide));
         }

         for (int i = 0; i < footsteps.size(); i++)
         {
            waypoint = waypoints.add();
            Footstep footstep = footsteps.get(i);
            tempFootPoses.get(footstep.getRobotSide()).set(footstep.getFootstepPose());
            double yaw = computeAverage(tempFootPoses, waypoint.position);
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
      return computeAverage(input.get(RobotSide.LEFT), input.get(RobotSide.RIGHT), output);
   }

   private static double computeAverage(Pose3DReadOnly firstPose, Pose3DReadOnly secondPose, Point3DBasics output)
   {
      output.interpolate(firstPose.getPosition(), secondPose.getPosition(), 0.5);
      return AngleTools.computeAngleAverage(firstPose.getYaw(), secondPose.getYaw());
   }

   public void computeTrajectory(WalkingStateEnum currentState)
   {
      updateWaypoints(currentState);
      updatePolynomials();

      double currentTime = MathTools.clamp(time.getValue() - startTime.getValue(), 0.0, totalDuration.getValue());

      if (footsteps.isEmpty())
      {
         WaypointData firstWaypoint = waypoints.getFirst();
         currentPosition.set(firstWaypoint.position);
         currentLinearVelocity.set(firstWaypoint.linearVelocity);
         currentYaw.set(AngleTools.trimAngleMinusPiToPi(firstWaypoint.yaw.getValue()));
         currentYawRate.set(firstWaypoint.yawRate.getValue());
      }
      else
      {
         trajectoryManager.computePosition(currentTime, currentPosition);
         trajectoryManager.computeLinearVelocity(currentTime, currentLinearVelocity);
         currentYaw.set(AngleTools.trimAngleMinusPiToPi(trajectoryManager.computeYaw(currentTime)));
         currentYawRate.set(trajectoryManager.computeYawRate(currentTime));
      }

      lastPosition.set(currentPosition);
      lastYaw.set(currentYaw.getValue());
      lastLinearVelocity.set(currentLinearVelocity);
      lastYawRate.set(currentYawRate.getValue());

      walkingTrajectoryPathFrame.update();

      updateViz();
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

   private void updateWaypoints(WalkingStateEnum currentState)
   {
      if (currentState.isDoubleSupport())
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyTransform soleTransform = soleFrames.get(robotSide).getTransformToRoot();
            supportFootPoses.get(robotSide).set(soleTransform);
            tempFootPoses.get(robotSide).set(soleTransform);
         }
      }
      else
      {
         RobotSide supportSide = currentState.getSupportSide();
         RigidBodyTransform soleTransform = soleFrames.get(supportSide).getTransformToRoot();
         supportFootPoses.get(supportSide).set(soleTransform);

         for (RobotSide robotSide : RobotSide.values)
         {
            tempFootPoses.get(robotSide).set(supportFootPoses.get(robotSide));
         }

      }

      WaypointData firstWaypoint = waypoints.getFirst();
      firstWaypoint.yaw.set(computeAverage(supportFootPoses, firstWaypoint.position));
      
      if (!footsteps.isEmpty())
      {
         Footstep firstFootstep = footsteps.get(0);
         tempFootPoses.get(firstFootstep.getRobotSide()).set(firstFootstep.getFootstepPose());
         WaypointData secondWaypoint = waypoints.get(1);
         secondWaypoint.yaw.set(computeAverage(tempFootPoses, secondWaypoint.position));
      }
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
            waypoint.yawRate.set(trajectoryManager.computeYawRate(waypoint.time.getValue()));
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
         linearVelocity.setToNaN();
         yawRate.setToNaN();
      }

      public void updateViz()
      {
         if (headingViz != null)
         {
            zUpViz.set(Axis3D.Z);
            RotationMatrixTools.applyYawRotation(yaw.getValue(), Axis3D.X, headingViz);
         }
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

   private static class TrajectoryManager
   {
      private final DMatrixRMaj[] linearSolutions = {new DMatrixRMaj(1, 1), new DMatrixRMaj(1, 1), new DMatrixRMaj(1, 1)};
      private final DMatrixRMaj yawSolution = new DMatrixRMaj(1, 1);
      private final MultiCubicSpline1DSolver[] linearSolvers = {new MultiCubicSpline1DSolver(), new MultiCubicSpline1DSolver(), new MultiCubicSpline1DSolver()};
      private final MultiCubicSpline1DSolver yawSolver = new MultiCubicSpline1DSolver();
      private double totalDuration;

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
}
