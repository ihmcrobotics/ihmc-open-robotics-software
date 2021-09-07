package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.robotics.trajectories.providers.CurrentRigidBodyStateProvider;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class JumpingSwingTrajectoryCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final MovingReferenceFrame centerOfMassFrame;
   private final TwoWaypointSwingGenerator swingTrajectoryOptimizer;
   private final MultipleWaypointsPoseTrajectoryGenerator swingTrajectory;

   private final CurrentRigidBodyStateProvider currentStateProvider;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialLinearVelocity = new FrameVector3D();
   private final FrameQuaternion initialOrientation = new FrameQuaternion();
   private final FrameVector3D initialAngularVelocity = new FrameVector3D();
   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalLinearVelocity = new FrameVector3D();
   private final FrameQuaternion finalOrientation = new FrameQuaternion();
   private final FrameVector3D finalAngularVelocity = new FrameVector3D();

   private final double[] waypointProportions = new double[2];
   private final List<DoubleProvider> defaultWaypointProportions = new ArrayList<>();

   private final YoDouble swingDuration;
   private final YoDouble swingHeight;

   private final FramePose3D footstepPose = new FramePose3D();

   private final FrameEuclideanTrajectoryPoint tempPositionTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

   public JumpingSwingTrajectoryCalculator(RobotSide robotSide,
                                           JumpingControllerToolbox controllerToolbox,
                                           WalkingControllerParameters walkingControllerParameters,
                                           YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(robotSide.getLowerCaseName() + getClass().getSimpleName());
      this.centerOfMassFrame = controllerToolbox.getCenterOfMassFrame();

      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "FootSwing";

      double maxSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getMaxSwingHeightFromStanceFoot();
      double minSwingHeightFromStanceFoot = 0.0;
      double defaultSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getDefaultSwingHeightFromStanceFoot();
      double customWaypointAngleThreshold = walkingControllerParameters.getSteppingParameters().getCustomWaypointAngleThreshold();

      SwingTrajectoryParameters swingTrajectoryParameters = walkingControllerParameters.getSwingTrajectoryParameters();
      int numberWaypoints = 2;
      double[] defaultWaypointProportions = swingTrajectoryParameters.getSwingWaypointProportions();

      for (int i = 0; i < numberWaypoints; i++)
      {
         DoubleParameter waypointProportion = new DoubleParameter(namePrefix + "WaypointProportion" + i, registry, defaultWaypointProportions[i]);
         this.defaultWaypointProportions.add(waypointProportion);
      }

      currentStateProvider = new CurrentRigidBodyStateProvider(controllerToolbox.getReferenceFrames().getSoleFrame(robotSide));

      swingTrajectoryOptimizer = new TwoWaypointSwingGenerator(namePrefix,
                                                               minSwingHeightFromStanceFoot,
                                                               maxSwingHeightFromStanceFoot,
                                                               defaultSwingHeightFromStanceFoot,
                                                               customWaypointAngleThreshold,
                                                               centerOfMassFrame,
                                                               registry,
                                                               controllerToolbox.getYoGraphicsListRegistry());

      swingDuration = new YoDouble(namePrefix + "Duration", registry);
      swingHeight = new YoDouble(namePrefix + "Height", registry);

      swingTrajectory = new MultipleWaypointsPoseTrajectoryGenerator(namePrefix, Footstep.maxNumberOfSwingWaypoints + 2, registry);

      parentRegistry.addChild(registry);
   }

   public void informDone()
   {
      swingTrajectoryOptimizer.informDone();
   }

   public void setFootstep(FramePose3DReadOnly footstepPoseRelativeToTouchdownCoM, double swingHeight, double swingTime)
   {
      this.footstepPose.setIncludingFrame(centerOfMassFrame, footstepPoseRelativeToTouchdownCoM);
      this.swingHeight.set(swingHeight);

      List<DoubleProvider> waypointProportions = defaultWaypointProportions;
      this.waypointProportions[0] = waypointProportions.get(0).getValue();
      this.waypointProportions[1] = waypointProportions.get(1).getValue();

      swingDuration.set(swingTime);

      finalPosition.setIncludingFrame(footstepPose.getPosition());
      finalOrientation.setIncludingFrame(footstepPose.getOrientation());

      finalLinearVelocity.setToZero(centerOfMassFrame);
      finalAngularVelocity.setToZero(worldFrame);
      finalAngularVelocity.changeFrame(centerOfMassFrame);
   }

   public double getSwingDuration()
   {
      return swingDuration.getDoubleValue();
   }

   public boolean doOptimizationUpdate()
   {
      return swingTrajectoryOptimizer.doOptimizationUpdate();
   }

   public void setTrajectoryFromOptimizer(boolean initializeOptimizer)
   {
      // append current pose as initial trajectory point
      swingTrajectory.clear(centerOfMassFrame);

      swingTrajectory.appendPositionWaypoint(0.0, initialPosition, initialLinearVelocity);
      swingTrajectory.appendOrientationWaypoint(0.0, initialOrientation, initialAngularVelocity);

      // TODO: initialize optimizer somewhere else
      if (initializeOptimizer)
         initializeTrajectoryOptimizer();

      for (int i = 0; i < swingTrajectoryOptimizer.getNumberOfWaypoints(); i++)
      {
         swingTrajectoryOptimizer.getWaypointData(i, tempPositionTrajectoryPoint);
         swingTrajectory.appendPositionWaypoint(tempPositionTrajectoryPoint);
      }

      // append footstep pose if not provided in the waypoints
      swingTrajectory.appendPositionWaypoint(swingDuration.getDoubleValue(), finalPosition, finalLinearVelocity);
      swingTrajectory.appendOrientationWaypoint(swingDuration.getDoubleValue(), finalOrientation, finalAngularVelocity);
   }

   public void initializeTrajectoryWaypoints(boolean initializeOptimizer)
   {
      swingTrajectory.clear(centerOfMassFrame);
      setTrajectoryFromOptimizer(initializeOptimizer);
      swingTrajectory.initialize();
   }

   public void setShouldVisualize(boolean visualize)
   {
      swingTrajectoryOptimizer.setShouldVisualize(visualize);
   }

   private void initializeTrajectoryOptimizer()
   {
      swingTrajectoryOptimizer.setInitialConditions(initialPosition, initialLinearVelocity);
      swingTrajectoryOptimizer.setFinalConditions(finalPosition, finalLinearVelocity);
      swingTrajectoryOptimizer.setStepTime(swingDuration.getValue());
      swingTrajectoryOptimizer.setTrajectoryType(TrajectoryType.DEFAULT, null);
      swingTrajectoryOptimizer.setSwingHeight(swingHeight.getDoubleValue());
      swingTrajectoryOptimizer.setWaypointProportions(waypointProportions);
      swingTrajectoryOptimizer.initialize();
   }

   public MultipleWaypointsPoseTrajectoryGenerator getSwingTrajectory()
   {
      return swingTrajectory;
   }

   public void setInitialConditionsToCurrent()
   {
      currentStateProvider.getPosition(initialPosition);
//      currentStateProvider.getLinearVelocity(initialLinearVelocity);
      currentStateProvider.getOrientation(initialOrientation);
//      currentStateProvider.getAngularVelocity(initialAngularVelocity);
      initialPosition.changeFrame(centerOfMassFrame);
      initialOrientation.changeFrame(centerOfMassFrame);
      initialAngularVelocity.setToZero(centerOfMassFrame);
      initialLinearVelocity.setToZero(centerOfMassFrame);
   }
}
