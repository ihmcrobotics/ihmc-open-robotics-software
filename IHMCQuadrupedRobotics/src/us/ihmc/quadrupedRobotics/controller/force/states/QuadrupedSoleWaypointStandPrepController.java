package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSoleWaypointController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.params.DoubleArrayParameter;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.quadrupedRobotics.planning.SoleWaypoint;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFMinimumJerkTrajectory;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import javax.vecmath.Vector3d;

public class QuadrupedSoleWaypointStandPrepController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // Parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter trajectoryTimeParameter = parameterFactory.createDouble("trajectoryTime", 1.0);
   private final DoubleParameter stanceLengthParameter = parameterFactory.createDouble("stanceLength", 1.0);
   private final DoubleParameter stanceWidthParameter = parameterFactory.createDouble("stanceWidth", 0.5);
   private final DoubleParameter stanceHeightParameter = parameterFactory.createDouble("stanceHeight", 0.60);
   private final DoubleParameter stanceXOffsetParameter = parameterFactory.createDouble("stanceXOffset", 0.05);
   private final DoubleParameter stanceYOffsetParameter = parameterFactory.createDouble("stanceYOffset", 0.0);

   //Yo Variables
   private final DoubleYoVariable robotTime;

   private QuadrupedSoleWaypointList quadrupedSoleWaypointList;
   private final QuadrupedSoleWaypointController quadrupedSoleWaypointController;
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedReferenceFrames referenceFrames;
   private QuadrantDependentList<SoleWaypoint> quadrupedInitialSoleWaypoint;
   private QuadrantDependentList<SoleWaypoint> quadrupedFinalSoleWaypoint;
   private FramePoint solePosition;
   private final Vector3d zeroVelocity;
   private double taskStartTime;

   public QuadrupedSoleWaypointStandPrepController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox)
   {
      robotTime = environment.getRobotTimestamp();
      referenceFrames = controllerToolbox.getReferenceFrames();

      // Task space controller
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      quadrupedSoleWaypointController = controllerToolbox.getQuadrupedSoleWaypointController();
      solePosition = new FramePoint();
      quadrupedInitialSoleWaypoint = new QuadrantDependentList<>();
      quadrupedFinalSoleWaypoint = new QuadrantDependentList<>();
      quadrupedSoleWaypointList = new QuadrupedSoleWaypointList();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         quadrupedInitialSoleWaypoint.set(quadrant, new SoleWaypoint());
         quadrupedFinalSoleWaypoint.set(quadrant, new SoleWaypoint());
      }
      zeroVelocity = new Vector3d(0, 0, 0);
      environment.getParentRegistry().addChild(registry);
   }

   @Override
   public void onEntry()
   {
      taskStartTime = robotTime.getDoubleValue();
      updateEstimates();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         quadrupedSoleWaypointList.get(quadrant).clear();
         solePosition = taskSpaceEstimates.getSolePosition(quadrant);
         solePosition.changeFrame(referenceFrames.getBodyFrame());
         quadrupedInitialSoleWaypoint.get(quadrant).set(solePosition.getPoint(), zeroVelocity, 0.0);
         quadrupedSoleWaypointList.get(quadrant).add(quadrupedInitialSoleWaypoint.get(quadrant));
         solePosition.setToZero(referenceFrames.getBodyFrame());
         solePosition.add(quadrant.getEnd().negateIfHindEnd(stanceLengthParameter.get() / 2.0), 0.0, 0.0);
         solePosition.add(0.0, quadrant.getSide().negateIfRightSide(stanceWidthParameter.get() / 2.0), 0.0);
         solePosition.add(stanceXOffsetParameter.get(), stanceYOffsetParameter.get(), -stanceHeightParameter.get());
         quadrupedFinalSoleWaypoint.get(quadrant).set(solePosition.getPoint(), zeroVelocity, trajectoryTimeParameter.get());
         quadrupedSoleWaypointList.get(quadrant).add(quadrupedFinalSoleWaypoint.get(quadrant));
      }
      quadrupedSoleWaypointController.initialize(quadrupedSoleWaypointList);
   }

   @Override
   public ControllerEvent process()
   {
      quadrupedSoleWaypointController.compute();
      return isMotionExpired() ? ControllerEvent.DONE : null;
   }

   @Override
   public void onExit()
   {
   }

   private void updateEstimates()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
   }

   private boolean isMotionExpired()
   {
      double currentTime = robotTime.getDoubleValue();
      return currentTime  - taskStartTime > trajectoryTimeParameter.get();
   }
}
