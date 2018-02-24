package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionControllerSetpoints;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedMoveViaWaypointsState extends QuadrupedFootState
{
   // Yo variables
   private final YoVariableRegistry registry;
   private final YoDouble robotTime;

   // SoleWaypoint variables
   private final MultipleWaypointsPositionTrajectoryGenerator quadrupedWaypointsPositionTrajectoryGenerator;

   // Feedback controller
   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionControllerSetpoints solePositionControllerSetpoints;
   private final FrameVector3D initialSoleForces = new FrameVector3D();

   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame soleFrame;
   private final QuadrupedSoleWaypointList quadrupedSoleWaypointList = new QuadrupedSoleWaypointList();

   private final QuadrupedFootControlModuleParameters parameters;
   private final RobotQuadrant robotQuadrant;

   private final QuadrupedForceControllerToolbox controllerToolbox;

   private double taskStartTime;

   public QuadrupedMoveViaWaypointsState(RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox toolbox,
                                         QuadrupedSolePositionController solePositionController, YoVariableRegistry parentRegistry)
   {
      this.robotQuadrant = robotQuadrant;
      this.controllerToolbox = toolbox;
      this.bodyFrame = toolbox.getReferenceFrames().getBodyFrame();
      this.soleFrame = toolbox.getSoleReferenceFrame(robotQuadrant);
      this.parameters = toolbox.getFootControlModuleParameters();
      robotTime = toolbox.getRuntimeEnvironment().getRobotTimestamp();

      // Feedback controller
      this.solePositionController = solePositionController;
      solePositionControllerSetpoints = new QuadrupedSolePositionControllerSetpoints(robotQuadrant);

      registry = new YoVariableRegistry(robotQuadrant.getShortName() + getClass().getSimpleName());

      // Create waypoint trajectory
      quadrupedWaypointsPositionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator(robotQuadrant.getCamelCaseName() + "SoleTrajectory",
                                                                                                       bodyFrame, registry);
      parentRegistry.addChild(registry);
   }

   public void handleWaypointList(QuadrupedSoleWaypointList quadrupedSoleWaypointList)
   {
      this.quadrupedSoleWaypointList.set(quadrupedSoleWaypointList);
   }

   public void initialize(boolean useInitialSoleForceAsFeedforwardTerm)
   {
      solePositionControllerSetpoints.initialize(soleFrame);
      solePositionController.reset();

      if (useInitialSoleForceAsFeedforwardTerm)
      {
         this.initialSoleForces.setIncludingFrame(controllerToolbox.getTaskSpaceEstimates().getSoleVirtualForce(robotQuadrant));
         this.initialSoleForces.changeFrame(bodyFrame);
      }
      else
      {
         this.initialSoleForces.setToZero(bodyFrame);
      }

      createSoleWaypointTrajectory();
      taskStartTime = robotTime.getDoubleValue();
   }

   @Override
   public void onEntry()
   {
      solePositionController.reset();
      solePositionController.getGains().setProportionalGains(parameters.getSolePositionProportionalGainsParameter());
      solePositionController.getGains().setDerivativeGains(parameters.getSolePositionDerivativeGainsParameter());
      solePositionController.getGains().setIntegralGains(parameters.getSolePositionIntegralGainsParameter(), parameters.getSolePositionMaxIntegralErrorParameter());
      solePositionControllerSetpoints.initialize(soleFrame);
   }

   @Override
   public QuadrupedFootControlModule.FootEvent process()
   {
      double currentTrajectoryTime = robotTime.getDoubleValue() - taskStartTime;

      if (currentTrajectoryTime > quadrupedSoleWaypointList.getFinalTime())
      {
         soleForceCommand.setToZero();

         if (waypointCallback != null)
            waypointCallback.isDoneMoving(true);

         return QuadrupedFootControlModule.FootEvent.TIMEOUT;
      }
      else
      {
         quadrupedWaypointsPositionTrajectoryGenerator.compute(currentTrajectoryTime);
         quadrupedWaypointsPositionTrajectoryGenerator.getPosition(solePositionControllerSetpoints.getSolePosition());
         solePositionControllerSetpoints.getSoleLinearVelocity().setToZero();
         solePositionControllerSetpoints.getSoleForceFeedforward().setIncludingFrame(initialSoleForces);
         solePositionController.compute(soleForceCommand, solePositionControllerSetpoints, controllerToolbox.getTaskSpaceEstimates().getSoleLinearVelocity(robotQuadrant));

         if (waypointCallback != null)
            waypointCallback.isDoneMoving(false);

         return null;
      }
   }

   @Override
   public void onExit()
   {
      soleForceCommand.setToZero();
   }

   private void createSoleWaypointTrajectory()
   {
      quadrupedWaypointsPositionTrajectoryGenerator.clear();
      for (int i = 0; i < quadrupedSoleWaypointList.size(); ++i)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.appendWaypoint(quadrupedSoleWaypointList.get(i).getTime(),
                                                                      quadrupedSoleWaypointList.get(i).getPosition(),
                                                                      quadrupedSoleWaypointList.get(i).getVelocity());
      }
      if (quadrupedSoleWaypointList.size() > 0)
      {
         quadrupedWaypointsPositionTrajectoryGenerator.initialize();
      }
   }
}
