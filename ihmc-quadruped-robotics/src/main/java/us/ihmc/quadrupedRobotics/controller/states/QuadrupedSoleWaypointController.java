package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedWaypointCallback;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachine.extra.EventState;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedSoleWaypointController implements EventState, QuadrupedWaypointCallback
{
   // Yo variables
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedStepMessageHandler stepMessageHandler;

   private final QuadrupedBodyOrientationManager bodyOrientationManager;
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;

   private final QuadrantDependentList<YoBoolean> isDoneMoving = new QuadrantDependentList<>();

   private final QuadrupedControllerToolbox controllerToolbox;

   private final QuadrantDependentList<QuadrupedTimedStep> spoofStepPool = new QuadrantDependentList<>();
   private final QuadrantDependentList<FramePoint3D> feetAtEntry = new QuadrantDependentList<>();
   private final List<QuadrupedTimedStep> spoofSteps = new ArrayList<>();


   private final DoubleParameter transferToSoleWaypointDuration = new DoubleParameter("transferToSoleWaypointDuration", registry, 1.0);
   private final YoBoolean doneWithInitialTransfer = new YoBoolean("doneWithInitialTransfer", registry);
   private final YoDouble currentTime;

   public QuadrupedSoleWaypointController(QuadrupedControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                          QuadrupedStepMessageHandler stepMessageHandler, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.stepMessageHandler = stepMessageHandler;

      currentTime = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      feetManager = controlManagerFactory.getOrCreateFeetManager();
      bodyOrientationManager = controlManagerFactory.getOrCreateBodyOrientationManager();
      balanceManager = controlManagerFactory.getOrCreateBalanceManager();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         isDoneMoving.put(robotQuadrant, new YoBoolean(robotQuadrant.getShortName() + "SoleWaypointDoneMoving", registry));
         spoofStepPool.put(robotQuadrant, new QuadrupedTimedStep());
         feetAtEntry.put(robotQuadrant, new FramePoint3D());
      }

      parentRegistry.addChild(registry);
   }


   @Override
   public void isDoneMoving(RobotQuadrant robotQuadrant, boolean doneMoving)
   {
      boolean done = doneMoving && !isDoneMoving.get(robotQuadrant).getBooleanValue();
      isDoneMoving.get(robotQuadrant).set(done);
   }

   @Override
   public void onEntry()
   {
      spoofSteps.clear();
      doneWithInitialTransfer.set(false);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D footPosition = feetAtEntry.get(robotQuadrant);
         footPosition.setToZero(controllerToolbox.getSoleReferenceFrame(robotQuadrant));
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());

         if (stepMessageHandler.hasFootTrajectoryForSolePositionControl(robotQuadrant))
         {
            isDoneMoving.get(robotQuadrant).set(false);
            QuadrupedTimedStep spoofStep = spoofStepPool.get(robotQuadrant);
            spoofStep.setRobotQuadrant(robotQuadrant);
            spoofStep.setGoalPosition(footPosition);
            spoofStep.getTimeInterval().setInterval(currentTime.getDoubleValue() + transferToSoleWaypointDuration.getValue(), Double.POSITIVE_INFINITY);

            spoofSteps.add(spoofStep);
         }
         else
         {
            isDoneMoving.get(robotQuadrant).set(true);
         }
      }

      balanceManager.clearStepSequence();
      balanceManager.addStepsToSequence(spoofSteps);
      balanceManager.initializeForStepping();

      feetManager.registerWaypointCallback(this);
   }

   @Override
   public void doAction(double timeInState)
   {
      if (timeInState > transferToSoleWaypointDuration.getValue() && !doneWithInitialTransfer.getBooleanValue())
         doneWithInitialTransfer.set(true);

      if (doneWithInitialTransfer.getBooleanValue())
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (stepMessageHandler.hasFootTrajectoryForSolePositionControl(robotQuadrant))
            {
               feetManager.initializeWaypointTrajectory(stepMessageHandler.pollFootTrajectoryForSolePositionControl(robotQuadrant));
               isDoneMoving.get(robotQuadrant).set(false);
            }
         }
      }

      controllerToolbox.updateSupportPolygon();

      balanceManager.compute();
      bodyOrientationManager.compute();
      feetManager.compute();
   }

   @Override
   public ControllerEvent fireEvent(double timeInState)
   {
      boolean allAreDone = true;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         allAreDone &= isDoneMoving.get(robotQuadrant).getBooleanValue();

      return allAreDone ? ControllerEvent.DONE : null;
   }

   @Override
   public void onExit()
   {
      feetManager.registerWaypointCallback(null);
   }
}
