package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedWaypointCallback;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class QuadrupedSoleWaypointController implements QuadrupedController, QuadrupedWaypointCallback
{
   // Yo variables
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoBoolean forceFeedbackControlEnabled;
   // Parameters
   private final BooleanParameter requestUseForceFeedbackControlParameter = new BooleanParameter("requestUseForceFeedbackControl", registry, false);
   private final BooleanParameter useInitialSoleForces =  new BooleanParameter("useInitialSoleForces", registry, true);

   private final QuadrupedStepMessageHandler stepMessageHandler;
   private final QuadrupedFeetManager feetManager;

   private final FullQuadrupedRobotModel fullRobotModel;

   private final YoBoolean isDoneMoving = new YoBoolean("soleWaypointDoneMoving", registry);
   private final QuadrupedControllerToolbox controllerToolbox;
   private final JointDesiredOutputList jointDesiredOutputList;

   public QuadrupedSoleWaypointController(QuadrupedControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                          QuadrupedStepMessageHandler stepMessageHandler, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.jointDesiredOutputList = controllerToolbox.getRuntimeEnvironment().getJointDesiredOutputList();
      this.stepMessageHandler = stepMessageHandler;
      feetManager = controlManagerFactory.getOrCreateFeetManager();
      forceFeedbackControlEnabled = new YoBoolean("forceFeedbackControlEnabled", registry);
      fullRobotModel = controllerToolbox.getRuntimeEnvironment().getFullRobotModel();

      parentRegistry.addChild(registry);
   }

   @Override
   public void isDoneMoving(boolean doneMoving)
   {
      boolean done = doneMoving && isDoneMoving.getBooleanValue();
      isDoneMoving.set(done);
   }

   @Override
   public void onEntry()
   {
      controllerToolbox.update();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (stepMessageHandler.hasFootTrajectoryForSolePositionControl(robotQuadrant))
            feetManager.initializeWaypointTrajectory(stepMessageHandler.pollFootTrajectoryForSolePositionControl(robotQuadrant), useInitialSoleForces.getValue());
      }

      forceFeedbackControlEnabled.set(requestUseForceFeedbackControlParameter.getValue());
      // Initialize force feedback
      for (OneDoFJoint oneDoFJoint : fullRobotModel.getOneDoFJoints())
      {
         QuadrupedJointName jointName = fullRobotModel.getNameForOneDoFJoint(oneDoFJoint);
         if (oneDoFJoint != null && jointName.getRole().equals(JointRole.LEG))
         {
            if (forceFeedbackControlEnabled.getBooleanValue())
               jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint).setControlMode(JointDesiredControlMode.EFFORT);
            else
               jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint).setControlMode(JointDesiredControlMode.POSITION);
         }
      }

      feetManager.registerWaypointCallback(this);
   }

   @Override
   public void doAction(double timeInState)
   {
      controllerToolbox.update();
      feetManager.updateSupportPolygon();
      feetManager.compute();
   }

   @Override
   public ControllerEvent fireEvent(double timeInState)
   {
      return isDoneMoving.getBooleanValue() ? ControllerEvent.DONE : null;
   }

   @Override
   public void onExit()
   {
      forceFeedbackControlEnabled.set(true);
      for (OneDoFJoint oneDoFJoint : fullRobotModel.getOneDoFJoints())
      {
         QuadrupedJointName jointName = fullRobotModel.getNameForOneDoFJoint(oneDoFJoint);
         if (oneDoFJoint != null && jointName.getRole().equals(JointRole.LEG))
         {
            if (forceFeedbackControlEnabled.getBooleanValue())
               jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint).setControlMode(JointDesiredControlMode.EFFORT);
            else
               jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint).setControlMode(JointDesiredControlMode.POSITION);
         }
      }

      feetManager.registerWaypointCallback(null);
   }
}
