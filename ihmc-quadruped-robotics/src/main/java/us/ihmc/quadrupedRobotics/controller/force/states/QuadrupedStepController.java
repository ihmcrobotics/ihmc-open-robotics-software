package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedJointSpaceManager;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedStep;
import us.ihmc.quadrupedRobotics.planning.stepStream.QuadrupedStepStream;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class QuadrupedStepController implements QuadrupedController, QuadrupedStepTransitionCallback
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final QuadrupedStepStream stepStream;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ParameterVector3D comForceCommandWeightsParameter;
   private final ParameterVector3D comTorqueCommandWeightsParameter;

   private final DoubleParameter jointDampingParameter = new DoubleParameter("jointDamping", registry, 1.0);
   private final DoubleParameter jointPositionLimitDampingParameter = new DoubleParameter("jointPositionLimitDampingParameter", registry, 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = new DoubleParameter("jointPositionLimitStiffnessParameter", registry, 100);
   private final DoubleParameter coefficientOfFrictionParameter = new DoubleParameter("coefficientOfFrictionParameter", registry, 0.5);

   private final QuadrupedStepMessageHandler stepMessageHandler;

   // managers
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedBodyOrientationManager bodyOrientationManager;
   private final QuadrupedJointSpaceManager jointSpaceManager;

   // task space controller
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;

   // step planner
   private final GroundPlaneEstimator groundPlaneEstimator;
   private final QuadrantDependentList<YoFramePoint> groundPlanePositions;

   // inputs
   private final YoBoolean onLiftOffTriggered = new YoBoolean("onLiftOffTriggered", registry);
   private final YoBoolean onTouchDownTriggered = new YoBoolean("onTouchDownTriggered", registry);

   private final QuadrupedForceControllerToolbox controllerToolbox;

   public QuadrupedStepController(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                  QuadrupedStepStream stepStream, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.stepStream = stepStream;
      stepMessageHandler = new QuadrupedStepMessageHandler(stepStream, controllerToolbox.getRuntimeEnvironment().getRobotTimestamp(), registry);

      // feedback controllers
      feetManager = controlManagerFactory.getOrCreateFeetManager();
      balanceManager = controlManagerFactory.getOrCreateBalanceManager();
      bodyOrientationManager = controlManagerFactory.getOrCreateBodyOrientationManager();
      jointSpaceManager = controlManagerFactory.getOrCreateJointSpaceManager();

      // task space controllers
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();

      // step planner
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      groundPlanePositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.set(robotQuadrant, new YoFramePoint(robotQuadrant.getCamelCaseName() + "GroundPlanePosition", worldFrame, registry));
      }

      Vector3D defaultComForceCommandWeights = new Vector3D(1.0, 1.0, 1.0);
      Vector3D defaultComTorqueCommandWeight = new Vector3D(1.0, 1.0, 1.0);

      comForceCommandWeightsParameter = new ParameterVector3D("comForceCommandWeight", defaultComForceCommandWeights, registry);
      comTorqueCommandWeightsParameter = new ParameterVector3D("comTorqueCommandWeight", defaultComTorqueCommandWeight, registry);

      parentRegistry.addChild(registry);
   }

   private void updateGains()
   {
      taskSpaceControllerSettings.getContactForceLimits().setCoefficientOfFriction(coefficientOfFrictionParameter.getValue());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.getValue());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.getValue());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.getValue());
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComForceCommandWeights(comForceCommandWeightsParameter);
      taskSpaceControllerSettings.getContactForceOptimizationSettings().setComTorqueCommandWeights(comTorqueCommandWeightsParameter);
   }

   @Override
   public void onLiftOff(RobotQuadrant thisStepQuadrant)
   {
      // update ground plane estimate
      groundPlanePositions.get(thisStepQuadrant).setAndMatchFrame(controllerToolbox.getTaskSpaceEstimates().getSolePosition(thisStepQuadrant));
      onLiftOffTriggered.set(true);
   }

   @Override
   public void onTouchDown(RobotQuadrant thisStepQuadrant)
   {
      onTouchDownTriggered.set(true);
   }

   @Override
   public void onEntry()
   {
      // initialize step stream
      stepStream.onEntry();

      // initialize state
      stepMessageHandler.initialize();
      onLiftOffTriggered.set(false);
      onTouchDownTriggered.set(false);

      // update task space estimates
      controllerToolbox.update();
      QuadrupedTaskSpaceEstimates taskSpaceEstimates = controllerToolbox.getTaskSpaceEstimates();

      // initialize feedback controllers
      //balanceManager.initialize(taskSpaceEstimates.getComPosition());

      bodyOrientationManager.initialize(taskSpaceEstimates.getBodyOrientation());

      feetManager.registerStepTransitionCallback(this);
      feetManager.reset();
      feetManager.requestFullContact();

      // initialize task space controller
      taskSpaceControllerSettings.initialize();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.getContactForceOptimizationSettings().setContactForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.IN_CONTACT);
      }

      // initialize ground plane
      groundPlaneEstimator.clearContactPoints();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlanePositions.get(robotQuadrant).setAndMatchFrame(taskSpaceEstimates.getSolePosition(robotQuadrant));
         groundPlaneEstimator.addContactPoint(groundPlanePositions.get(robotQuadrant));
      }
      groundPlaneEstimator.compute();

      // compute step plan
      stepMessageHandler.consumeIncomingSteps();
      stepMessageHandler.adjustStepQueue(balanceManager.getAccumulatedStepAdjustment());

      balanceManager.clearStepSequence();
      balanceManager.addStepsToSequence(stepMessageHandler.getStepSequence());

      // compute step adjustment
      RecyclingArrayList<QuadrupedStep> adjustedSteps = balanceManager.computeStepAdjustment(stepMessageHandler.getActiveSteps());
      feetManager.adjustSteps(adjustedSteps);

      balanceManager.initializeForStepping(taskSpaceControllerSettings);
   }

   @Override
   public ControllerEvent process()
   {
      if (stepMessageHandler.isDoneWithStepSequence())
      {
         return ControllerEvent.DONE;
      }
      stepStream.process();

      updateGains();

      // update task space estimates
      controllerToolbox.update();
      feetManager.updateSupportPolygon();

      // trigger step events
      feetManager.triggerSteps(stepMessageHandler.getActiveSteps());

      // update ground plane estimate
      groundPlaneEstimator.clearContactPoints();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         groundPlaneEstimator.addContactPoint(groundPlanePositions.get(robotQuadrant));
      }
      groundPlaneEstimator.compute();

      // update step plan
      stepMessageHandler.consumeIncomingSteps();
      stepMessageHandler.adjustStepQueue(balanceManager.getAccumulatedStepAdjustment());

      balanceManager.clearStepSequence();
      balanceManager.addStepsToSequence(stepMessageHandler.getStepSequence());

      // update step adjustment
      RecyclingArrayList<QuadrupedStep> adjustedSteps = balanceManager.computeStepAdjustment(stepMessageHandler.getActiveSteps());
      feetManager.adjustSteps(adjustedSteps);

      // update desired horizontal com forces
      balanceManager.compute(taskSpaceControllerSettings);

      // update desired body orientation, angular velocity, and torque
      bodyOrientationManager.compute(stepStream.getBodyOrientation());

      // update desired contact state and sole forces
      feetManager.compute();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(robotQuadrant, feetManager.getContactState(robotQuadrant));
      }

      jointSpaceManager.compute();

      // update accumulated step adjustment
      if (onLiftOffTriggered.getBooleanValue())
      {
         onLiftOffTriggered.set(false);
      }
      if (onTouchDownTriggered.getBooleanValue())
      {
         onTouchDownTriggered.set(false);
         balanceManager.completedStep();
      }
      return null;
   }

   @Override
   public void onExit()
   {
      // clean up step stream
      stepStream.onExit();

      feetManager.registerStepTransitionCallback(null);
   }

   public void halt()
   {
      stepMessageHandler.halt();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
