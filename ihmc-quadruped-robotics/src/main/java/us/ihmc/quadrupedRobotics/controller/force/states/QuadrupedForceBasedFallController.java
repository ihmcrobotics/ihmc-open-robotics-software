package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedWaypointCallback;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedForceBasedFallController implements QuadrupedController, QuadrupedWaypointCallback
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public enum FallBehaviorType
   {
      FREEZE, GO_HOME_Z, GO_HOME_XYZ, DO_NOTHING
   }

   // Parameters
   private final DoubleParameter trajectoryTimeParameter = new DoubleParameter("trajectoryTime", registry, 3.0);
   private final DoubleParameter stanceLengthParameter = new DoubleParameter("stanceLength", registry, 1.0);
   private final DoubleParameter stanceWidthParameter = new DoubleParameter("stanceWidth", registry, 0.35);
   private final DoubleParameter stanceHeightParameter = new DoubleParameter("stanceHeight", registry, 0.5);
   private final DoubleParameter stanceXOffsetParameter = new DoubleParameter("stanceXOffset", registry, 0.0);
   private final DoubleParameter stanceYOffsetParameter = new DoubleParameter("stanceYOffset", registry, 0.0);
   private final DoubleParameter jointDampingParameter = new DoubleParameter("jointDamping", registry, 15.0);
   private final DoubleParameter jointPositionLimitDampingParameter = new DoubleParameter("jointPositionLimitDamping", registry, 10.0);
   private final DoubleParameter jointPositionLimitStiffnessParameter = new DoubleParameter("jointPositionLimitStiffness", registry, 100.0);
   private final BooleanParameter requestUseForceFeedbackControlParameter = new BooleanParameter("requestUseForceFeedbackControl", registry, false);

   // YoVariables
   private final YoBoolean forceFeedbackControlEnabled;
   private final YoEnum<FallBehaviorType> fallBehaviorType = YoEnum.create("fallBehaviorType", FallBehaviorType.class, registry);

   // Task space controller
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   private final QuadrantDependentList<FrameEuclideanTrajectoryPointList> soleWaypointLists = new QuadrantDependentList<>();
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedReferenceFrames referenceFrames;
   private final FramePoint3D solePositionSetpoint;
   private final Vector3D zeroVelocity;
   private final FullQuadrupedRobotModel fullRobotModel;

   private final YoBoolean isDoneMoving = new YoBoolean("fallIsDoneMoving", registry);
   private final QuadrupedForceControllerToolbox controllerToolbox;
   private final JointDesiredOutputList jointDesiredOutputList;

   public QuadrupedForceBasedFallController(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                            YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.jointDesiredOutputList = controllerToolbox.getRuntimeEnvironment().getJointDesiredOutputList();
      this.fallBehaviorType.set(FallBehaviorType.GO_HOME_XYZ);

      feetManager = controlManagerFactory.getOrCreateFeetManager();

      referenceFrames = controllerToolbox.getReferenceFrames();
      solePositionSetpoint = new FramePoint3D();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         soleWaypointLists.set(quadrant, new FrameEuclideanTrajectoryPointList());
      }
      zeroVelocity = new Vector3D(0, 0, 0);
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      this.taskSpaceController = controllerToolbox.getTaskSpaceController();
      fullRobotModel = controllerToolbox.getRuntimeEnvironment().getFullRobotModel();

      forceFeedbackControlEnabled = new YoBoolean("forceFeedbackControlEnabled", registry);

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
      // Create sole waypoint trajectories
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FrameEuclideanTrajectoryPointList soleWaypoints = soleWaypointLists.get(quadrant);
         soleWaypoints.clear();

         solePositionSetpoint.setIncludingFrame(controllerToolbox.getTaskSpaceEstimates().getSolePosition(quadrant));
         solePositionSetpoint.changeFrame(referenceFrames.getBodyFrame());
         soleWaypoints.addTrajectoryPoint(0.0, solePositionSetpoint, zeroVelocity);
         switch (fallBehaviorType.getEnumValue())
         {
         case GO_HOME_XYZ:
            solePositionSetpoint.set(quadrant.getEnd().negateIfHindEnd(stanceLengthParameter.getValue() / 2.0),
                  quadrant.getSide().negateIfRightSide(stanceWidthParameter.getValue() / 2.0), 0.0);
            solePositionSetpoint.add(stanceXOffsetParameter.getValue(), stanceYOffsetParameter.getValue(), -stanceHeightParameter.getValue());
            break;
         case GO_HOME_Z:
            solePositionSetpoint.setZ(-stanceHeightParameter.getValue());
            break;
         case DO_NOTHING:
            break;
         }
         soleWaypoints.addTrajectoryPoint(trajectoryTimeParameter.getValue(), solePositionSetpoint, zeroVelocity);

         feetManager.initializeWaypointTrajectory(quadrant, soleWaypoints, false);
      }

      // Initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.getValue());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.getValue());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.getValue());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.NO_CONTACT);
      }
      taskSpaceController.reset();

      // Initialize force feedback
      forceFeedbackControlEnabled.set(requestUseForceFeedbackControlParameter.getValue());
      for (OneDoFJoint oneDoFJoint : fullRobotModel.getOneDoFJoints())
      {
         QuadrupedJointName jointName = fullRobotModel.getNameForOneDoFJoint(oneDoFJoint);
         if (oneDoFJoint != null && jointName.getRole().equals(JointRole.LEG))
         {
            if (forceFeedbackControlEnabled.getValue())
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
      feetManager.getDesiredSoleForceCommand(taskSpaceControllerCommands.getSoleForce());
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
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
