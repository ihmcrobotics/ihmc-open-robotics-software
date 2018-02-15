package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.*;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.quadrupedRobotics.planning.SoleWaypoint;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.dataStructures.parameter.BooleanParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
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
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter trajectoryTimeParameter = parameterFactory.createDouble("trajectoryTime", 3.0);
   private final DoubleParameter stanceLengthParameter = parameterFactory.createDouble("stanceLength", 1.0);
   private final DoubleParameter stanceWidthParameter = parameterFactory.createDouble("stanceWidth", 0.35);
   private final DoubleParameter stanceHeightParameter = parameterFactory.createDouble("stanceHeight", 0.5);
   private final DoubleParameter stanceXOffsetParameter = parameterFactory.createDouble("stanceXOffset", 0.00);
   private final DoubleParameter stanceYOffsetParameter = parameterFactory.createDouble("stanceYOffset", 0.0);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);
   private final DoubleParameter jointPositionLimitDampingParameter = parameterFactory.createDouble("jointPositionLimitDamping", 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = parameterFactory.createDouble("jointPositionLimitStiffness", 100);
   private final BooleanParameter useForceFeedbackControlParameter = parameterFactory.createBoolean("useForceFeedbackControl", false);

   // YoVariables
   private final YoBoolean yoUseForceFeedbackControl;
   private final YoEnum<FallBehaviorType> fallBehaviorType = YoEnum.create("fallBehaviorType", FallBehaviorType.class, registry);

   // Task space controller
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   private final QuadrantDependentList<QuadrupedSoleWaypointList> quadrupedSoleWaypointLists = new QuadrantDependentList<>();
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedTaskSpaceEstimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedReferenceFrames referenceFrames;
   private final FramePoint3D solePositionSetpoint;
   private final Vector3D zeroVelocity;
   private final FullQuadrupedRobotModel fullRobotModel;

   private final YoBoolean isDoneMoving = new YoBoolean("fallIsDoneMoving", registry);

   public QuadrupedForceBasedFallController(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                            YoVariableRegistry parentRegistry)
   {
      this.fallBehaviorType.set(FallBehaviorType.GO_HOME_XYZ);
      feetManager = controlManagerFactory.getOrCreateFeetManager();
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      referenceFrames = controllerToolbox.getReferenceFrames();
      solePositionSetpoint = new FramePoint3D();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         QuadrupedSoleWaypointList quadrupedSoleWaypointList = new QuadrupedSoleWaypointList();
         quadrupedSoleWaypointList.add(new SoleWaypoint());
         quadrupedSoleWaypointList.add(new SoleWaypoint());
         quadrupedSoleWaypointLists.set(quadrant, quadrupedSoleWaypointList);
      }
      zeroVelocity = new Vector3D(0, 0, 0);
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      this.taskSpaceController = controllerToolbox.getTaskSpaceController();
      fullRobotModel = controllerToolbox.getRuntimeEnvironment().getFullRobotModel();

      yoUseForceFeedbackControl = new YoBoolean("useForceFeedbackControl", registry);

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
      taskSpaceEstimator.compute(taskSpaceEstimates);
      // Create sole waypoint trajectories
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionSetpoint.setIncludingFrame(taskSpaceEstimates.getSolePosition(quadrant));
         solePositionSetpoint.changeFrame(referenceFrames.getBodyFrame());
         quadrupedSoleWaypointLists.get(quadrant).get(0).set(solePositionSetpoint, zeroVelocity, 0.0);
         switch (fallBehaviorType.getEnumValue())
         {
         case GO_HOME_XYZ:
            solePositionSetpoint.set(quadrant.getEnd().negateIfHindEnd(stanceLengthParameter.get() / 2.0),
                  quadrant.getSide().negateIfRightSide(stanceWidthParameter.get() / 2.0), 0.0);
            solePositionSetpoint.add(stanceXOffsetParameter.get(), stanceYOffsetParameter.get(), -stanceHeightParameter.get());
            break;
         case GO_HOME_Z:
            solePositionSetpoint.setZ(-stanceHeightParameter.get());
            break;
         case DO_NOTHING:
            break;
         }
         quadrupedSoleWaypointLists.get(quadrant).get(1).set(solePositionSetpoint, zeroVelocity, trajectoryTimeParameter.get());
      }

      feetManager.initializeWaypointTrajectory(quadrupedSoleWaypointLists, taskSpaceEstimates, false);

      // Initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(robotQuadrant, ContactState.NO_CONTACT);
      }
      taskSpaceController.reset();

      // Initialize force feedback
      yoUseForceFeedbackControl.set(useForceFeedbackControlParameter.get());
      for (QuadrupedJointName jointName : QuadrupedJointName.values())
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointName);
         if (oneDoFJoint != null && jointName.getRole().equals(JointRole.LEG))
         {
            oneDoFJoint.setUseFeedBackForceControl(useForceFeedbackControlParameter.get());
         }
      }

      feetManager.registerWaypointCallback(this);
   }


   @Override
   public ControllerEvent process()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
      feetManager.compute(taskSpaceControllerCommands.getSoleForce(), taskSpaceEstimates);
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
      return isDoneMoving.getBooleanValue() ? ControllerEvent.DONE : null;
   }

   @Override
   public void onExit()
   {
      yoUseForceFeedbackControl.set(true);
      for (QuadrupedJointName jointName : QuadrupedJointName.values())
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointName);
         if (oneDoFJoint != null && jointName.getRole().equals(JointRole.LEG))
         {
            oneDoFJoint.setUseFeedBackForceControl(yoUseForceFeedbackControl.getBooleanValue());
         }
      }

      feetManager.registerWaypointCallback(null);
   }
}
