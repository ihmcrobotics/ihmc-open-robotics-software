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
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedSoleWaypointList;
import us.ihmc.quadrupedRobotics.planning.SoleWaypoint;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.robotics.dataStructures.parameter.BooleanParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class QuadrupedForceBasedStandPrepController implements QuadrupedController, QuadrupedWaypointCallback
{
   //Yo Variables
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   // Parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter trajectoryTimeParameter = parameterFactory.createDouble("trajectoryTime", 1.0);
   private final DoubleParameter stanceLengthParameter = parameterFactory.createDouble("stanceLength", 1.0);
   private final DoubleParameter stanceWidthParameter = parameterFactory.createDouble("stanceWidth", 0.35);
   private final DoubleParameter stanceHeightParameter = parameterFactory.createDouble("stanceHeight", 0.60);
   private final DoubleParameter stanceXOffsetParameter = parameterFactory.createDouble("stanceXOffset", 0.05);
   private final DoubleParameter stanceYOffsetParameter = parameterFactory.createDouble("stanceYOffset", 0.0);
   private final DoubleParameter stancePitchParameter = parameterFactory.createDouble("stancePitch", 0.0);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);
   private final DoubleParameter jointPositionLimitDampingParameter = parameterFactory.createDouble("jointPositionLimitDamping", 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = parameterFactory.createDouble("jointPositionLimitStiffness", 100);
   private final BooleanParameter useForceFeedbackControlParameter = parameterFactory.createBoolean("useForceFeedbackControl", false);

   // Yo variables
   private final YoBoolean yoUseForceFeedbackControl;

   // Task space controller
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   private final QuadrantDependentList<QuadrupedSoleWaypointList> quadrupedSoleWaypointLists = new QuadrantDependentList<>();
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedReferenceFrames referenceFrames;
   private FramePoint3D solePositionSetpoint;
   private final Vector3D zeroVelocity;
   private final double robotLength;
   private final FullQuadrupedRobotModel fullRobotModel;

   private final YoBoolean isDoneMoving = new YoBoolean("standPrepDoneMoving", registry);

   private final QuadrupedForceControllerToolbox controllerToolbox;

   public QuadrupedForceBasedStandPrepController(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                                 YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;

      feetManager = controlManagerFactory.getOrCreateFeetManager();
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
      yoUseForceFeedbackControl = new YoBoolean("useForceFeedbackControl", registry);

      // Calculate the robot length
      referenceFrames.updateFrames();
      FramePoint3D frontLeftHipRollFrame = new FramePoint3D();
      frontLeftHipRollFrame.setToZero(referenceFrames.getLegAttachmentFrame(RobotQuadrant.FRONT_LEFT));
      frontLeftHipRollFrame.changeFrame(referenceFrames.getBodyFrame());
      FramePoint3D hindLeftHipRollFrame = new FramePoint3D();
      hindLeftHipRollFrame.setToZero(referenceFrames.getLegAttachmentFrame(RobotQuadrant.HIND_LEFT));
      hindLeftHipRollFrame.changeFrame(referenceFrames.getBodyFrame());
      robotLength = frontLeftHipRollFrame.getX() - hindLeftHipRollFrame.getX();
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
      // Create sole waypoint trajectories
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionSetpoint.setIncludingFrame(controllerToolbox.getTaskSpaceEstimates().getSolePosition(quadrant));
         solePositionSetpoint.changeFrame(referenceFrames.getBodyFrame());
         quadrupedSoleWaypointLists.get(quadrant).get(0).set(solePositionSetpoint, zeroVelocity, 0.0);
         solePositionSetpoint.setToZero(referenceFrames.getBodyFrame());
         solePositionSetpoint.add(quadrant.getEnd().negateIfHindEnd(stanceLengthParameter.get() / 2.0), 0.0, 0.0);
         solePositionSetpoint.add(0.0, quadrant.getSide().negateIfRightSide(stanceWidthParameter.get() / 2.0), 0.0);
         solePositionSetpoint.add(stanceXOffsetParameter.get(), stanceYOffsetParameter.get(),
               quadrant.getEnd().negateIfHindEnd(Math.sin(stancePitchParameter.get())) * robotLength / 2 - stanceHeightParameter.get());
         quadrupedSoleWaypointLists.get(quadrant).get(1).set(solePositionSetpoint, zeroVelocity, trajectoryTimeParameter.get());
      }
      feetManager.initializeWaypointTrajectory(quadrupedSoleWaypointLists, controllerToolbox.getTaskSpaceEstimates(), false);

      // Initialize task space controller
      taskSpaceControllerSettings.initialize();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(quadrant, ContactState.NO_CONTACT);
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
      controllerToolbox.update();
      feetManager.compute(taskSpaceControllerCommands.getSoleForce(), controllerToolbox.getTaskSpaceEstimates());
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
      return isDoneMoving.getBooleanValue() ? ControllerEvent.DONE : null;
   }

   @Override
   public void onExit()
   {
      yoUseForceFeedbackControl.set(false);  // This bool should match that used in the standReady controller (freeze)
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
