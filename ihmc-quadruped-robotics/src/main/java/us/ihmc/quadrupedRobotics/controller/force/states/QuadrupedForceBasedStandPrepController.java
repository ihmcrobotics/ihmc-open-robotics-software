package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSoleWaypointController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
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
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class QuadrupedForceBasedStandPrepController implements QuadrupedController
{
   //Yo Variables
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoPID3DGains yoPositionControllerGains;

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
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory
         .createDoubleArray("solePositionProportionalGains", 10000, 10000, 10000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 100, 100, 100);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);
   private final BooleanParameter useForceFeedbackControlParameter = parameterFactory.createBoolean("useForceFeedbackControl", false);

   // Yo variables
   private final YoBoolean yoUseForceFeedbackControl;

   // Task space controller
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   private QuadrupedSoleWaypointList quadrupedSoleWaypointList;
   private final QuadrupedSoleWaypointController quadrupedSoleWaypointController;
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedReferenceFrames referenceFrames;
   private FramePoint3D solePositionSetpoint;
   private final Vector3D zeroVelocity;
   private final double robotLength;
   private FullQuadrupedRobotModel fullRobotModel;

   public QuadrupedForceBasedStandPrepController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox)
   {
      quadrupedSoleWaypointController = controllerToolbox.getSoleWaypointController();
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      referenceFrames = controllerToolbox.getReferenceFrames();
      quadrupedSoleWaypointList = new QuadrupedSoleWaypointList();
      solePositionSetpoint = new FramePoint3D();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         quadrupedSoleWaypointList.get(quadrant).add(new SoleWaypoint());
         quadrupedSoleWaypointList.get(quadrant).add(new SoleWaypoint());
      }
      zeroVelocity = new Vector3D(0, 0, 0);
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      this.taskSpaceController = controllerToolbox.getTaskSpaceController();
      yoPositionControllerGains = new DefaultYoPID3DGains("positionControllerGains", GainCoupling.NONE, true, registry);
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
      fullRobotModel = environment.getFullRobotModel();
      environment.getParentRegistry().addChild(registry);
   }

   @Override
   public void onEntry()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
      updateGains();
      // Create sole waypoint trajectories
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionSetpoint.setIncludingFrame(taskSpaceEstimates.getSolePosition(quadrant));
         solePositionSetpoint.changeFrame(referenceFrames.getBodyFrame());
         quadrupedSoleWaypointList.get(quadrant).get(0).set(solePositionSetpoint, zeroVelocity, 0.0);
         solePositionSetpoint.setToZero(referenceFrames.getBodyFrame());
         solePositionSetpoint.add(quadrant.getEnd().negateIfHindEnd(stanceLengthParameter.get() / 2.0), 0.0, 0.0);
         solePositionSetpoint.add(0.0, quadrant.getSide().negateIfRightSide(stanceWidthParameter.get() / 2.0), 0.0);
         solePositionSetpoint.add(stanceXOffsetParameter.get(), stanceYOffsetParameter.get(),
               quadrant.getEnd().negateIfHindEnd(Math.sin(stancePitchParameter.get())) * robotLength / 2 - stanceHeightParameter.get());
         quadrupedSoleWaypointList.get(quadrant).get(1).set(solePositionSetpoint, zeroVelocity, trajectoryTimeParameter.get());
      }
      quadrupedSoleWaypointController.initialize(quadrupedSoleWaypointList, yoPositionControllerGains, taskSpaceEstimates, false);

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
   }

   @Override
   public ControllerEvent process()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
      boolean success = quadrupedSoleWaypointController.compute(taskSpaceControllerCommands.getSoleForce(), taskSpaceEstimates);
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
      return success ? null : ControllerEvent.DONE;
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
   }

   private void updateGains()
   {
      yoPositionControllerGains.setProportionalGains(solePositionProportionalGainsParameter.get());
      yoPositionControllerGains.setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
      yoPositionControllerGains.setDerivativeGains(solePositionDerivativeGainsParameter.get());
   }
}
