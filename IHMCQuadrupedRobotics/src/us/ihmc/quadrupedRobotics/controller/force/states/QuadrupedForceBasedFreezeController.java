package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
import us.ihmc.SdfLoader.partNames.JointRole;
import us.ihmc.SdfLoader.partNames.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.params.BooleanParameter;
import us.ihmc.quadrupedRobotics.params.DoubleArrayParameter;
import us.ihmc.quadrupedRobotics.params.DoubleParameter;
import us.ihmc.quadrupedRobotics.params.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class QuadrupedForceBasedFreezeController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(QuadrupedForceBasedFreezeController.class.getSimpleName());

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter jointDampingParameter = parameterFactory.createDouble("jointDamping", 15.0);
   private final DoubleArrayParameter solePositionProportionalGainsParameter = parameterFactory
         .createDoubleArray("solePositionProportionalGains", 20000, 20000, 20000);
   private final DoubleArrayParameter solePositionDerivativeGainsParameter = parameterFactory.createDoubleArray("solePositionDerivativeGains", 200, 200, 200);
   private final DoubleArrayParameter solePositionIntegralGainsParameter = parameterFactory.createDoubleArray("solePositionIntegralGains", 0, 0, 0);
   private final DoubleParameter solePositionMaxIntegralErrorParameter = parameterFactory.createDouble("solePositionMaxIntegralError", 0);
   private final DoubleParameter jointPositionLimitDampingParameter = parameterFactory.createDouble("jointPositionLimitDamping", 10);
   private final DoubleParameter jointPositionLimitStiffnessParameter = parameterFactory.createDouble("jointPositionLimitStiffness", 100);
   private final BooleanParameter useForceFeedbackControl = parameterFactory.createBoolean("useForceFeedbackControl", true);

   // Reference frames
   private final ReferenceFrame bodyFrame;

   // Feedback controller
   private final QuadrantDependentList<QuadrupedSolePositionController> solePositionController;
   private final QuadrantDependentList<QuadrupedSolePositionController.Setpoints> solePositionControllerSetpoints;

   // Task space controller
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final QuadrupedTaskSpaceController.Commands taskSpaceControllerCommands;
   private final QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings;
   private final QuadrupedTaskSpaceController taskSpaceController;

   private SDFFullQuadrupedRobotModel fullRobotModel;

   public QuadrupedForceBasedFreezeController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox)
   {
      // Reference frames
      bodyFrame = controllerToolbox.getReferenceFrames().getBodyFrame();

      // Feedback controller
      solePositionController = controllerToolbox.getSolePositionController();
      solePositionControllerSetpoints = new QuadrantDependentList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionControllerSetpoints.set(quadrant, new QuadrupedSolePositionController.Setpoints(quadrant));
      }

      // Task space controller
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      taskSpaceEstimator = controllerToolbox.getTaskSpaceEstimator();
      taskSpaceControllerCommands = new QuadrupedTaskSpaceController.Commands();
      taskSpaceControllerSettings = new QuadrupedTaskSpaceController.Settings();
      taskSpaceController = controllerToolbox.getTaskSpaceController();
      fullRobotModel = environment.getFullRobotModel();

      environment.getParentRegistry().addChild(registry);
   }

   @Override
   public void onEntry()
   {
      updateEstimates();

      // Initialize sole position controller
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionController.get(quadrant).reset();
         solePositionControllerSetpoints.get(quadrant).initialize(taskSpaceEstimates);
      }

      // Initialize task space controller
      taskSpaceControllerSettings.initialize();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         taskSpaceControllerSettings.setContactState(quadrant, ContactState.NO_CONTACT);
      }
      taskSpaceController.reset();

      // Initial sole position setpoints
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionControllerSetpoints.get(quadrant).getSolePosition().setIncludingFrame(taskSpaceEstimates.getSolePosition(quadrant));
         solePositionControllerSetpoints.get(quadrant).getSolePosition().changeFrame(bodyFrame);
      }

      // Initialize force feedback
      for (QuadrupedJointName jointName : QuadrupedJointName.values)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointName);
         if (oneDoFJoint != null && jointName.getRole().equals(JointRole.LEG))
         {
            oneDoFJoint.setUseFeedBackForceControl(useForceFeedbackControl.get());
         }
      }
   }

   @Override
   public ControllerEvent process()
   {
      updateGains();
      updateEstimates();
      updateSetpoints();
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());

      return null;
   }

   @Override
   public void onExit()
   {
      for (QuadrupedJointName jointName : QuadrupedJointName.values)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointName);
         if (oneDoFJoint != null && jointName.getRole().equals(JointRole.LEG))
         {
            oneDoFJoint.setUseFeedBackForceControl(true);
         }
      }
   }

   private void updateGains()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionController.get(quadrant).getGains().setProportionalGains(solePositionProportionalGainsParameter.get());
         solePositionController.get(quadrant).getGains().setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
         solePositionController.get(quadrant).getGains().setDerivativeGains(solePositionDerivativeGainsParameter.get());
      }
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointDamping(jointDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitDamping(jointPositionLimitDampingParameter.get());
      taskSpaceControllerSettings.getVirtualModelControllerSettings().setJointPositionLimitStiffness(jointPositionLimitStiffnessParameter.get());
   }

   private void updateEstimates()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
   }

   private void updateSetpoints()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionController.get(quadrant)
               .compute(taskSpaceControllerCommands.getSoleForce(quadrant), solePositionControllerSetpoints.get(quadrant), taskSpaceEstimates);
      }
      taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
   }
}
