package us.ihmc.quadrupedRobotics.controller.force.states;

import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimator;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.robotics.dataStructures.parameter.BooleanParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleArrayParameter;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
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
   private final BooleanParameter useForceFeedbackControlParameter = parameterFactory.createBoolean("useForceFeedbackControl", false);
   private final BooleanParameter useSoleForceFeedForwardParameter = parameterFactory.createBoolean("useSoleForceFeedForward", true);
   private final DoubleParameter feedForwardRampTimeParameter = parameterFactory.createDouble("feedForwardRampTime", 2.0);

   // Yo variables
   private final BooleanYoVariable yoUseForceFeedbackControl;

   private final QuadrantDependentList<Double[]> initialSoleForces;
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
   private final DoubleYoVariable robotTimestamp;
   private double initialTime;

   private FullQuadrupedRobotModel fullRobotModel;

   public QuadrupedForceBasedFreezeController(QuadrupedRuntimeEnvironment environment, QuadrupedForceControllerToolbox controllerToolbox)
   {
      this.robotTimestamp = environment.getRobotTimestamp();
      // Reference frames
      bodyFrame = controllerToolbox.getReferenceFrames().getBodyFrame();
      // Yo variables
      yoUseForceFeedbackControl = new BooleanYoVariable("useForceFeedbackControl", registry);
      initialSoleForces = new QuadrantDependentList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         initialSoleForces.set(quadrant, new Double[3]);
      }
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
      initialTime = robotTimestamp.getDoubleValue();
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
         taskSpaceEstimates.getSoleVirtualForce(quadrant).changeFrame(bodyFrame);
         initialSoleForces.get(quadrant)[0] = taskSpaceEstimates.getSoleVirtualForce(quadrant).getX();
         initialSoleForces.get(quadrant)[1] = taskSpaceEstimates.getSoleVirtualForce(quadrant).getY();
         initialSoleForces.get(quadrant)[2] = taskSpaceEstimates.getSoleVirtualForce(quadrant).getZ();
      }
      yoUseForceFeedbackControl.set(useForceFeedbackControlParameter.get());
      // Initialize force feedback
      for (QuadrupedJointName jointName : QuadrupedJointName.values())
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointName);
         if (oneDoFJoint != null && jointName.getRole().equals(JointRole.LEG))
         {
            oneDoFJoint.setUseFeedBackForceControl(yoUseForceFeedbackControl.getBooleanValue());
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
      yoUseForceFeedbackControl.set(true);
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
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         solePositionController.get(quadrant).getGains().setProportionalGains(solePositionProportionalGainsParameter.get());
         solePositionController.get(quadrant).getGains()
               .setIntegralGains(solePositionIntegralGainsParameter.get(), solePositionMaxIntegralErrorParameter.get());
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
      double currentTime = robotTimestamp.getDoubleValue();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
      {
         if (useSoleForceFeedForwardParameter.get())
         {
            double rampMultiplier = 1 - Math.min(1.0, (currentTime - initialTime) / feedForwardRampTimeParameter.get());
            solePositionControllerSetpoints.get(quadrant).getSoleForceFeedforward()
                  .set(rampMultiplier * initialSoleForces.get(quadrant)[0],
                        rampMultiplier * initialSoleForces.get(quadrant)[1],
                        rampMultiplier * initialSoleForces.get(quadrant)[2]);
         } solePositionController.get(quadrant)
            .compute(taskSpaceControllerCommands.getSoleForce(quadrant), solePositionControllerSetpoints.get(quadrant), taskSpaceEstimates);
      } taskSpaceController.compute(taskSpaceControllerSettings, taskSpaceControllerCommands);
   }
}
