package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.aware.controller.force.taskSpaceController.feedbackController.QuadrupedBodyOrientationFeedbackController;
import us.ihmc.aware.controller.force.taskSpaceController.feedbackController.QuadrupedComPositionFeedbackController;
import us.ihmc.aware.controller.force.taskSpaceController.feedbackController.QuadrupedSolePositionFeebackController;
import us.ihmc.aware.controller.force.taskSpaceController.feedbackController.QuadrupedTaskSpaceFeedbackController;
import us.ihmc.aware.util.ContactState;
import us.ihmc.aware.vmc.*;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedJointLimits;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import java.util.ArrayList;

public class QuadrupedTaskSpaceController
{
   private final QuadrupedJointLimits jointLimits;
   private final QuadrupedVirtualModelController virtualModelController;
   private final QuadrupedVirtualModelControllerSettings virtualModelControllerSettings;
   private final QuadrupedContactForceLimits contactForceLimits;
   private final QuadrupedContactForceOptimization contactForceOptimization;
   private final QuadrupedContactForceOptimizationSettings contactForceOptimizationSettings;
   private final FrameVector contactForceStorage;
   private final QuadrupedComPositionFeedbackController comPositionFeedbackController;
   private final QuadrupedBodyOrientationFeedbackController bodyOrientationFeedbackController;
   private final QuadrantDependentList<QuadrupedSolePositionFeebackController> solePositionFeedbackController;
   private final ArrayList<QuadrupedTaskSpaceFeedbackController> feedbackControllers;
   private final QuadrupedTaskSpaceCommands feedbackCommands;

   private final YoVariableRegistry registry = new YoVariableRegistry("taskSpaceController");

   public QuadrupedTaskSpaceController(SDFFullRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, QuadrupedJointNameMap jointNameMap, QuadrupedJointLimits jointLimits, double controlDT, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.jointLimits = jointLimits;

      // virtual model controller
      virtualModelController = new QuadrupedVirtualModelController(fullRobotModel, referenceFrames, jointNameMap, registry, graphicsListRegistry);
      virtualModelControllerSettings = new QuadrupedVirtualModelControllerSettings();
      contactForceLimits = new QuadrupedContactForceLimits();
      contactForceOptimization = new QuadrupedContactForceOptimization(referenceFrames, registry);
      contactForceOptimizationSettings = new QuadrupedContactForceOptimizationSettings();
      contactForceStorage = new FrameVector();

      // feedback controllers
      comPositionFeedbackController = new QuadrupedComPositionFeedbackController(referenceFrames.getCenterOfMassZUpFrame(), controlDT, registry);
      bodyOrientationFeedbackController = new QuadrupedBodyOrientationFeedbackController(referenceFrames.getBodyFrame(), controlDT, registry);
      solePositionFeedbackController = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionFeedbackController.set(robotQuadrant, new QuadrupedSolePositionFeebackController(robotQuadrant, referenceFrames.getFootFrame(robotQuadrant), controlDT, registry));
      }
      feedbackControllers = new ArrayList<>();
      feedbackControllers.add(comPositionFeedbackController);
      feedbackControllers.add(bodyOrientationFeedbackController);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         feedbackControllers.add(solePositionFeedbackController.get(robotQuadrant));
      }
      feedbackCommands = new QuadrupedTaskSpaceCommands();
      parentRegistry.addChild(registry);
      reset();
   }

   public void reset()
   {
      virtualModelController.reset();
      contactForceOptimization.reset();
      for (int i = 0; i < feedbackControllers.size(); i++)
      {
         feedbackControllers.get(i).reset();
      }
   }

   public void compute(QuadrupedTaskSpaceControllerSettings settings, QuadrupedTaskSpaceSetpoints setpoints, QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceCommands commands)
   {
      // initialize commands
      commands.setToZero();
      commands.changeFrame(ReferenceFrame.getWorldFrame());

      // initialize feedback controllers
      settings.getBodyOrientationFeedbackGains(bodyOrientationFeedbackController.getGains());
      settings.getComPositionFeedbackGains(comPositionFeedbackController.getGains());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         settings.getSolePositionFeedbackGains(robotQuadrant, solePositionFeedbackController.get(robotQuadrant).getGains());
      }

      // compute commands
      for (int i = 0; i < feedbackControllers.size(); i++)
      {
         feedbackCommands.setToZero();
         feedbackControllers.get(i).computeFeedback(estimates, setpoints, feedbackCommands);
         feedbackCommands.changeFrame(ReferenceFrame.getWorldFrame());
         commands.add(feedbackCommands);
      }

      // compute optimal contact force distribution for quadrants that are in contact
      settings.getContactForceOptimizationSettings(contactForceOptimizationSettings);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         // note: sole forces are inverted to obtain commanded reaction forces
         commands.getSoleForce().get(robotQuadrant).scale(-1.0);
         contactForceOptimization.setContactForceCommand(robotQuadrant, commands.getSoleForce().get(robotQuadrant));
         commands.getSoleForce().get(robotQuadrant).scale(-1.0);
         contactForceOptimization.setContactState(robotQuadrant, settings.getContactState(robotQuadrant));
      }
      contactForceOptimization.setComForceCommand(commands.getComForce());
      contactForceOptimization.setComTorqueCommand(commands.getComTorque());
      contactForceOptimization.solve(contactForceLimits, contactForceOptimizationSettings);

      // compute leg joint torques using jacobian transpose
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (settings.getContactState(robotQuadrant) == ContactState.IN_CONTACT)
         {
            contactForceOptimization.getContactForceSolution(robotQuadrant, contactForceStorage);
            virtualModelController.setSoleContactForce(robotQuadrant, contactForceStorage);
            virtualModelController.setSoleContactForceVisible(robotQuadrant, true);
            virtualModelController.setSoleVirtualForceVisible(robotQuadrant, false);
         }
         else
         {
            virtualModelController.setSoleVirtualForce(robotQuadrant, commands.getSoleForce().get(robotQuadrant));
            virtualModelController.setSoleContactForceVisible(robotQuadrant, false);
            virtualModelController.setSoleVirtualForceVisible(robotQuadrant, true);
         }
      }
      virtualModelController.compute(jointLimits, virtualModelControllerSettings);
   }
}
