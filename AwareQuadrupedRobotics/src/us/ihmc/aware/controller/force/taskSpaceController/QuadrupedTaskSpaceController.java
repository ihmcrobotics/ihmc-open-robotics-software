package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.aware.util.ContactState;
import us.ihmc.aware.vmc.QuadrupedContactForceOptimization;
import us.ihmc.aware.vmc.QuadrupedVirtualModelController;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import java.util.ArrayList;

public class QuadrupedTaskSpaceController
{
   private static final int FEEDBACK_BLOCK_CAPACITY = 50;
   private static final int COMMAND_FILTER_CAPACITY = 50;

   private final ArrayList<QuadrupedTaskSpaceFeedbackBlock> feedbackBlocks;
   private final ArrayList<QuadrupedTaskSpaceCommandFilter> commandFilters;
   private final QuadrupedTaskSpaceCommands feedbackCommands;
   private final QuadrupedTaskSpaceCommands filteredCommands;
   private final QuadrupedVirtualModelController virtualModelController;
   private final QuadrupedContactForceOptimization contactForceOptimization;
   private final FrameVector contactForceStorage;
   private final YoVariableRegistry registry = new YoVariableRegistry("quadrupedTaskSpaceController");

   public QuadrupedTaskSpaceController(SDFFullRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, QuadrupedJointNameMap jointNameMap, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      feedbackBlocks = new ArrayList<>(FEEDBACK_BLOCK_CAPACITY);
      commandFilters = new ArrayList<>(COMMAND_FILTER_CAPACITY);
      feedbackCommands = new QuadrupedTaskSpaceCommands();
      filteredCommands = new QuadrupedTaskSpaceCommands();
      virtualModelController = new QuadrupedVirtualModelController(fullRobotModel, referenceFrames, jointNameMap, registry, graphicsListRegistry);
      contactForceOptimization = new QuadrupedContactForceOptimization(referenceFrames, registry);
      contactForceStorage = new FrameVector();
      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      virtualModelController.reset();
      contactForceOptimization.reset();
      for (int i = 0; i < feedbackBlocks.size(); i++)
      {
         feedbackBlocks.get(i).reset();
      }
   }

   public void compute(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceControllerParameters parameters)
   {
      // initialize commands to zero
      filteredCommands.setToZero();
      filteredCommands.changeFrame(ReferenceFrame.getWorldFrame());

      // compute feedback commands
      for (int i = 0; i < feedbackBlocks.size(); i++)
      {
         feedbackCommands.setToZero();
         feedbackBlocks.get(i).compute(estimates, feedbackCommands);
         feedbackCommands.changeFrame(ReferenceFrame.getWorldFrame());
         filteredCommands.add(feedbackCommands);
      }

      // apply command filters
      for (int i = 0; i < commandFilters.size(); i++)
      {
         commandFilters.get(i).compute(estimates, filteredCommands);
      }

      // compute optimal contact force distribution for quadrants that are in contact
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         // note: sole forces are inverted to obtain commanded reaction forces
         filteredCommands.getSoleForce().get(robotQuadrant).scale(-1.0);
         contactForceOptimization.setContactForceCommand(robotQuadrant, filteredCommands.getSoleForce().get(robotQuadrant));
         filteredCommands.getSoleForce().get(robotQuadrant).scale(-1.0);
         contactForceOptimization.setContactState(robotQuadrant, parameters.contactState.get(robotQuadrant));
      }
      contactForceOptimization.setComForceCommand(filteredCommands.getComForce());
      contactForceOptimization.setComTorqueCommand(filteredCommands.getComForce());
      contactForceOptimization.solve(parameters.contactForceLimits, parameters.contactForceOptimizationSettings);

      // compute leg joint torques using jacobian transpose
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (parameters.contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            contactForceOptimization.getContactForceSolution(robotQuadrant, contactForceStorage);
            virtualModelController.setSoleContactForce(robotQuadrant, contactForceStorage);
            virtualModelController.setSoleContactForceVisible(robotQuadrant, true);
            virtualModelController.setSoleVirtualForceVisible(robotQuadrant, false);
         }
         else
         {
            virtualModelController.setSoleVirtualForce(robotQuadrant, filteredCommands.getSoleForce().get(robotQuadrant));
            virtualModelController.setSoleContactForceVisible(robotQuadrant, false);
            virtualModelController.setSoleVirtualForceVisible(robotQuadrant, true);
         }
      }
      virtualModelController.compute(parameters.jointLimits, parameters.virtualModelControllerSettings);
   }
}
