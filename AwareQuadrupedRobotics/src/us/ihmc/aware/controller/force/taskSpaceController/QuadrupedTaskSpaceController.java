package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.aware.util.ContactState;
import us.ihmc.aware.vmc.*;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedJointLimits;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import java.util.ArrayList;

public class QuadrupedTaskSpaceController
{
   private static final int CONTROL_BLOCK_CAPACITY = 50;
   private static final int FILTER_BLOCK_CAPACITY = 50;

   private final QuadrupedJointLimits jointLimits;
   private final QuadrupedVirtualModelController virtualModelController;
   private final QuadrupedVirtualModelControllerSettings virtualModelControllerSettings;
   private final QuadrupedContactForceLimits contactForceLimits;
   private final QuadrupedContactForceOptimization contactForceOptimization;
   private final QuadrupedContactForceOptimizationSettings contactForceOptimizationSettings;
   private final FrameVector contactForceStorage;
   private final QuadrupedTaskSpaceCommands controlCommands;
   private final ArrayList<QuadrupedTaskSpaceControlBlock> controlBlocks;
   private final ArrayList<QuadrupedTaskSpaceFilterBlock> filterBlocks;
   private final YoVariableRegistry registry = new YoVariableRegistry("taskSpaceController");

   public QuadrupedTaskSpaceController(SDFFullRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, QuadrupedJointNameMap jointNameMap, QuadrupedJointLimits jointLimits, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.jointLimits = jointLimits;
      virtualModelController = new QuadrupedVirtualModelController(fullRobotModel, referenceFrames, jointNameMap, registry, graphicsListRegistry);
      virtualModelControllerSettings = new QuadrupedVirtualModelControllerSettings();
      contactForceLimits = new QuadrupedContactForceLimits();
      contactForceOptimization = new QuadrupedContactForceOptimization(referenceFrames, registry);
      contactForceOptimizationSettings = new QuadrupedContactForceOptimizationSettings();
      contactForceStorage = new FrameVector();
      controlCommands = new QuadrupedTaskSpaceCommands();
      controlBlocks = new ArrayList<>(CONTROL_BLOCK_CAPACITY);
      filterBlocks = new ArrayList<>(FILTER_BLOCK_CAPACITY);
      parentRegistry.addChild(registry);
   }

   public void addControlBlock(QuadrupedTaskSpaceControlBlock controlBlock)
   {
      controlBlocks.add(controlBlock);
   }

   public void removeControlBlock(QuadrupedTaskSpaceControlBlock controlBlock)
   {
      controlBlocks.remove(controlBlock);
   }

   public void removeControlBlocks()
   {
      controlBlocks.clear();
   }

   public void addFilterBlock(QuadrupedTaskSpaceFilterBlock filterBlock)
   {
      filterBlocks.add(filterBlock);
   }

   public void removeFilterBlock(QuadrupedTaskSpaceFilterBlock filterBlock)
   {
      filterBlocks.remove(filterBlock);
   }

   public void removeFilterBlocks()
   {
      filterBlocks.clear();
   }

   public void reset()
   {
      virtualModelController.reset();
      contactForceOptimization.reset();
      for (int i = 0; i < controlBlocks.size(); i++)
      {
         controlBlocks.get(i).reset();
      }
      for (int i = 0; i < filterBlocks.size(); i++)
      {
         filterBlocks.get(i).reset();
      }
   }

   public void compute(QuadrupedTaskSpaceEstimates inputEstimates, QuadrupedTaskSpaceCommands outputCommands, QuadrupedTaskSpaceControllerSettings settings)
   {
      // initialize commands
      outputCommands.setToZero();
      outputCommands.changeFrame(ReferenceFrame.getWorldFrame());

      // compute control commands
      for (int i = 0; i < controlBlocks.size(); i++)
      {
         controlCommands.setToZero();
         controlBlocks.get(i).compute(inputEstimates, controlCommands);
         controlCommands.changeFrame(ReferenceFrame.getWorldFrame());
         outputCommands.add(controlCommands);
      }

      // compute filtered commands
      for (int i = 0; i < filterBlocks.size(); i++)
      {
         filterBlocks.get(i).compute(outputCommands);
      }

      // compute optimal contact force distribution for quadrants that are in contact
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         // note: sole forces are inverted to obtain commanded reaction forces
         outputCommands.getSoleForce().get(robotQuadrant).scale(-1.0);
         contactForceOptimization.setContactForceCommand(robotQuadrant, outputCommands.getSoleForce().get(robotQuadrant));
         outputCommands.getSoleForce().get(robotQuadrant).scale(-1.0);
         contactForceOptimizationSettings.setContactForceCommandWeights(robotQuadrant, settings.getSoleForceCommandWeights(robotQuadrant));
         contactForceOptimization.setContactState(robotQuadrant, settings.getContactState(robotQuadrant));
      }
      contactForceOptimization.setComForceCommand(outputCommands.getComForce());
      contactForceOptimizationSettings.setComForceCommandWeights(settings.getComForceCommandWeights());
      contactForceOptimization.setComTorqueCommand(outputCommands.getComTorque());
      contactForceOptimizationSettings.setComTorqueCommandWeights(settings.getComTorqueCommandWeights());
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
            virtualModelController.setSoleVirtualForce(robotQuadrant, outputCommands.getSoleForce().get(robotQuadrant));
            virtualModelController.setSoleContactForceVisible(robotQuadrant, false);
            virtualModelController.setSoleVirtualForceVisible(robotQuadrant, true);
         }
      }
      virtualModelController.compute(jointLimits, virtualModelControllerSettings);
   }
}
