package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.aware.util.ContactState;
import us.ihmc.aware.vmc.QuadrupedContactForceLimits;
import us.ihmc.aware.vmc.QuadrupedContactForceOptimizationSettings;
import us.ihmc.aware.vmc.QuadrupedVirtualModelControllerSettings;
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedJointLimits;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTaskSpaceControllerParameters
{
   public final QuadrupedJointLimits jointLimits = new QuadrupedJointLimits();
   public final QuadrupedContactForceLimits contactForceLimits = new QuadrupedContactForceLimits();
   public final QuadrupedVirtualModelControllerSettings virtualModelControllerSettings = new QuadrupedVirtualModelControllerSettings();
   public final QuadrupedContactForceOptimizationSettings contactForceOptimizationSettings = new QuadrupedContactForceOptimizationSettings();
   public final QuadrantDependentList<ContactState> contactState = new QuadrantDependentList<>();

   public QuadrupedTaskSpaceControllerParameters()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
      }
   }

   public QuadrupedJointLimits getJointLimits()
   {
      return jointLimits;
   }

   public QuadrupedContactForceLimits getContactForceLimits()
   {
      return contactForceLimits;
   }

   public QuadrupedVirtualModelControllerSettings getVirtualModelControllerSettings()
   {
      return virtualModelControllerSettings;
   }

   public QuadrupedContactForceOptimizationSettings getContactForceOptimizationSettings()
   {
      return contactForceOptimizationSettings;
   }

   public QuadrantDependentList<ContactState> getContactState()
   {
      return contactState;
   }
}
