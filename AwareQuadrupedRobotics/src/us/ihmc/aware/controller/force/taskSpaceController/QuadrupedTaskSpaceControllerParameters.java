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
   public final QuadrupedJointLimits jointLimits;
   public final QuadrupedContactForceLimits contactForceLimits;
   public final QuadrupedVirtualModelControllerSettings virtualModelControllerSettings;
   public final QuadrupedContactForceOptimizationSettings contactForceOptimizationSettings;
   public final QuadrantDependentList<ContactState> contactState;

   public QuadrupedTaskSpaceControllerParameters(QuadrupedJointLimits jointLimits, QuadrupedContactForceLimits contactForceLimits)
   {
      this.jointLimits = new QuadrupedJointLimits(jointLimits);
      this.contactForceLimits = new QuadrupedContactForceLimits(contactForceLimits);
      this.virtualModelControllerSettings = new QuadrupedVirtualModelControllerSettings();
      this.contactForceOptimizationSettings = new QuadrupedContactForceOptimizationSettings();
      this.contactState = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
      }
   }

   public QuadrupedTaskSpaceControllerParameters(QuadrupedJointLimits jointLimits)
   {
      this(jointLimits, new QuadrupedContactForceLimits());
   }

   public QuadrupedTaskSpaceControllerParameters()
   {
      this(new QuadrupedJointLimits(), new QuadrupedContactForceLimits());
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
