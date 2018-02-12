package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineState;

public abstract class QuadrupedFootState implements FiniteStateMachineState<QuadrupedFootControlModule.FootEvent>
{
   protected final FrameVector3D soleForceCommand = new FrameVector3D();
   protected final QuadrupedTaskSpaceEstimates estimates = new QuadrupedTaskSpaceEstimates();

   public abstract void updateEstimates(QuadrupedTaskSpaceEstimates estimates);

   public FrameVector3DReadOnly getSoleForceCommand()
   {
      return soleForceCommand;
   }
}
