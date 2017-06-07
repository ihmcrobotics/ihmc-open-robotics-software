package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

public class LeapOfFaithModule
{
   private final ExternalWrenchCommand externalWrenchCommand = new ExternalWrenchCommand();
   private final Wrench footWrench;
   private final RigidBody footBody;

   private final DoubleYoVariable swingDuration;
   private final double stiffness = 100.0;

   public LeapOfFaithModule(DoubleYoVariable swingDuration, RigidBody footBody)
   {
      this.swingDuration = swingDuration;
      this.footBody = footBody;

      this.footWrench = new Wrench(footBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame());
   }

   public void compute(double currentTime)
   {
      double timePastEnd = Math.max(currentTime - swingDuration.getDoubleValue(), 0.0);

      footWrench.setToZero(footBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame());
      footWrench.setLinearPartZ(-stiffness * timePastEnd);

      externalWrenchCommand.set(footBody, footWrench);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return externalWrenchCommand;
   }
}
