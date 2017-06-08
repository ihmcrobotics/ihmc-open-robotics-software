package us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class FootLeapOfFaithModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final ExternalWrenchCommand externalWrenchCommand = new ExternalWrenchCommand();
   private final YoWrench footWrench;
   private final RigidBody footBody;

   private final FrameVector tempVector = new FrameVector();
   private final DoubleYoVariable swingDuration;

   private final DoubleYoVariable gain = new DoubleYoVariable("leapOfFaithFootGain", registry);

   public FootLeapOfFaithModule(DoubleYoVariable swingDuration, RigidBody footBody, YoVariableRegistry parentRegistry)
   {
      this.swingDuration = swingDuration;
      this.footBody = footBody;

      footWrench = new YoWrench("leapOfFaithFoot", "", footBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), registry);

      gain.set(100.0);

      parentRegistry.addChild(registry);
   }

   public void compute(double currentTime)
   {
      double timePastEnd = Math.max(currentTime - swingDuration.getDoubleValue(), 0.0);

      footWrench.setToZero();
      tempVector.setToZero(ReferenceFrame.getWorldFrame());
      tempVector.setZ(-gain.getDoubleValue() * timePastEnd);
      footWrench.setLinearPart(tempVector);

      externalWrenchCommand.set(footBody, footWrench.getWrench());
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return externalWrenchCommand;
   }
}
