package us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith;

import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
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

   private final BooleanYoVariable useFootForce = new BooleanYoVariable("leapOfFaithUseFootForce", registry);
   private final DoubleYoVariable gain = new DoubleYoVariable("leapOfFaithFootForceGain", registry);

   public FootLeapOfFaithModule(DoubleYoVariable swingDuration, RigidBody footBody, LeapOfFaithParameters parameters, YoVariableRegistry parentRegistry)
   {
      this.swingDuration = swingDuration;
      this.footBody = footBody;

      footWrench = new YoWrench("leapOfFaithFoot", "", footBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), registry);

      useFootForce.set(parameters.useFootForce());
      gain.set(parameters.getFootForceGain());

      parentRegistry.addChild(registry);
   }

   public void compute(double currentTime)
   {
      footWrench.setToZero();

      if (useFootForce.getBooleanValue())
      {
         double timePastEnd = Math.max(currentTime - swingDuration.getDoubleValue(), 0.0);

         tempVector.setToZero(ReferenceFrame.getWorldFrame());
         tempVector.setZ(-gain.getDoubleValue() * timePastEnd);
         footWrench.setLinearPart(tempVector);
      }

      externalWrenchCommand.set(footBody, footWrench.getWrench());
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return externalWrenchCommand;
   }
}
