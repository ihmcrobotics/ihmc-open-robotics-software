package us.ihmc.commonWalkingControlModules.momentumControlCore;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CoMHeightController implements HeightController<CenterOfMassFeedbackControlCommand>
{
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final PDController comHeightController;

   private final YoDouble currentCoMHeightInWorld;
   private final YoDouble desiredCoMHeightInWorld;
   private final YoDouble desiredCoMVelocityInWorld;
   private final YoDouble currentCoMVelocityInWorld;

   private final YoDouble feedbackCoMAcceleration;
   private final YoDouble feedForwardCoMAcceleration;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FramePoint3D controlPosition = new FramePoint3D();
   private final FrameVector3D feedForwardLinearAcceleration = new FrameVector3D();
   private final FrameVector3D currentLinearVelocity = new FrameVector3D();

   public CoMHeightController(CenterOfMassJacobian centerOfMassJacobian, YoRegistry parentRegistry)
   {
      this.centerOfMassJacobian = centerOfMassJacobian;
      comHeightController = new PDController("CoMHeight", registry);

      currentCoMHeightInWorld = new YoDouble("currentCoMHeightInWorld", registry);
      desiredCoMHeightInWorld = new YoDouble("desiredCoMHeightInWorld", registry);
      desiredCoMVelocityInWorld = new YoDouble("desiredCoMVelocityInWorld", registry);
      currentCoMVelocityInWorld = new YoDouble("currentCoMVelocityInWorld", registry);
      feedbackCoMAcceleration = new YoDouble("feedbackCoMAcceleration", registry);
      feedForwardCoMAcceleration = new YoDouble("feedForwardCoMAcceleration", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void compute(CenterOfMassFeedbackControlCommand feedbackCommand)
   {
      controlPosition.setIncludingFrame(centerOfMassJacobian.getCenterOfMass());
      currentLinearVelocity.setIncludingFrame(centerOfMassJacobian.getCenterOfMassVelocity());

      controlPosition.changeFrame(ReferenceFrame.getWorldFrame());
      currentLinearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      feedForwardLinearAcceleration.changeFrame(ReferenceFrame.getWorldFrame());

      currentCoMHeightInWorld.set(controlPosition.getZ());
      desiredCoMHeightInWorld.set(feedbackCommand.getReferencePosition().getZ());
      currentCoMVelocityInWorld.set(currentLinearVelocity.getZ());
      desiredCoMVelocityInWorld.set(feedbackCommand.getReferenceLinearVelocity().getZ());

      feedForwardLinearAcceleration.setIncludingFrame(feedbackCommand.getReferenceLinearAcceleration());

      comHeightController.setProportionalGain(feedbackCommand.getGains().getProportionalGains()[3]);
      comHeightController.setDerivativeGain(feedbackCommand.getGains().getDerivativeGains()[3]);
      comHeightController.setPositionDeadband(0.0);

      feedForwardCoMAcceleration.set(feedForwardLinearAcceleration.getZ());
      feedbackCoMAcceleration.set(comHeightController.compute(currentCoMHeightInWorld.getValue(), desiredCoMHeightInWorld.getValue(),
                                                              currentCoMVelocityInWorld.getValue(), desiredCoMVelocityInWorld.getValue()));
      feedbackCoMAcceleration.add(feedForwardLinearAcceleration.getZ());
   }

   @Override
   public double getHeightAcceleration()
   {
      return feedbackCoMAcceleration.getDoubleValue();
   }
}
