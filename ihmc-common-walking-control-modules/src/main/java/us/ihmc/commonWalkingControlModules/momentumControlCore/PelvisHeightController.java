package us.ihmc.commonWalkingControlModules.momentumControlCore;

import us.ihmc.commonWalkingControlModules.controlModules.YoSE3OffsetFrame;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PelvisHeightController implements HeightController<PointFeedbackControlCommand>
{
   private final MovingReferenceFrame pelvisFrame;
   private final ReferenceFrame baseFrame;

   private final YoDouble currentPelvisHeightInWorld;
   private final YoDouble desiredPelvisHeightInWorld;
   private final YoDouble desiredPelvisVelocityInWorld;
   private final YoDouble currentPelvisVelocityInWorld;

   private final YoDouble feedbackPelvisAcceleration;
   private final YoDouble feedForwardPelvisAcceleration;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final PDController linearMomentumZPDController;
   private final YoSE3OffsetFrame yoControlFrame;

   private final FramePoint3D controlPosition = new FramePoint3D();
   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredLinearVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardLinearAcceleration = new FrameVector3D();
   private final FrameVector3D currentLinearVelocity = new FrameVector3D();
   private final Twist twist = new Twist();

   public PelvisHeightController(MovingReferenceFrame pelvisFrame, ReferenceFrame baseFrame, YoRegistry parentRegistry)
   {
      this.pelvisFrame = pelvisFrame;
      this.baseFrame = baseFrame;

      yoControlFrame = new YoSE3OffsetFrame("pelvisHeightBodyFixedControlFrame", pelvisFrame, registry);
      linearMomentumZPDController = new PDController("pelvisHeightControlState_linearMomentumZPDController", registry);

      currentPelvisHeightInWorld = new YoDouble("currentPelvisHeightInWorld", registry);
      desiredPelvisHeightInWorld = new YoDouble("desiredPelvisHeightInWorld", registry);
      desiredPelvisVelocityInWorld = new YoDouble("desiredPelvisVelocityInWorld", registry);
      currentPelvisVelocityInWorld = new YoDouble("currentPelvisVelocityInWorld", registry);
      feedbackPelvisAcceleration = new YoDouble("feedbackPelvisAcceleration", registry);
      feedForwardPelvisAcceleration = new YoDouble("feedForwardPelvisAcceleration", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void compute(PointFeedbackControlCommand feedbackCommand)
   {
      controlPosition.setIncludingFrame(feedbackCommand.getBodyFixedPointToControl());
      controlPosition.changeFrame(pelvisFrame);
      yoControlFrame.setOffsetToParentToTranslationOnly(controlPosition);
      yoControlFrame.getTwistRelativeToOther(baseFrame, twist);
      currentLinearVelocity.setIncludingFrame(twist.getLinearPart());

      feedForwardLinearAcceleration.setIncludingFrame(feedbackCommand.getReferenceLinearAcceleration());

      desiredPosition.setIncludingFrame(feedbackCommand.getReferencePosition());
      desiredLinearVelocity.setIncludingFrame(feedbackCommand.getReferenceLinearVelocity());

      controlPosition.changeFrame(ReferenceFrame.getWorldFrame());
      currentLinearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      feedForwardLinearAcceleration.changeFrame(ReferenceFrame.getWorldFrame());

      desiredPosition.changeFrame(ReferenceFrame.getWorldFrame());
      desiredLinearVelocity.changeFrame(ReferenceFrame.getWorldFrame());

      currentPelvisHeightInWorld.set(controlPosition.getZ());
      desiredPelvisHeightInWorld.set(desiredPosition.getZ());
      currentPelvisVelocityInWorld.set(currentLinearVelocity.getZ());
      desiredPelvisVelocityInWorld.set(desiredLinearVelocity.getZ());

      linearMomentumZPDController.setProportionalGain(feedbackCommand.getGains().getProportionalGains()[2]);
      linearMomentumZPDController.setDerivativeGain(feedbackCommand.getGains().getDerivativeGains()[2]);
      linearMomentumZPDController.setPositionDeadband(0.0);

      feedForwardPelvisAcceleration.set(feedForwardLinearAcceleration.getZ());
      feedbackPelvisAcceleration.set(linearMomentumZPDController.compute(currentPelvisHeightInWorld.getValue(), desiredPelvisHeightInWorld.getValue(),
                                                 currentPelvisVelocityInWorld.getValue(), desiredPelvisVelocityInWorld.getValue()));
      feedbackPelvisAcceleration.add(feedForwardLinearAcceleration.getZ());
   }

   @Override
   public double getHeightAcceleration()
   {
      return feedbackPelvisAcceleration.getDoubleValue();
   }
}
