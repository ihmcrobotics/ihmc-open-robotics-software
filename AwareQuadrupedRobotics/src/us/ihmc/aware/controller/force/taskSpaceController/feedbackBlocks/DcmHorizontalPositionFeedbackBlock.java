package us.ihmc.aware.controller.force.taskSpaceController.feedbackBlocks;

import us.ihmc.aware.controller.common.DivergentComponentOfMotionController;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceFeedbackBlock;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class DcmHorizontalPositionFeedbackBlock implements QuadrupedTaskSpaceFeedbackBlock
{
   private final FramePoint dcmPositionSetpoint;
   private final FrameVector dcmVelocitySetpoint;
   private final FramePoint icpPositionSetpoint;
   private final FrameVector icpVelocitySetpoint;
   private final FramePoint cmpPositionSetpoint;
   private final FramePoint vrpPositionSetpoint;
   private final FrameVector comForceCommand;
   private final DivergentComponentOfMotionController dcmPositionController;

   public DcmHorizontalPositionFeedbackBlock(ReferenceFrame comZUpFrame, double controlDT, double mass, double gravity, double naturalFrequency, YoVariableRegistry registry)
   {
      dcmPositionSetpoint = new FramePoint();
      dcmVelocitySetpoint = new FrameVector();
      icpPositionSetpoint = new FramePoint();
      icpVelocitySetpoint = new FrameVector();
      cmpPositionSetpoint = new FramePoint();
      vrpPositionSetpoint = new FramePoint();
      comForceCommand = new FrameVector();
      dcmPositionController = new DivergentComponentOfMotionController("dcmPosition", comZUpFrame, controlDT, mass, gravity, naturalFrequency, registry);
   }

   public void setDcmPositionSetpoint(FramePoint dcmPositionSetpoint)
   {
      this.dcmPositionSetpoint.setIncludingFrame(dcmPositionSetpoint);
   }

   public void setDcmVelocitySetpoint(FramePoint dcmVelocitySetpoint)
   {
      this.dcmVelocitySetpoint.setIncludingFrame(dcmVelocitySetpoint);
   }

   public void setDcmNaturalFrequency(double dcmNaturalFrequency)
   {
      dcmPositionController.setNaturalFrequency(Math.max(dcmNaturalFrequency, 0.001));
   }

   public void setProportionalGains(double[] proportionalGains)
   {
     this.dcmPositionController.setProportionalGains(proportionalGains);
   }

   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      this.dcmPositionController.setIntegralGains(integralGains, maxIntegralError);
   }

   @Override public void reset()
   {
      dcmPositionController.reset();
   }

   @Override public void compute(QuadrupedTaskSpaceEstimates estimates, QuadrupedTaskSpaceCommands commands)
   {
      FramePoint dcmPositionEstimate = estimates.getDcmPosition();
      dcmPositionController.compute(comForceCommand, vrpPositionSetpoint, cmpPositionSetpoint, dcmPositionSetpoint, dcmVelocitySetpoint, dcmPositionEstimate);
      icpPositionSetpoint.setIncludingFrame(dcmPositionSetpoint);
      icpPositionSetpoint.sub(0, 0, dcmPositionController.getComHeightConstant());
      icpVelocitySetpoint.setIncludingFrame(dcmVelocitySetpoint);
      commands.getComForce().setIncludingFrame(comForceCommand);
      commands.getComForce().changeFrame(ReferenceFrame.getWorldFrame());
      commands.getComForce().setZ(0.0);
   }
}
