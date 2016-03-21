package us.ihmc.aware.controller.force.taskSpaceController.controlBlocks;

import us.ihmc.aware.controller.common.DivergentComponentOfMotionController;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceControlBlock;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Vector3d;

public class DcmPositionControlBlock implements QuadrupedTaskSpaceControlBlock
{
   private final Vector3d commandMask;
   private final ReferenceFrame comZUpFrame;
   private final FramePoint dcmPositionSetpoint;
   private final FrameVector dcmVelocitySetpoint;
   private final FramePoint icpPositionSetpoint;
   private final FrameVector icpVelocitySetpoint;
   private final FramePoint cmpPositionSetpoint;
   private final FramePoint vrpPositionSetpoint;
   private final DivergentComponentOfMotionController dcmPositionController;

   public DcmPositionControlBlock(ReferenceFrame comZUpFrame, double controlDT, double mass, double gravity, YoVariableRegistry registry)
   {
      this.comZUpFrame = comZUpFrame;
      commandMask = new Vector3d(1, 1, 1);
      dcmPositionSetpoint = new FramePoint();
      dcmVelocitySetpoint = new FrameVector();
      icpPositionSetpoint = new FramePoint();
      icpVelocitySetpoint = new FrameVector();
      cmpPositionSetpoint = new FramePoint();
      vrpPositionSetpoint = new FramePoint();
      dcmPositionController = new DivergentComponentOfMotionController("dcmPosition", comZUpFrame, controlDT, mass, gravity, 1.0, registry);
   }

   public void setDcmPosition(FramePoint dcmPositionSetpoint)
   {
      this.dcmPositionSetpoint.setIncludingFrame(dcmPositionSetpoint);
   }

   public void setDcmVelocity(FrameVector dcmVelocitySetpoint)
   {
      this.dcmVelocitySetpoint.setIncludingFrame(dcmVelocitySetpoint);
   }

   public void setCommandMask(double maskX, double maskY, double maskZ)
   {
      commandMask.set(maskX, maskY, maskZ);
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
      dcmVelocitySetpoint.setToZero();
      dcmPositionController.reset();
   }

   @Override public void compute(QuadrupedTaskSpaceEstimates inputEstimates, QuadrupedTaskSpaceCommands outputCommands)
   {
      FramePoint dcmPositionEstimate = inputEstimates.getDcmPosition();
      FrameVector comForceCommand = outputCommands.getComForce();

      // compute com force
      comForceCommand.setToZero(comZUpFrame);
      dcmPositionController.setNaturalFrequency(inputEstimates.getLipNaturalFrequency());
      dcmPositionController.compute(comForceCommand, vrpPositionSetpoint, cmpPositionSetpoint, dcmPositionSetpoint, dcmVelocitySetpoint, dcmPositionEstimate);
      icpPositionSetpoint.setIncludingFrame(dcmPositionSetpoint);
      icpPositionSetpoint.sub(0, 0, dcmPositionController.getComHeightConstant());
      icpVelocitySetpoint.setIncludingFrame(dcmVelocitySetpoint);

      // apply command mask
      if (commandMask.getX() == 0)
      {
         comForceCommand.setX(0);
      }
      if (commandMask.getY() == 0)
      {
         comForceCommand.setY(0);
      }
      if (commandMask.getZ() == 0)
      {
         comForceCommand.setZ(0);
      }
   }
}
