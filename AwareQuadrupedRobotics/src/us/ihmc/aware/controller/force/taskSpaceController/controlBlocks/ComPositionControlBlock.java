package us.ihmc.aware.controller.force.taskSpaceController.controlBlocks;

import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceCommands;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceEstimates;
import us.ihmc.aware.controller.force.taskSpaceController.QuadrupedTaskSpaceControlBlock;
import us.ihmc.robotics.controllers.EuclideanPositionController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Vector3d;

public class ComPositionControlBlock implements QuadrupedTaskSpaceControlBlock
{
   private double mass;
   private double gravity;
   private double controlDT;
   private final ReferenceFrame comZUpFrame;
   private final Vector3d controlAxes;
   private final FramePoint comPositionSetpoint;
   private final FrameVector comVelocitySetpoint;
   private final FrameVector comAccelerationSetpoint;
   private final FrameVector comForceFeedforwardSetpoint;
   private final EuclideanPositionController comPositionController;

   public ComPositionControlBlock(ReferenceFrame comZUpFrame, double controlDT, double mass, double gravity, YoVariableRegistry registry)
   {
      this.mass = mass;
      this.gravity = gravity;
      this.controlDT = controlDT;
      this.comZUpFrame = comZUpFrame;
      controlAxes = new Vector3d(1, 1, 1);
      comPositionSetpoint = new FramePoint();
      comVelocitySetpoint = new FrameVector();
      comAccelerationSetpoint = new FrameVector();
      comForceFeedforwardSetpoint = new FrameVector();
      comPositionController = new EuclideanPositionController("comPosition", comZUpFrame, controlDT, registry);
   }

   public void setComPositionSetpoint(FramePoint comPositionSetpoint)
   {
      this.comPositionSetpoint.setIncludingFrame(comPositionSetpoint);
   }

   public void setComVelocitySetpoint(FrameVector comVelocitySetpoint)
   {
      this.comVelocitySetpoint.setIncludingFrame(comVelocitySetpoint);
   }

   public void setComAccelerationSetpoint(FrameVector comAccelerationSetpoint)
   {
      this.comAccelerationSetpoint.setIncludingFrame(comAccelerationSetpoint);
   }

   public void setControlAxes(int xEnable, int yEnable, int zEnable)
   {
      controlAxes.set(xEnable, yEnable, zEnable);
   }

   public void setProportionalGains(double[] proportionalGains)
   {
      comPositionController.setProportionalGains(proportionalGains);
   }

   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      comPositionController.setIntegralGains(integralGains, maxIntegralError);
   }

   public void setDerivativeGains(double[] derivativeGains)
   {
      comPositionController.setDerivativeGains(derivativeGains);
   }

   @Override public void reset()
   {
      comVelocitySetpoint.setToZero();
      comAccelerationSetpoint.setToZero();
      comPositionController.reset();
   }

   @Override public void compute(QuadrupedTaskSpaceEstimates inputEstimates, QuadrupedTaskSpaceCommands outputCommands)
   {
      FrameVector comVelocityEstimate = inputEstimates.getComVelocity();
      FrameVector comForceCommand = outputCommands.getComForce();

      // compute com force
      comForceCommand.setToZero(comZUpFrame);
      comPositionSetpoint.changeFrame(comZUpFrame);
      comVelocitySetpoint.changeFrame(comZUpFrame);
      comVelocityEstimate.changeFrame(comZUpFrame);
      comForceFeedforwardSetpoint.setIncludingFrame(comAccelerationSetpoint);
      comForceFeedforwardSetpoint.changeFrame(comZUpFrame);
      comForceFeedforwardSetpoint.add(0, 0, gravity);
      comForceFeedforwardSetpoint.scale(mass);
      comPositionController.compute(comForceCommand, comPositionSetpoint, comVelocitySetpoint, comVelocityEstimate, comForceFeedforwardSetpoint);

      // apply command mask
      if (controlAxes.getX() == 0)
      {
         comForceCommand.setX(0);
      }
      if (controlAxes.getY() == 0)
      {
         comForceCommand.setY(0);
      }
      if (controlAxes.getZ() == 0)
      {
         comForceCommand.setZ(0);
      }
   }
}
