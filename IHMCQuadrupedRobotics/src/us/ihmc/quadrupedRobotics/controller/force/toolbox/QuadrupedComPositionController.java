package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.robotics.controllers.EuclideanPositionController;
import us.ihmc.robotics.controllers.YoEuclideanPositionGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class QuadrupedComPositionController
{
   public static class Setpoints
   {
      private final FramePoint comPosition = new FramePoint();
      private final FrameVector comVelocity = new FrameVector();
      private final FrameVector comForceFeedforward = new FrameVector();

      public void initialize(QuadrupedTaskSpaceEstimator.Estimates estimates)
      {
         comPosition.setIncludingFrame(estimates.getComPosition());
         comPosition.changeFrame(ReferenceFrame.getWorldFrame());
         comVelocity.setToZero();
         comForceFeedforward.setToZero();
      }

      public FramePoint getComPosition()
      {
         return comPosition;
      }

      public FrameVector getComVelocity()
      {
         return comVelocity;
      }

      public FrameVector getComForceFeedforward()
      {
         return comForceFeedforward;
      }
   }

   private final ReferenceFrame comZUpFrame;
   private final EuclideanPositionController comPositionController;
   private final YoEuclideanPositionGains comPositionControllerGains;
   private final YoFramePoint yoComPositionSetpoint;
   private final YoFrameVector yoComVelocitySetpoint;
   private final YoFrameVector yoComForceFeedforwardSetpoint;

   public QuadrupedComPositionController(ReferenceFrame comZUpFrame, double controlDT, YoVariableRegistry registry)
   {
      this.comZUpFrame = comZUpFrame;
      comPositionController = new EuclideanPositionController("comPosition", comZUpFrame, controlDT, registry);
      comPositionControllerGains = new YoEuclideanPositionGains("comPosition", registry);
      yoComPositionSetpoint = new YoFramePoint("comPositionSetpoint", ReferenceFrame.getWorldFrame(), registry);
      yoComVelocitySetpoint = new YoFrameVector("comVelocitySetpoint", ReferenceFrame.getWorldFrame(), registry);
      yoComForceFeedforwardSetpoint = new YoFrameVector("comForceFeedforwardSetpoint", ReferenceFrame.getWorldFrame(), registry);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return comZUpFrame;
   }

   public YoEuclideanPositionGains getGains()
   {
      return comPositionControllerGains;
   }

   public void reset()
   {
      comPositionController.reset();
      comPositionController.resetIntegrator();
   }

   public void compute(FrameVector comForceCommand, Setpoints setpoints, QuadrupedTaskSpaceEstimator.Estimates estimates)
   {
      FramePoint comPositionSetpoint = setpoints.getComPosition();
      FrameVector comVelocitySetpoint = setpoints.getComVelocity();
      FrameVector comVelocityEstimate = estimates.getComVelocity();
      FrameVector comForceFeedforwardSetpoint = setpoints.getComForceFeedforward();

      ReferenceFrame comPositionSetpointFrame = comPositionSetpoint.getReferenceFrame();
      ReferenceFrame comVelocitySetpointFrame = comVelocitySetpoint.getReferenceFrame();
      ReferenceFrame comVelocityEstimateFrame = comVelocityEstimate.getReferenceFrame();
      ReferenceFrame comForceFeedforwardSetpointFrame = comForceFeedforwardSetpoint.getReferenceFrame();

      // compute com force
      comForceCommand.setToZero(comZUpFrame);
      comPositionSetpoint.changeFrame(comZUpFrame);
      comVelocitySetpoint.changeFrame(comZUpFrame);
      comVelocityEstimate.changeFrame(comZUpFrame);
      comForceFeedforwardSetpoint.changeFrame(comZUpFrame);
      comPositionController.setGains(comPositionControllerGains);
      comPositionController.compute(comForceCommand, comPositionSetpoint, comVelocitySetpoint, comVelocityEstimate, comForceFeedforwardSetpoint);

      // update log variables
      yoComPositionSetpoint.setAndMatchFrame(comPositionSetpoint);
      yoComVelocitySetpoint.setAndMatchFrame(comVelocitySetpoint);
      yoComForceFeedforwardSetpoint.setAndMatchFrame(comForceFeedforwardSetpoint);

      comPositionSetpoint.changeFrame(comPositionSetpointFrame);
      comVelocitySetpoint.changeFrame(comVelocitySetpointFrame);
      comVelocityEstimate.changeFrame(comVelocityEstimateFrame);
      comForceFeedforwardSetpoint.changeFrame(comForceFeedforwardSetpointFrame);
   }
}
