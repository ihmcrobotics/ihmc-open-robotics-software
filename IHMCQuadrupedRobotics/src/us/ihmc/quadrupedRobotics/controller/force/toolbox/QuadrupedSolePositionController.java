package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.robotics.controllers.EuclideanPositionController;
import us.ihmc.robotics.controllers.YoEuclideanPositionGains;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedSolePositionController
{
   public static class Setpoints
   {
      private RobotQuadrant robotQuadrant;
      private final FramePoint solePosition;
      private final FrameVector soleLinearVelocity;
      private final FrameVector soleForceFeedforward;

      public Setpoints(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
         this.solePosition = new FramePoint();
         this.soleLinearVelocity = new FrameVector();
         this.soleForceFeedforward = new FrameVector();
      }

      public void initialize(QuadrupedTaskSpaceEstimator.Estimates estimates)
      {
         this.solePosition.setIncludingFrame(estimates.getSolePosition(robotQuadrant));
         this.solePosition.changeFrame(ReferenceFrame.getWorldFrame());
         this.soleLinearVelocity.setToZero();
         this.soleForceFeedforward.setToZero();
      }

      public RobotQuadrant getRobotQuadrant()
      {
         return robotQuadrant;
      }

      public FramePoint getSolePosition()
      {
         return solePosition;
      }

      public FrameVector getSoleLinearVelocity()
      {
         return soleLinearVelocity;
      }

      public FrameVector getSoleForceFeedforward()
      {
         return soleForceFeedforward;
      }
   }

   private final RobotQuadrant robotQuadrant;
   private final ReferenceFrame soleFrame;
   private final EuclideanPositionController solePositionController;
   private final YoEuclideanPositionGains solePositionControllerGains;
   private final YoFramePoint yoSolePositionSetpoint;
   private final YoFrameVector yoSoleLinearVelocitySetpoint;
   private final YoFrameVector yoSoleForceFeedforwardSetpoint;

   public QuadrupedSolePositionController(RobotQuadrant robotQuadrant, ReferenceFrame soleFrame, double controlDT, YoVariableRegistry registry)
   {
      String prefix = robotQuadrant.getPascalCaseName();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      this.robotQuadrant = robotQuadrant;
      this.soleFrame = soleFrame;
      this.solePositionController = new EuclideanPositionController(prefix + "SolePosition", soleFrame, controlDT, registry);
      this.solePositionControllerGains = new YoEuclideanPositionGains(prefix + "SolePosition", registry);
      this.yoSolePositionSetpoint = new YoFramePoint(prefix + "SolePositionSetpoint", worldFrame, registry);
      this.yoSoleLinearVelocitySetpoint = new YoFrameVector(prefix + "SoleLinearVelocitySetpoint", worldFrame, registry);
      this.yoSoleForceFeedforwardSetpoint = new YoFrameVector(prefix + "SoleForceFeedforwardSetpoint", worldFrame, registry);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return soleFrame;
   }

   public YoEuclideanPositionGains getGains()
   {
      return solePositionControllerGains;
   }

   public void reset()
   {
      solePositionController.reset();
      solePositionController.resetIntegrator();
   }

   public void compute(FrameVector soleForceCommand, Setpoints setpoints, QuadrupedTaskSpaceEstimator.Estimates estimates)
   {
      FramePoint solePositionSetpoint = setpoints.getSolePosition();
      FrameVector soleLinearVelocitySetpoint = setpoints.getSoleLinearVelocity();
      FrameVector soleLinearVelocityEstimate = estimates.getSoleLinearVelocity(robotQuadrant);
      FrameVector soleForceFeedforwardSetpoint = setpoints.getSoleForceFeedforward();

      ReferenceFrame solePositionSetpointFrame = solePositionSetpoint.getReferenceFrame();
      ReferenceFrame soleLinearVelocitySetpointFrame = soleLinearVelocitySetpoint.getReferenceFrame();
      ReferenceFrame soleLinearVelocityEstimateFrame = soleLinearVelocityEstimate.getReferenceFrame();
      ReferenceFrame soleForceFeedforwardSetpointFrame = soleForceFeedforwardSetpoint.getReferenceFrame();

      // compute sole force
      soleForceCommand.setToZero(soleFrame);
      solePositionSetpoint.changeFrame(soleFrame);
      soleLinearVelocitySetpoint.changeFrame(soleFrame);
      soleLinearVelocityEstimate.changeFrame(soleFrame);
      soleForceFeedforwardSetpoint.changeFrame(soleFrame);
      solePositionController.setGains(solePositionControllerGains);
      solePositionController
            .compute(soleForceCommand, solePositionSetpoint, soleLinearVelocitySetpoint, soleLinearVelocityEstimate, soleForceFeedforwardSetpoint);

      // update log variables
      yoSolePositionSetpoint.setAndMatchFrame(solePositionSetpoint);
      yoSoleLinearVelocitySetpoint.setAndMatchFrame(soleLinearVelocitySetpoint);
      yoSoleForceFeedforwardSetpoint.setAndMatchFrame(soleForceFeedforwardSetpoint);

      solePositionSetpoint.changeFrame(solePositionSetpointFrame);
      soleLinearVelocitySetpoint.changeFrame(soleLinearVelocitySetpointFrame);
      soleLinearVelocityEstimate.changeFrame(soleLinearVelocityEstimateFrame);
      soleForceFeedforwardSetpoint.changeFrame(soleForceFeedforwardSetpointFrame);
   }
}