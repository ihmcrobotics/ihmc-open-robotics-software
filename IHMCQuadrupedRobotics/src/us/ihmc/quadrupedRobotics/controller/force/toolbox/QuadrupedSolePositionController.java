package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.controllers.EuclideanPositionController;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedSolePositionController
{
   public static class Setpoints
   {
      private RobotQuadrant robotQuadrant;
      private final FramePoint3D solePosition;
      private final FrameVector3D soleLinearVelocity;
      private final FrameVector3D soleForceFeedforward;

      public Setpoints(RobotQuadrant robotQuadrant)
      {
         this.robotQuadrant = robotQuadrant;
         this.solePosition = new FramePoint3D();
         this.soleLinearVelocity = new FrameVector3D();
         this.soleForceFeedforward = new FrameVector3D();
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

      public FramePoint3D getSolePosition()
      {
         return solePosition;
      }

      public FrameVector3D getSoleLinearVelocity()
      {
         return soleLinearVelocity;
      }

      public FrameVector3D getSoleForceFeedforward()
      {
         return soleForceFeedforward;
      }
   }

   private final RobotQuadrant robotQuadrant;
   private final ReferenceFrame soleFrame;
   private final EuclideanPositionController solePositionController;
   private final YoPID3DGains solePositionControllerGains;
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
      this.solePositionControllerGains = new DefaultYoPID3DGains(prefix + "SolePosition", GainCoupling.NONE, true, registry);
      this.yoSolePositionSetpoint = new YoFramePoint(prefix + "SolePositionSetpoint", worldFrame, registry);
      this.yoSoleLinearVelocitySetpoint = new YoFrameVector(prefix + "SoleLinearVelocitySetpoint", worldFrame, registry);
      this.yoSoleForceFeedforwardSetpoint = new YoFrameVector(prefix + "SoleForceFeedforwardSetpoint", worldFrame, registry);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return soleFrame;
   }

   public YoPID3DGains getGains()
   {
      return solePositionControllerGains;
   }

   public void reset()
   {
      solePositionController.reset();
      solePositionController.resetIntegrator();
   }

   public void compute(FrameVector3D soleForceCommand, Setpoints setpoints, QuadrupedTaskSpaceEstimator.Estimates estimates)
   {
      FramePoint3D solePositionSetpoint = setpoints.getSolePosition();
      FrameVector3D soleLinearVelocitySetpoint = setpoints.getSoleLinearVelocity();
      FrameVector3D soleLinearVelocityEstimate = estimates.getSoleLinearVelocity(robotQuadrant);
      FrameVector3D soleForceFeedforwardSetpoint = setpoints.getSoleForceFeedforward();

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