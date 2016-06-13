package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.robotics.controllers.EuclideanPositionController;
import us.ihmc.robotics.controllers.YoEuclideanPositionGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedSolePositionController
{
   public static class Setpoints
   {
      private final QuadrantDependentList<FramePoint> solePosition = new QuadrantDependentList<>();
      private final QuadrantDependentList<FrameVector> soleLinearVelocity = new QuadrantDependentList<>();
      private final QuadrantDependentList<FrameVector> soleForceFeedforward = new QuadrantDependentList<>();

      public Setpoints()
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            solePosition.set(robotQuadrant, new FramePoint());
            soleLinearVelocity.set(robotQuadrant, new FrameVector());
            soleForceFeedforward.set(robotQuadrant, new FrameVector());
         }
      }

      public void initialize(QuadrupedTaskSpaceEstimator.Estimates estimates)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            solePosition.get(robotQuadrant).setIncludingFrame(estimates.getSolePosition(robotQuadrant));
            solePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
            soleLinearVelocity.get(robotQuadrant).setToZero();
            soleForceFeedforward.get(robotQuadrant).setToZero();
         }
      }

      public FramePoint getSolePosition(RobotQuadrant robotQuadrant)
      {
         return solePosition.get(robotQuadrant);
      }

      public FrameVector getSoleLinearVelocity(RobotQuadrant robotQuadrant)
      {
         return soleLinearVelocity.get(robotQuadrant);
      }

      public FrameVector getSoleForceFeedforward(RobotQuadrant robotQuadrant)
      {
         return soleForceFeedforward.get(robotQuadrant);
      }
   }

   private final QuadrantDependentList<ReferenceFrame> soleFrame;
   private final QuadrantDependentList<EuclideanPositionController> solePositionController;
   private final QuadrantDependentList<YoEuclideanPositionGains> solePositionControllerGains;
   private final QuadrantDependentList<YoFramePoint> yoSolePositionSetpoint;
   private final QuadrantDependentList<YoFrameVector> yoSoleLinearVelocitySetpoint;
   private final QuadrantDependentList<YoFrameVector> yoSoleForceFeedforwardSetpoint;

   public QuadrupedSolePositionController(QuadrantDependentList<ReferenceFrame> soleFrame, double controlDT, YoVariableRegistry registry)
   {
      this.soleFrame = soleFrame;
      this.solePositionController = new QuadrantDependentList<>();
      this.solePositionControllerGains = new QuadrantDependentList<>();
      this.yoSolePositionSetpoint = new QuadrantDependentList<>();
      this.yoSoleLinearVelocitySetpoint = new QuadrantDependentList<>();
      this.yoSoleForceFeedforwardSetpoint = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         solePositionController.set(robotQuadrant, new EuclideanPositionController(prefix + "SolePosition", soleFrame.get(robotQuadrant), controlDT, registry));
         solePositionControllerGains.set(robotQuadrant, new YoEuclideanPositionGains(prefix + "SolePosition", registry));
         yoSolePositionSetpoint.set(robotQuadrant, new YoFramePoint(prefix + "SolePositionSetpoint", ReferenceFrame.getWorldFrame(), registry));
         yoSoleLinearVelocitySetpoint.set(robotQuadrant, new YoFrameVector(prefix + "SoleLinearVelocitySetpoint", ReferenceFrame.getWorldFrame(), registry));
         yoSoleForceFeedforwardSetpoint.set(robotQuadrant, new YoFrameVector(prefix + "SoleForceFeedforwardSetpoint", ReferenceFrame.getWorldFrame(), registry));
      }
   }

   public QuadrantDependentList<YoEuclideanPositionGains> getGains()
   {
      return solePositionControllerGains;
   }

   public YoEuclideanPositionGains getGains(RobotQuadrant robotQuadrant)
   {
      return solePositionControllerGains.get(robotQuadrant);
   }

   public void reset()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionController.get(robotQuadrant).reset();
      }
   }

   public void compute(QuadrantDependentList<FrameVector> soleForceCommand, Setpoints setpoints, QuadrupedTaskSpaceEstimator.Estimates estimates)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         FramePoint solePositionSetpoint = setpoints.getSolePosition(robotQuadrant);
         FrameVector soleLinearVelocitySetpoint = setpoints.getSoleLinearVelocity(robotQuadrant);
         FrameVector soleLinearVelocityEstimate = estimates.getSoleLinearVelocity(robotQuadrant);
         FrameVector soleForceFeedforwardSetpoint = setpoints.getSoleForceFeedforward(robotQuadrant);

         ReferenceFrame solePositionSetpointFrame = solePositionSetpoint.getReferenceFrame();
         ReferenceFrame soleLinearVelocitySetpointFrame = soleLinearVelocitySetpoint.getReferenceFrame();
         ReferenceFrame soleLinearVelocityEstimateFrame = soleLinearVelocityEstimate.getReferenceFrame();
         ReferenceFrame soleForceFeedforwardSetpointFrame = soleForceFeedforwardSetpoint.getReferenceFrame();

         // compute sole force
         soleForceCommand.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         solePositionSetpoint.changeFrame(soleFrame.get(robotQuadrant));
         soleLinearVelocitySetpoint.changeFrame(soleFrame.get(robotQuadrant));
         soleLinearVelocityEstimate.changeFrame(soleFrame.get(robotQuadrant));
         soleForceFeedforwardSetpoint.changeFrame(soleFrame.get(robotQuadrant));
         solePositionController.get(robotQuadrant).setGains(solePositionControllerGains.get(robotQuadrant));
         solePositionController.get(robotQuadrant).compute(soleForceCommand.get(robotQuadrant), solePositionSetpoint, soleLinearVelocitySetpoint, soleLinearVelocityEstimate, soleForceFeedforwardSetpoint);

         // update log variables
         yoSolePositionSetpoint.get(robotQuadrant).setAndMatchFrame(solePositionSetpoint);
         yoSoleLinearVelocitySetpoint.get(robotQuadrant).setAndMatchFrame(soleLinearVelocitySetpoint);
         yoSoleForceFeedforwardSetpoint.get(robotQuadrant).setAndMatchFrame(soleForceFeedforwardSetpoint);

         solePositionSetpoint.changeFrame(solePositionSetpointFrame);
         soleLinearVelocitySetpoint.changeFrame(soleLinearVelocitySetpointFrame);
         soleLinearVelocityEstimate.changeFrame(soleLinearVelocityEstimateFrame);
         soleForceFeedforwardSetpoint.changeFrame(soleForceFeedforwardSetpointFrame);
      }
   }
}