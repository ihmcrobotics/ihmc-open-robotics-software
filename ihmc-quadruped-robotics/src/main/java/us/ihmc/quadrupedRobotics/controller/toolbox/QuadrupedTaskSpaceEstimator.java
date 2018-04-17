package us.ihmc.quadrupedRobotics.controller.toolbox;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTaskSpaceEstimator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final QuadrupedReferenceFrames referenceFrames;
   private final QuadrantDependentList<MovingReferenceFrame> soleFrames;

   // estimates
   private final QuadrantDependentList<YoFramePoint3D> yoSolePositionEstimate;

   // solvers

   private final YoVariableRegistry registry = new YoVariableRegistry("taskSpaceEstimator");

   public QuadrupedTaskSpaceEstimator(FullQuadrupedRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry graphicsListRegistry)
   {
      this.referenceFrames = referenceFrames;
      soleFrames = referenceFrames.getFootReferenceFrames();

      // estimates
      yoSolePositionEstimate = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseName();
         yoSolePositionEstimate.set(robotQuadrant, new YoFramePoint3D(prefix + "SolePositionEstimate", worldFrame, registry));
      }

      // graphics
      if (graphicsListRegistry != null)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            String prefix = robotQuadrant.getCamelCaseName();
            YoFramePoint3D yoSolePosition = yoSolePositionEstimate.get(robotQuadrant);
            YoGraphicPosition yoSolePositionGraphic = new YoGraphicPosition(prefix + "SolePositionEstimate", yoSolePosition, 0.01, YoAppearance.Black());
            graphicsListRegistry.registerArtifact(prefix + "SolePositionEstimate", yoSolePositionGraphic.createArtifact());
         }
      }

      parentRegistry.addChild(registry);
   }

   public void compute(QuadrupedTaskSpaceEstimates estimates)
   {
      // update solvers
      referenceFrames.updateFrames();

      // compute sole poses, twists, and forces
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         estimates.getSolePosition(robotQuadrant).setToZero(soleFrames.get(robotQuadrant));
      }

      // update yovariables
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSolePositionEstimate.get(robotQuadrant).setMatchingFrame(estimates.getSolePosition(robotQuadrant));
      }
   }
}
