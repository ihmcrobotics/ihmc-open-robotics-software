package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SwingTrajectoryCalculator
{
   private final TwoWaypointSwingGenerator swingTrajectoryOptimizer;
   private final ReferenceFrame oppositeSoleZUpFrame;


   public SwingTrajectoryCalculator(String namePrefix, RobotSide robotSide,
                                    HighLevelHumanoidControllerToolbox controllerToolbox,
                                    WalkingControllerParameters walkingControllerParameters, YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double maxSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getMaxSwingHeightFromStanceFoot();
      double minSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getMinSwingHeightFromStanceFoot();
      double defaultSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getDefaultSwingHeightFromStanceFoot();

      oppositeSoleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(robotSide.getOppositeSide());

      swingTrajectoryOptimizer = new TwoWaypointSwingGenerator(namePrefix, minSwingHeightFromStanceFoot, maxSwingHeightFromStanceFoot,
                                                               defaultSwingHeightFromStanceFoot, registry, yoGraphicsListRegistry);
      double minDistanceToStance = walkingControllerParameters.getMinSwingTrajectoryClearanceFromStanceFoot();
      swingTrajectoryOptimizer.enableStanceCollisionAvoidance(robotSide, oppositeSoleZUpFrame, minDistanceToStance);
   }

   public void informDone()
   {
      swingTrajectoryOptimizer.informDone();
   }

   public void doOptimizationUpdate()
   {
      swingTrajectoryOptimizer.doOptimizationUpdate();
   }

   private void initializeOptimizer()
   {
      swingTrajectoryOptimizer.setInitialConditions(initialPosition, initialLinearVelocity);
      swingTrajectoryOptimizer.setFinalConditions(finalPosition, finalLinearVelocity);
      swingTrajectoryOptimizer.setFinalConditionWeights(null, touchdownVelocityWeight);
      swingTrajectoryOptimizer.setStepTime(swingDuration.getDoubleValue());
      swingTrajectoryOptimizer.setTrajectoryType(activeTrajectoryType.getEnumValue(), positionWaypointsForSole);
      swingTrajectoryOptimizer.setSwingHeight(swingHeight.getDoubleValue());
      swingTrajectoryOptimizer.setStanceFootPosition(stanceFootPosition);
      swingTrajectoryOptimizer.setWaypointProportions(waypointProportions);
      swingTrajectoryOptimizer.initialize();
   }
}
