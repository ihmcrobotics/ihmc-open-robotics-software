package us.ihmc.avatar;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.stepAdjustment.PlanarRegionFootstepPlanSnapper;
import us.ihmc.avatar.stepAdjustment.PlanarRegionSnapVisualizer;
import us.ihmc.avatar.stepAdjustment.SimpleSteppableRegionsCalculator;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepPlanAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.StepGeneratorCommandInputManager;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStepParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/**
 * This is designed to work along side the plugins. It must be added as a PlanarRegionsList Consumer to the {@link StepGeneratorCommandInputManager} and as an
 * Updatable to clear the graphics to the
 * {@link us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HumanoidSteppingPluginFactory#addUpdatable(Updatable)}
 */
public class HumanoidSteppingPluginEnvironmentalConstraints implements Consumer<PlanarRegionsListCommand>, Updatable
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   private final YoBoolean shouldSnapToRegions;
   private final YoInteger numberOfSteppableRegions;

   private final SteppingParameters steppingParameters;

   private final PlanarRegionFootstepPlanSnapper stepSnapper;
   private final List<FootstepValidityIndicator> footstepValidityIndicators = new ArrayList<>();

   private final BoundingBoxCollisionDetector collisionDetector;

   private final SimpleSteppableRegionsCalculator steppableRegionsCalculator;

   // temp variables
   private final PlanarRegionSnapVisualizer snapVisualizer;

   public HumanoidSteppingPluginEnvironmentalConstraints(RobotContactPointParameters<RobotSide> contactPointParameters,
                                                         SteppingParameters steppingParameters,
                                                         SnapAndWiggleSingleStepParameters snapAndWiggleSingleStepParameters)
   {
      this.steppingParameters = steppingParameters;

      steppableRegionsCalculator = new SimpleSteppableRegionsCalculator(snapAndWiggleSingleStepParameters);

      shouldSnapToRegions = new YoBoolean("shouldSnapToRegions", registry);
      numberOfSteppableRegions = new YoInteger("numberOfSteppableRegions", registry);

      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(robotSide);
         footPolygons.put(robotSide, new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints)));
      }

      stepSnapper = new PlanarRegionFootstepPlanSnapper(footPolygons, steppableRegionsCalculator, registry)
      {
         @Override
         public void adjustFootstepPlan(FramePose3DReadOnly stanceFootPose,
                                       int stepIndexToStart,
                                       FootstepDataListMessage dataListToSnap)
         {
            if (!shouldSnapToRegions.getValue())
               return;

            super.adjustFootstepPlan(stanceFootPose, stepIndexToStart, dataListToSnap);
         }
      };
      snapVisualizer = new PlanarRegionSnapVisualizer(registry, graphicsListRegistry);
      stepSnapper.attachPlanarRegionSnapVisualizer(snapVisualizer);

      double collisionBoxDepth = 0.65;
      double collisionBoxWidth = 1.15;
      double collisionBoxHeight = 1.0;
      collisionDetector = new BoundingBoxCollisionDetector();
      collisionDetector.setBoxDimensions(collisionBoxDepth, collisionBoxWidth, collisionBoxHeight);

//      footstepValidityIndicators.add(this::isStepSnappable);
      footstepValidityIndicators.add(this::isSafeStepHeight);
      //      footstepValidityIndicators.add(this::isSafeDistanceFromObstacle);
   }

   public void setShouldSnapToRegions(boolean shouldSnapToRegions)
   {
      this.shouldSnapToRegions.set(shouldSnapToRegions);
   }

   @Override
   public void update(double timeInState)
   {
      reset();
   }

   public void reset()
   {
      snapVisualizer.reset();
   }

   @Override
   public void accept(PlanarRegionsListCommand planarRegionsListCommand)
   {
      steppableRegionsCalculator.consume(planarRegionsListCommand);
      numberOfSteppableRegions.set(steppableRegionsCalculator.getSteppableRegions().size());
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public YoGraphicsListRegistry getGraphicsListRegistry()
   {
      return graphicsListRegistry;
   }

   public FootstepPlanAdjustment getFootstepPlanAdjustment()
   {
      return stepSnapper;
   }

   public List<FootstepValidityIndicator> getFootstepValidityIndicators()
   {
      return footstepValidityIndicators;
   }

   private boolean isSafeStepHeight(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      double heightChange = touchdownPose.getZ() - stancePose.getZ();
      return heightChange < steppingParameters.getMaxStepUp() && heightChange > -steppingParameters.getMaxStepDown();
   }

   // FIXME this generates a LOT of garbage
   private boolean isSafeDistanceFromObstacle(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      // FIXME should not use the step snapper regions, as those may filter out collisions
      if (steppableRegionsCalculator.getSteppableRegions().isEmpty() || !shouldSnapToRegions.getValue())
         return true;

      double halfStanceWidth = 0.5 * steppingParameters.getInPlaceWidth();

      /** Shift box vertically by max step up, regions below this could be steppable */
      double heightOffset = steppingParameters.getMaxStepUp();

      double soleYaw = touchdownPose.getYaw();
      double lateralOffset = swingSide.negateIfLeftSide(halfStanceWidth);
      double offsetX = -lateralOffset * Math.sin(soleYaw);
      double offsetY = lateralOffset * Math.cos(soleYaw);
      collisionDetector.setBoxPose(touchdownPose.getX() + offsetX, touchdownPose.getY() + offsetY, touchdownPose.getZ() + heightOffset, soleYaw);

      return !collisionDetector.checkForCollision(null);
   }
}
