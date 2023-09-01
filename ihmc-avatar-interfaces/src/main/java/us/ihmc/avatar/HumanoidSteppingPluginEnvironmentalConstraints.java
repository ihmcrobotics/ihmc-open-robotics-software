package us.ihmc.avatar;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.stepAdjustment.PlanarRegionFootstepPlanSnapper;
import us.ihmc.avatar.stepAdjustment.PlanarRegionSnapVisualizer;
import us.ihmc.avatar.stepAdjustment.SimpleSteppableRegionsCalculator;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.ConstraintOptimizerParametersReadOnly;
import us.ihmc.commonWalkingControlModules.configurations.SteppingEnvironmentalConstraintParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.YoSteppingEnvironmentalConstraintParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepPlanAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.StepGeneratorCommandInputManager;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.footstepPlanning.simplePlanners.SnapAndWiggleSingleStepParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
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
 * This is designed to work alongside the plugins. It must be added as a PlanarRegionsList Consumer to the {@link StepGeneratorCommandInputManager} and as an
 * Updatable to clear the graphics to the
 * {@link us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HumanoidSteppingPluginFactory#addUpdatable(Updatable)}
 */
public class HumanoidSteppingPluginEnvironmentalConstraints implements Consumer<PlanarRegionsListCommand>, Updatable
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   private final YoBoolean shouldSnapToRegions;
   private final YoBoolean checkFootHeightInWorld;
   private final YoBoolean checkFootHeightDelta;
   private final YoBoolean checkStepLengthIsStupid;
   private final YoInteger numberOfSteppableRegions;

   private final SteppingParameters steppingParameters;
   private final SteppingEnvironmentalConstraintParameters environmentalConstraintParameters;

   private final PlanarRegionFootstepPlanSnapper stepSnapper;
   private final List<FootstepValidityIndicator> footstepValidityIndicators = new ArrayList<>();

   private final BoundingBoxCollisionDetector collisionDetector;

   private final SimpleSteppableRegionsCalculator steppableRegionsCalculator;

   private final PlanarRegionSnapVisualizer snapVisualizer;

   public HumanoidSteppingPluginEnvironmentalConstraints(RobotContactPointParameters<RobotSide> contactPointParameters,
                                                         SteppingParameters steppingParameters,
                                                         SteppingEnvironmentalConstraintParameters environmentalConstraintParameters)
   {
      this.steppingParameters = steppingParameters;
      this.environmentalConstraintParameters = new YoSteppingEnvironmentalConstraintParameters(environmentalConstraintParameters, registry);

      steppableRegionsCalculator = new SimpleSteppableRegionsCalculator(this.environmentalConstraintParameters::getMinPlanarRegionAreaForStepping,
                                                                        this.environmentalConstraintParameters::getMaxPlanarRegionNormalAngleForStepping);

      shouldSnapToRegions = new YoBoolean("shouldSnapToRegions", registry);
      checkFootHeightInWorld = new YoBoolean("checkFootHeightInWorld", registry);
      checkFootHeightDelta = new YoBoolean("checkFootHeightDelta", registry);
      checkStepLengthIsStupid = new YoBoolean("checkStepLengthIsStupid", registry);
      numberOfSteppableRegions = new YoInteger("numberOfSteppableRegions", registry);

      checkFootHeightDelta.set(true);
      checkFootHeightInWorld.set(false);
      checkStepLengthIsStupid.set(true);

      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(robotSide);
         footPolygons.put(robotSide, new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints)));
      }

      stepSnapper = new PlanarRegionFootstepPlanSnapper(footPolygons, steppableRegionsCalculator, this.environmentalConstraintParameters, registry)
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
      footstepValidityIndicators.add(this::isFootHeightRight);
      footstepValidityIndicators.add(this::isStepLengthStupid);
      //      footstepValidityIndicators.add(this::isSafeDistanceFromObstacle);
   }

   /**
    * Sets the boolean as to whether to snap the foothold to the planar regions.
    */
   public void setShouldSnapToRegions(boolean shouldSnapToRegions)
   {
      this.shouldSnapToRegions.set(shouldSnapToRegions);
   }

   /**
    * Update called to reset the visualizer
    */
   @Override
   public void update(double timeInState)
   {
      snapVisualizer.reset();
   }

   /**
    * Consume the planar regions that are published to the step genreator.
    * @param planarRegionsListCommand the input argument
    */
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

   /**
    * Returns a snapper that will snap a {@link FootstepDataListMessage} to the environment modeled by planar regions
    */
   public FootstepPlanAdjustment getFootstepPlanAdjustment()
   {
      return stepSnapper;
   }

   /**
    * Returns the list of validity indicators, that check whether or not a footstep is valid.
    */
   public List<FootstepValidityIndicator> getFootstepValidityIndicators()
   {
      return footstepValidityIndicators;
   }

   private boolean isSafeStepHeight(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (!checkFootHeightDelta.getBooleanValue())
         return true;

      double heightChange = touchdownPose.getZ() - stancePose.getZ();
      return heightChange < steppingParameters.getMaxStepUp() && heightChange > -steppingParameters.getMaxStepDown();
   }

   private boolean isStepLengthStupid(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (!checkStepLengthIsStupid.getValue())
         return true;

      return touchdownPose.getPosition().distanceXY(stancePose.getPosition()) < 1.2 * steppingParameters.getMaxStepLength();
   }

   private final FramePoint3D pointInRegion = new FramePoint3D();

   private boolean isFootHeightRight(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (!checkFootHeightInWorld.getValue() || !shouldSnapToRegions.getValue())
         return true;

      double heightAtPoint = Double.MIN_VALUE;
      for (int i = 0; i < steppableRegionsCalculator.getSteppableRegions().size(); i++)
      {
         pointInRegion.set(touchdownPose.getPosition());
         PlanarRegion region = steppableRegionsCalculator.getSteppableRegions().get(i);
         region.transformFromWorldToLocal(pointInRegion);

         if (region.isPointInside(pointInRegion.getX(), pointInRegion.getY()))
            heightAtPoint = Math.max(steppableRegionsCalculator.getSteppableRegions().get(i).getPlaneZGivenXY(touchdownPose.getX(), touchdownPose.getY()), heightAtPoint);
      }

      if (!Double.isFinite(heightAtPoint))
         heightAtPoint = stancePose.getZ();

      return MathTools.epsilonEquals(heightAtPoint, touchdownPose.getZ(), 3e-2);
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

      return collisionDetector.checkForCollision() == null;
   }
}
