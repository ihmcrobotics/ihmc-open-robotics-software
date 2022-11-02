package us.ihmc.avatar;

import us.ihmc.avatar.stepAdjustment.PlanarRegionFootstepSnapper;
import us.ihmc.avatar.stepAdjustment.SimpleSteppableRegionsCalculator;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.StepGeneratorCommandInputManager;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
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
   private static final int numberOfRegionsToVisualize = 3;
   private static final int maximumVertices = 40;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   private final YoBoolean shouldSnapToRegions;
   private final YoBoolean shouldVisualizeSnappedRegions;
   private final YoInteger numberOfSteppableRegions;

   private final SteppingParameters steppingParameters;

   private final PlanarRegionFootstepSnapper stepSnapper;
   private final List<FootstepValidityIndicator> footstepValidityIndicators = new ArrayList<>();
   //   private final BipedalSupportPlanarRegionCalculator supportPlanarRegionCalculator;

   private final BoundingBoxCollisionDetector collisionDetector;

   private final SimpleSteppableRegionsCalculator steppableRegionsCalculator = new SimpleSteppableRegionsCalculator();

   private final RecyclingArrayList<PlanarRegion> regionsSnapped = new RecyclingArrayList<>(PlanarRegion::new);
   private final YoFrameConvexPolygon2D[] concaveRegionHulls = new YoFrameConvexPolygon2D[numberOfRegionsToVisualize];
   private final YoFramePose3D[] concaveRegionPoses = new YoFramePose3D[numberOfRegionsToVisualize];

   // temp variables
   private final ConvexPolygon2D footPolygon = new ConvexPolygon2D();
   private final RigidBodyTransform snapTransform = new RigidBodyTransform();
   private final PlanarRegion tempRegion = new PlanarRegion();

   public HumanoidSteppingPluginEnvironmentalConstraints(RobotContactPointParameters<RobotSide> contactPointParameters, SteppingParameters steppingParameters)
   {
      this.steppingParameters = steppingParameters;

      shouldSnapToRegions = new YoBoolean("shouldSnapToRegions", registry);
      shouldVisualizeSnappedRegions = new YoBoolean("shouldVisualizeSnappedRegions", registry);
      numberOfSteppableRegions = new YoInteger("numberOfSteppableRegions", registry);

      shouldVisualizeSnappedRegions.set(true);

      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(robotSide);
         footPolygons.put(robotSide, new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints)));
      }

      for (int i = 0; i < numberOfRegionsToVisualize; i++)
      {
         concaveRegionHulls[i] = new YoFrameConvexPolygon2D("concaveRegionHull" + i, ReferenceFrame.getWorldFrame(), maximumVertices, registry);
         concaveRegionPoses[i] = new YoFramePose3D("concaveRegionPose" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPolygon graphicPolygon = new YoGraphicPolygon("concaveRegionHull" + i,
                                                                concaveRegionHulls[i],
                                                                concaveRegionPoses[i],
                                                                1.0,
                                                                YoAppearance.Blue());

         graphicsListRegistry.registerYoGraphic("Environmental Constraints", graphicPolygon);
      }

      stepSnapper = new PlanarRegionFootstepSnapper(footPolygons, steppableRegionsCalculator)
      {
         @Override
         public boolean adjustFootstep(FramePose3DReadOnly stanceFootPose,
                                       FramePose2DReadOnly footstepPose,
                                       RobotSide footSide,
                                       FixedFramePose3DBasics adjustedPoseToPack)
         {
            if (!shouldSnapToRegions.getValue())
               return true;

            boolean snapped = super.adjustFootstep(stanceFootPose, footstepPose, footSide, adjustedPoseToPack);

            // stash these for visualization purposes.
            if (shouldSnapToRegions.getValue() && regionsSnapped.size() < numberOfRegionsToVisualize)
            {
               for (int i = 0; i < regionsSnapped.size(); i++)
               {
                  if (regionsSnapped.get(i).epsilonEquals(regionToSnapTo, 1e-2))
                     return snapped;
               }

               regionsSnapped.add().set(regionToSnapTo);

               int index = regionsSnapped.size() - 1;
               concaveRegionHulls[index].clear();
               for (int i = 0; i < Math.min(maximumVertices, regionToSnapTo.getConvexHull().getNumberOfVertices()); i++)
                  concaveRegionHulls[index].addVertex(regionToSnapTo.getConcaveHullVertex(i));
               concaveRegionHulls[index].update();
               concaveRegionHulls[index].applyTransform(regionToSnapTo.getTransformToLocal(), false);

               concaveRegionPoses[index].set(regionToSnapTo.getTransformToWorld());
            }

            return snapped;
         }
      };

      double collisionBoxDepth = 0.65;
      double collisionBoxWidth = 1.15;
      double collisionBoxHeight = 1.0;
      collisionDetector = new BoundingBoxCollisionDetector();
      collisionDetector.setBoxDimensions(collisionBoxDepth, collisionBoxWidth, collisionBoxHeight);

      footstepValidityIndicators.add(this::isStepSnappable);
      footstepValidityIndicators.add(this::isSafeStepHeight);
      //      footstepValidityIndicators.add(this::isSafeDistanceFromObstacle);

      registry.addChild(stepSnapper.getRegistry());
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
      for (int i = 0; i < numberOfRegionsToVisualize; i++)
      {
         concaveRegionHulls[i].clear();
         concaveRegionPoses[i].setToNaN();
      }

      regionsSnapped.clear();
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

   public FootstepAdjustment getFootstepAdjustment()
   {
      return stepSnapper;
   }

   public List<FootstepValidityIndicator> getFootstepValidityIndicators()
   {
      return footstepValidityIndicators;
   }

   private boolean isStepSnappable(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (steppableRegionsCalculator.getSteppableRegions().isEmpty() || !shouldSnapToRegions.getValue())
         return true;

      footPolygon.set(stepSnapper.getFootPolygon(swingSide));
      footPolygon.applyTransform(touchdownPose, false);

      return stepSnapper.getSnapper()
                        .snapPolygonToPlanarRegionsList(footPolygon,
                                                        steppableRegionsCalculator.getSteppableRegions(),
                                                        Double.POSITIVE_INFINITY,
                                                        tempRegion,
                                                        snapTransform);
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
