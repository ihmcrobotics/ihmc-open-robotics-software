package us.ihmc.avatar;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.stepAdjustment.HeightMapFootstepSnapper;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepAdjustment;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepValidityIndicator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.HumanoidSteppingPluginFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.StepGeneratorCommandInputManager;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HeightMapCommand;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/**
 * This is designed to work alongside the plugins. It must be added as a Height Map Consumer to the {@link StepGeneratorCommandInputManager} and as an
 * Updatable to clear the graphics to the
 * {@link HumanoidSteppingPluginFactory#addUpdatable(Updatable)}
 */
public class HumanoidSteppingPluginEnvironmentalConstraints implements Consumer<HeightMapCommand>, Updatable, SCS2YoGraphicHolder
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean checkStepHeight;
   private final YoBoolean checkStepLength;

   private final SteppingParameters steppingParameters;
   private final HeightMapFootstepSnapper stepSnapper;
   private final List<FootstepValidityIndicator> footstepValidityIndicators = new ArrayList<>();

   public HumanoidSteppingPluginEnvironmentalConstraints(RobotContactPointParameters<RobotSide> contactPointParameters, SteppingParameters steppingParameters)
   {
      this.steppingParameters = steppingParameters;

      checkStepHeight = new YoBoolean("checkStepHeight", registry);
      checkStepLength = new YoBoolean("checkStepLength", registry);

      checkStepHeight.set(true);
      checkStepLength.set(true);

      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(robotSide);
         footPolygons.put(robotSide, new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints)));
      }

      stepSnapper = new HeightMapFootstepSnapper(footPolygons, registry);

      footstepValidityIndicators.add(this::isSafeStepHeight);
      footstepValidityIndicators.add(this::isSafeStepLength);
   }

   /**
    * Whether to snap the foothold to the height map, if false the height of the stance foot is used.
    */
   public void setSnapToHeightMap(boolean snapToHeightMap)
   {
      stepSnapper.setSnapToHeightMap(snapToHeightMap);
   }

   /**
    * Update called to reset the visualizer
    */
   @Override
   public void update(double timeInState)
   {

   }

   /**
    * Consume the height maps that are published to the step genreator.
    *
    * @param heightMapCommand the input argument
    */
   @Override
   public void accept(HeightMapCommand heightMapCommand)
   {
      stepSnapper.setHeightMap(heightMapCommand);
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   /**
    * Returns a snapper that will snap a {@link FootstepDataListMessage} to the environment modeled by planar regions
    */
   public FootstepAdjustment getFootstepAdjustment()
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
      if (!checkStepHeight.getBooleanValue())
         return true;

      double heightChange = touchdownPose.getZ() - stancePose.getZ();
      return heightChange < steppingParameters.getMaxStepUp() && heightChange > -steppingParameters.getMaxStepDown();
   }

   private boolean isSafeStepLength(FramePose3DReadOnly touchdownPose, FramePose3DReadOnly stancePose, RobotSide swingSide)
   {
      if (!checkStepLength.getValue())
         return true;

      // The 10% here is a "fudge factor". This is meant to reject steps that are wildly wrong.
      return touchdownPose.getPosition().distanceXY(stancePose.getPosition()) < 1.1 * EuclidCoreTools.norm(steppingParameters.getMaxStepLength(), steppingParameters.getMaxStepWidth());
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}
