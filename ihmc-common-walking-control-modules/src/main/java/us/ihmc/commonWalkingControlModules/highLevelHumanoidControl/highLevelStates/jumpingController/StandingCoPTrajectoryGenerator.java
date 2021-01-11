package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryGeneratorState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.saveableModule.YoSaveableModule;
import us.ihmc.yoVariables.registry.YoRegistry;

public class StandingCoPTrajectoryGenerator extends YoSaveableModule<JumpingCoPTrajectoryGeneratorState>
{
   private final CoPTrajectoryParameters parameters;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private final FramePoint2D tempFramePoint2D = new FramePoint2D();
   private final FramePoint2D tempPointForCoPCalculation = new FramePoint2D();

   public StandingCoPTrajectoryGenerator(CoPTrajectoryParameters parameters, YoRegistry parentRegistry)
   {
      super(StandingCoPTrajectoryGenerator.class, parentRegistry);

      this.parameters = parameters;

      clear();
   }

   @Override
   public void registerState(JumpingCoPTrajectoryGeneratorState state)
   {
      super.registerState(state);
   }

   public void clear()
   {
      contactStateProviders.clear();
   }

   @Override
   public void compute(JumpingCoPTrajectoryGeneratorState state)
   {
      clear();

      // compute cop waypoint location
      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.setStartTime(0.0);
      contactState.setStartCopPosition(state.getInitialCoP());

      SettableContactStateProvider previousContactState = contactState;

      tempPointForCoPCalculation.setIncludingFrame(state.getFootPolygonInSole(RobotSide.LEFT).getCentroid());
      tempPointForCoPCalculation.changeFrameAndProjectToXYPlane(worldFrame);
      tempFramePoint2D.setIncludingFrame(state.getFootPolygonInSole(RobotSide.RIGHT).getCentroid());
      tempFramePoint2D.changeFrameAndProjectToXYPlane(worldFrame);
      tempPointForCoPCalculation.interpolate(tempFramePoint2D, 0.5);

      double segmentDuration = parameters.getDefaultFinalTransferSplitFraction() * state.getFinalTransferDuration();
      previousContactState.setEndCopPosition(tempPointForCoPCalculation);
      previousContactState.setDuration(segmentDuration);

      segmentDuration = state.getFinalTransferDuration() - segmentDuration;
      contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndCopPosition(tempPointForCoPCalculation);
      contactState.setDuration(segmentDuration);

      previousContactState = contactState;
      contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndCopPosition(previousContactState.getCopStartPosition());
      contactState.setDuration(Double.POSITIVE_INFINITY);
   }

   public RecyclingArrayList<SettableContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }
}