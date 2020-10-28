package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.saveableModule.YoSaveableModule;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JumpingCoPTrajectoryGenerator extends YoSaveableModule<JumpingCoPTrajectoryGeneratorState>
{
   private final CoPTrajectoryParameters parameters;
   private final JumpingCoPTrajectoryParameters jumpingParameters;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private final FramePoint2D tempFramePoint2D = new FramePoint2D();
   private final FramePoint2D footMidpoint = new FramePoint2D();

   private final FramePoint2D goalMidpoint = new FramePoint2D();
   private final FramePose3D midstancePose = new FramePose3D();
   private final PoseReferenceFrame midstanceFrame = new PoseReferenceFrame("midstanceFrame", worldFrame);
   private final ZUpFrame midstanceZUpFrame = new ZUpFrame(worldFrame, midstanceFrame, "midstanceZUpFrame");

   public JumpingCoPTrajectoryGenerator(CoPTrajectoryParameters parameters, JumpingCoPTrajectoryParameters jumpingParameters, YoRegistry parentRegistry)
   {
      super(JumpingCoPTrajectoryGenerator.class, parentRegistry);

      this.parameters = parameters;
      this.jumpingParameters = jumpingParameters;

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

      footMidpoint.setIncludingFrame(state.getFootPolygonInSole(RobotSide.LEFT).getCentroid());
      footMidpoint.changeFrameAndProjectToXYPlane(worldFrame);
      tempFramePoint2D.setIncludingFrame(state.getFootPolygonInSole(RobotSide.RIGHT).getCentroid());
      tempFramePoint2D.changeFrameAndProjectToXYPlane(worldFrame);
      footMidpoint.interpolate(tempFramePoint2D, 0.5);

      midstancePose.interpolate(state.getFootPose(RobotSide.LEFT), state.getFootPose(RobotSide.RIGHT), 0.5);
      midstanceFrame.setPoseAndUpdate(midstancePose);
      midstanceZUpFrame.update();

      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.setStartTime(0.0);
      contactState.setStartCopPosition(state.getInitialCoP());

      computeForSupport();
      computeForFlight();

      computeForFinalTransfer();
   }

   private void computeForSupport()
   {
      SettableContactStateProvider previousContactState = contactStateProviders.getLast();

      double supportDuration = state.getJumpingGoal().getSupportDuration();
      double segmentDuration = jumpingParameters.getFractionSupportForShift() * supportDuration;
      previousContactState.setEndCopPosition(footMidpoint);
      previousContactState.setDuration(segmentDuration);

      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndCopPosition(footMidpoint);
      contactState.setDuration(supportDuration - segmentDuration);
   }

   private void computeForFlight()
   {
      double flightDuration = state.getJumpingGoal().getFlightDuration();

      SettableContactStateProvider previousContactState = contactStateProviders.getLast();
      SettableContactStateProvider contactSate = contactStateProviders.add();
      contactSate.setStartTime(previousContactState.getTimeInterval().getEndTime());
      contactSate.setDuration(flightDuration);
      contactSate.setContactState(ContactState.FLIGHT);
   }

   private void computeForFinalTransfer()
   {
      SettableContactStateProvider previousContactState = contactStateProviders.getLast();
      SettableContactStateProvider contactState = contactStateProviders.add();

      double goalLength = state.getJumpingGoal().getGoalLength();
      goalLength = Double.isNaN(goalLength) ? 0.0 : goalLength;

      goalMidpoint.setToZero(midstanceZUpFrame);
      goalMidpoint.setX(goalLength);
      goalMidpoint.changeFrameAndProjectToXYPlane(worldFrame);

      double segmentDuration = parameters.getDefaultFinalTransferSplitFraction() * state.getFinalTransferDuration();
      contactState.setStartCopPosition(goalMidpoint);
      contactState.setEndCopPosition(goalMidpoint);
      contactState.setStartTime(previousContactState.getTimeInterval().getEndTime());
      contactState.setDuration(segmentDuration);

      previousContactState = contactState;
      segmentDuration = state.getFinalTransferDuration() - segmentDuration;
      contactState = contactStateProviders.add();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndCopPosition(goalMidpoint);
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