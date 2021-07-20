package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.saveableModule.YoSaveableModule;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JumpingCoPTrajectoryGenerator extends YoSaveableModule<JumpingCoPTrajectoryGeneratorState>
{
   private final CoPTrajectoryParameters parameters;
   private final JumpingCoPTrajectoryParameters jumpingParameters;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RecyclingArrayList<ContactPlaneProvider> contactStateProviders = new RecyclingArrayList<>(ContactPlaneProvider::new);

   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final FramePoint3D footMidpoint = new FramePoint3D();

   private final FramePose3D midFootPose = new FramePose3D();
   private final FramePose3D midstancePose = new FramePose3D();
   private final PoseReferenceFrame midstanceFrame = new PoseReferenceFrame("midstanceFrame", worldFrame);
   private final ZUpFrame midstanceZUpFrame = new ZUpFrame(worldFrame, midstanceFrame, "midstanceZUpFrame");

   private final JumpingGoalFootholdCalculator jumpingGoalFootholdCalculator = new JumpingGoalFootholdCalculator();

   private final JumpingParameters regularParameters;
   private final ConvexPolygon2DReadOnly defaultSupportPolygon;

   public JumpingCoPTrajectoryGenerator(CoPTrajectoryParameters parameters,
                                        ConvexPolygon2DReadOnly defaultSupportPolygon,
                                        JumpingCoPTrajectoryParameters jumpingParameters,
                                        JumpingParameters regularParameters,
                                        YoRegistry parentRegistry)
   {
      super(JumpingCoPTrajectoryGenerator.class, parentRegistry);

      this.regularParameters = regularParameters;
      this.defaultSupportPolygon = defaultSupportPolygon;

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

      footMidpoint.setIncludingFrame(state.getFootPolygonInSole(RobotSide.LEFT).getCentroid(), 0.0);
      footMidpoint.changeFrame(worldFrame);
      tempFramePoint.setIncludingFrame(state.getFootPolygonInSole(RobotSide.RIGHT).getCentroid(), 0.0);
      tempFramePoint.changeFrame(worldFrame);
      footMidpoint.interpolate(tempFramePoint, 0.5);

      midstancePose.interpolate(state.getFootPose(RobotSide.LEFT), state.getFootPose(RobotSide.RIGHT), 0.5);
      midstanceFrame.setPoseAndUpdate(midstancePose);
      midstanceZUpFrame.update();

      ContactPlaneProvider contactState = contactStateProviders.add();
      contactState.reset();
      contactState.setStartTime(0.0);
      contactState.setStartECMPPosition(state.getInitialCoP());
      for (RobotSide robotSide : RobotSide.values)
         contactState.addContact(state.getFootPose(robotSide), state.getFootPolygonInSole(robotSide));

      computeForSupport();
      computeForFlight();

      computeForFinalTransfer();
   }

   private void computeForSupport()
   {
      ContactPlaneProvider previousContactState = contactStateProviders.getLast();

      double supportDuration = state.getJumpingGoal().getSupportDuration();
      double segmentDuration = jumpingParameters.getFractionSupportForShift() * supportDuration;
      previousContactState.setEndECMPPosition(footMidpoint);
      previousContactState.setDuration(segmentDuration);
      previousContactState.setLinearECMPVelocity();

      ContactPlaneProvider contactState = contactStateProviders.add();
      contactState.reset();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndECMPPosition(footMidpoint);
      contactState.setDuration(supportDuration - segmentDuration);
      contactState.setLinearECMPVelocity();
      for (RobotSide robotSide : RobotSide.values)
         contactState.addContact(state.getFootPose(robotSide), state.getFootPolygonInSole(robotSide));
   }

   private void computeForFlight()
   {
      double flightDuration = state.getJumpingGoal().getFlightDuration();

      ContactPlaneProvider previousContactState = contactStateProviders.getLast();
      ContactPlaneProvider contactSate = contactStateProviders.add();
      contactSate.setStartTime(previousContactState.getTimeInterval().getEndTime());
      contactSate.setDuration(flightDuration);
      contactSate.setContactState(ContactState.FLIGHT);
   }

   private void computeForFinalTransfer()
   {
      ContactPlaneProvider previousContactState = contactStateProviders.getLast();
      ContactPlaneProvider contactState = contactStateProviders.add();

      JumpingGoalVariable goal = state.getJumpingGoal();
      double goalLength = Double.isNaN(goal.getGoalLength()) ? 0.0 : goal.getGoalLength();
      double width = Double.isNaN(goal.getGoalFootWidth()) ? regularParameters.getDefaultFootWidth() : goal.getGoalFootWidth();
      double goalHeight = Double.isNaN(goal.getGoalHeight()) ? 0.0 : goal.getGoalHeight();
      double goalRotation = Double.isNaN(goal.getGoalRotation()) ? 0.0 : goal.getGoalRotation();

      jumpingGoalFootholdCalculator.computeGoalPose(midstanceZUpFrame, goalLength, width, goalHeight, goalRotation);


      double segmentDuration = parameters.getDefaultFinalTransferSplitFraction() * state.getFinalTransferDuration();
      contactState.reset();
      contactState.setStartECMPPosition(jumpingGoalFootholdCalculator.getGoalPose().getPosition());
      contactState.setEndECMPPosition(jumpingGoalFootholdCalculator.getGoalPose().getPosition());
      contactState.setStartTime(previousContactState.getTimeInterval().getEndTime());
      contactState.setDuration(segmentDuration);
      contactState.setLinearECMPVelocity();
      for (RobotSide robotSide : RobotSide.values)
      {
         contactState.addContact(jumpingGoalFootholdCalculator.getFootGoalPose(robotSide), defaultSupportPolygon);
      }

      previousContactState = contactState;
      segmentDuration = state.getFinalTransferDuration() - segmentDuration;
      contactState = contactStateProviders.add();
      contactState.reset();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndECMPPosition(jumpingGoalFootholdCalculator.getGoalPose().getPosition());
      contactState.setDuration(segmentDuration);
      contactState.setLinearECMPVelocity();
      for (RobotSide robotSide : RobotSide.values)
      {
         contactState.addContact(jumpingGoalFootholdCalculator.getFootGoalPose(robotSide), defaultSupportPolygon);
      }


      previousContactState = contactState;
      contactState = contactStateProviders.add();
      contactState.reset();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndECMPPosition(previousContactState.getECMPStartPosition());
      contactState.setDuration(Double.POSITIVE_INFINITY);
      contactState.setLinearECMPVelocity();
      // TODO contact pose
      for (RobotSide robotSide : RobotSide.values)
      {
         contactState.addContact(jumpingGoalFootholdCalculator.getFootGoalPose(robotSide), defaultSupportPolygon);
      }

   }

   public RecyclingArrayList<ContactPlaneProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }
}