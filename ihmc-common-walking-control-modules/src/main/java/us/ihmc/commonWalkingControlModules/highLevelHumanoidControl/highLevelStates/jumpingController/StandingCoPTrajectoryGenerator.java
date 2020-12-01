package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.saveableModule.YoSaveableModule;
import us.ihmc.yoVariables.registry.YoRegistry;

public class StandingCoPTrajectoryGenerator extends YoSaveableModule<JumpingCoPTrajectoryGeneratorState>
{
   private final CoPTrajectoryParameters parameters;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RecyclingArrayList<ContactPlaneProvider> contactStateProviders = new RecyclingArrayList<>(ContactPlaneProvider::new);

   private final FramePoint3D tempFramePoint = new FramePoint3D();
   private final FramePoint3D tempPointForCoPCalculation = new FramePoint3D();

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
      ContactPlaneProvider contactState = contactStateProviders.add();
      contactState.reset();
      contactState.setStartTime(0.0);
      contactState.setStartECMPPosition(state.getInitialCoP());
      for (RobotSide robotSide : RobotSide.values)
         contactState.addContact(state.getFootPose(robotSide), state.getFootPolygonInSole(robotSide));

      ContactPlaneProvider previousContactState = contactState;

      tempPointForCoPCalculation.setIncludingFrame(state.getFootPolygonInSole(RobotSide.LEFT).getCentroid(), 0.0);
      tempPointForCoPCalculation.changeFrame(worldFrame);
      tempFramePoint.setIncludingFrame(state.getFootPolygonInSole(RobotSide.RIGHT).getCentroid(), 0.0);
      tempFramePoint.changeFrame(worldFrame);
      tempPointForCoPCalculation.interpolate(tempFramePoint, 0.5);

      double segmentDuration = parameters.getDefaultFinalTransferSplitFraction() * state.getFinalTransferDuration();
      previousContactState.setEndECMPPosition(tempPointForCoPCalculation);
      previousContactState.setDuration(segmentDuration);
      previousContactState.setLinearECMPVelocity();

      segmentDuration = state.getFinalTransferDuration() - segmentDuration;
      contactState = contactStateProviders.add();
      contactState.reset();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndECMPPosition(tempPointForCoPCalculation);
      contactState.setDuration(segmentDuration);
      contactState.setLinearECMPVelocity();
      for (RobotSide robotSide : RobotSide.values)
         contactState.addContact(state.getFootPose(robotSide), state.getFootPolygonInSole(robotSide));

      previousContactState = contactState;
      contactState = contactStateProviders.add();
      contactState.reset();
      contactState.setStartFromEnd(previousContactState);
      contactState.setEndECMPPosition(previousContactState.getECMPStartPosition());
      contactState.setDuration(Double.POSITIVE_INFINITY);
      contactState.setLinearECMPVelocity();

      for (RobotSide robotSide : RobotSide.values)
         contactState.addContact(state.getFootPose(robotSide), state.getFootPolygonInSole(robotSide));
   }

   public RecyclingArrayList<ContactPlaneProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }
}