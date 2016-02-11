package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisPoseProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.StopAllTrajectoryMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.WaypointPositionTrajectoryData;
import us.ihmc.robotics.math.trajectories.providers.YoPositionProvider;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public class PelvisICPBasedTranslationManager
{
   private static final double minTrajectoryTime = 0.1;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable supportPolygonSafeMargin = new DoubleYoVariable("supportPolygonSafeMargin", registry);
   private final DoubleYoVariable frozenOffsetDecayAlpha = new DoubleYoVariable("frozenOffsetDecayAlpha", registry);

   private final YoFramePoint2d desiredPelvisPosition = new YoFramePoint2d("desiredPelvis", worldFrame, registry);

   private final DoubleYoVariable initialPelvisPositionTime = new DoubleYoVariable("initialPelvisPositionTime", registry);
   private final DoubleYoVariable pelvisPositionTrajectoryTime = new DoubleYoVariable("pelvisPositionTrajectoryTime", registry);
   private final YoFramePoint initialPelvisPosition = new YoFramePoint("initialPelvis", worldFrame, registry);
   private final YoFramePoint finalPelvisPosition = new YoFramePoint("finalPelvis", worldFrame, registry);

   private final BooleanYoVariable isUsingWaypointTrajectory;
   private PositionTrajectoryGenerator activeTrajectoryGenerator;
   private final StraightLinePositionTrajectoryGenerator pelvisPositionTrajectoryGenerator;
   private final MultipleWaypointsPositionTrajectoryGenerator waypointPositionTrajectoryGenerator;

   private final YoFrameVector2d pelvisPositionError = new YoFrameVector2d("pelvisPositionError", worldFrame, registry);
   private final DoubleYoVariable proportionalGain = new DoubleYoVariable("pelvisPositionProportionalGain", registry);
   private final YoFrameVector2d proportionalTerm = new YoFrameVector2d("pelvisPositionProportionalTerm", worldFrame, registry);

   private final YoFrameVector2d pelvisPositionCumulatedError = new YoFrameVector2d("pelvisPositionCumulatedError", worldFrame, registry);
   private final DoubleYoVariable integralGain = new DoubleYoVariable("pelvisPositionIntegralGain", registry);
   private final YoFrameVector2d integralTerm = new YoFrameVector2d("pelvisPositionIntegralTerm", worldFrame, registry);
   private final DoubleYoVariable maximumIntegralError = new DoubleYoVariable("maximumPelvisPositionIntegralError", registry);

   private final YoFrameVector2d desiredICPOffset = new YoFrameVector2d("desiredICPOffset", worldFrame, registry);

   private final BooleanYoVariable isEnabled = new BooleanYoVariable("isPelvisTranslationManagerEnabled", registry);
   private final BooleanYoVariable isRunning = new BooleanYoVariable("isPelvisTranslationManagerRunning", registry);
   private final BooleanYoVariable isFrozen = new BooleanYoVariable("isPelvisTranslationManagerFrozen", registry);

   private final BooleanYoVariable manualMode = new BooleanYoVariable("manualModeICPOffset", registry);

   private final DoubleYoVariable yoTime;
   private final double controlDT;

   private final PelvisPoseProvider desiredPelvisPoseProvider;
   private final StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber;
   private final BooleanYoVariable isTrajectoryStopped = new BooleanYoVariable("isPelvisTranslationalTrajectoryStopped", registry);

   private ReferenceFrame supportFrame;
   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final FramePoint tempPosition = new FramePoint();
   private final FrameVector tempVelocity = new FrameVector();
   private final FramePoint2d tempPosition2d = new FramePoint2d();
   private final FrameVector2d tempError2d = new FrameVector2d();
   private final FrameVector2d tempICPOffset = new FrameVector2d();
   private final FrameVector2d icpOffsetForFreezing = new FrameVector2d();

   public PelvisICPBasedTranslationManager(MomentumBasedController momentumBasedController, PelvisPoseProvider desiredPelvisPoseProvider, StopAllTrajectoryMessageSubscriber stopAllTrajectoryMessageSubscriber,
         YoPDGains pelvisXYControlGains, YoVariableRegistry parentRegistry)
   {
      supportPolygonSafeMargin.set(0.04);
      frozenOffsetDecayAlpha.set(0.998);

      yoTime = momentumBasedController.getYoTime();
      controlDT = momentumBasedController.getControlDT();
      pelvisZUpFrame = momentumBasedController.getPelvisZUpFrame();
      midFeetZUpFrame = momentumBasedController.getReferenceFrames().getMidFeetZUpFrame();
      ankleZUpFrames = momentumBasedController.getReferenceFrames().getAnkleZUpReferenceFrames();

      this.desiredPelvisPoseProvider = desiredPelvisPoseProvider;
      this.stopAllTrajectoryMessageSubscriber = stopAllTrajectoryMessageSubscriber;

      isUsingWaypointTrajectory = new BooleanYoVariable(getClass().getSimpleName() + "IsUsingWaypointTrajectory", registry);
      isUsingWaypointTrajectory.set(false);

      DoubleProvider trajectoryTimeProvider = new YoVariableDoubleProvider(pelvisPositionTrajectoryTime);
      PositionProvider initialPositionProvider = new YoPositionProvider(initialPelvisPosition);
      PositionProvider finalPositionProvider = new YoPositionProvider(finalPelvisPosition);
      pelvisPositionTrajectoryGenerator = new StraightLinePositionTrajectoryGenerator("pelvis", worldFrame, trajectoryTimeProvider, initialPositionProvider,
            finalPositionProvider, registry);
      pelvisPositionTrajectoryGenerator.initialize();
      activeTrajectoryGenerator = pelvisPositionTrajectoryGenerator;

      waypointPositionTrajectoryGenerator = new MultipleWaypointsPositionTrajectoryGenerator("pelvisWaypoints", 15, true, worldFrame, registry);

      proportionalGain.set(0.5);
      integralGain.set(1.5);
      maximumIntegralError.set(0.08);

      manualMode.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            initialize();
         }
      });

      parentRegistry.addChild(registry);
   }

   public void clearProvider()
   {
      desiredPelvisPoseProvider.clearPosition();
   }

   public void compute(RobotSide supportLeg, FramePoint2d actualICP)
   {
      if (isFrozen.getBooleanValue())
      {
         icpOffsetForFreezing.scale(frozenOffsetDecayAlpha.getDoubleValue());
         return;
      }

      if (isUsingWaypointTrajectory != null)
      {
         if (isUsingWaypointTrajectory.getBooleanValue())
            activeTrajectoryGenerator = waypointPositionTrajectoryGenerator;
         else
            activeTrajectoryGenerator = pelvisPositionTrajectoryGenerator;
      }

      supportFrame = supportLeg == null ? midFeetZUpFrame : ankleZUpFrames.get(supportLeg);

      if (!isEnabled.getBooleanValue())
      {
         desiredICPOffset.setToZero();
         return;
      }

      if (manualMode.getBooleanValue())
         return;

      handleStopAllTrajectoryMessage();
      updateDesireds();

      if (!isRunning.getBooleanValue())
      {
         desiredICPOffset.setToZero();
         return;
      }

      computeDesiredICPOffset();
   }

   private void updateDesireds()
   {
      if (desiredPelvisPoseProvider != null)
      {
         if (desiredPelvisPoseProvider.checkForHomePosition())
         {
            disable();
            enable();
         }
         else if (desiredPelvisPoseProvider.checkForNewPosition())
         {
            initialPelvisPositionTime.set(yoTime.getDoubleValue());
            if (desiredPelvisPoseProvider.getTrajectoryTime() < minTrajectoryTime)
               pelvisPositionTrajectoryTime.set(minTrajectoryTime);
            else
               pelvisPositionTrajectoryTime.set(desiredPelvisPoseProvider.getTrajectoryTime());
            tempPosition.setToZero(pelvisZUpFrame);
            initialPelvisPosition.setAndMatchFrame(tempPosition);
            finalPelvisPosition.setAndMatchFrame(desiredPelvisPoseProvider.getDesiredPelvisPosition(supportFrame));
            pelvisPositionTrajectoryGenerator.initialize();
            isUsingWaypointTrajectory.set(false);
            isTrajectoryStopped.set(false);
            activeTrajectoryGenerator = pelvisPositionTrajectoryGenerator;
            isRunning.set(true);
         }
         else if (desiredPelvisPoseProvider.checkForNewPositionWithWaypoints())
         {
            initialPelvisPositionTime.set(yoTime.getDoubleValue());
            waypointPositionTrajectoryGenerator.clear();

            tempPosition.setToZero(pelvisZUpFrame);
            tempPosition.changeFrame(worldFrame);
            tempVelocity.setToZero(worldFrame);

            WaypointPositionTrajectoryData desiredPelvisPositionWithWaypoints = desiredPelvisPoseProvider.getDesiredPelvisPositionWithWaypoints();
            desiredPelvisPositionWithWaypoints.changeFrame(worldFrame);
            waypointPositionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
            waypointPositionTrajectoryGenerator.appendWaypoints(desiredPelvisPositionWithWaypoints);

            isUsingWaypointTrajectory.set(true);
            isTrajectoryStopped.set(false);
            activeTrajectoryGenerator = waypointPositionTrajectoryGenerator;
            isRunning.set(true);
         }
      }

      if (isRunning.getBooleanValue())
      {
         if (!isTrajectoryStopped.getBooleanValue())
         {
            double deltaTime = yoTime.getDoubleValue() - initialPelvisPositionTime.getDoubleValue();
            activeTrajectoryGenerator.compute(deltaTime);
         }
         activeTrajectoryGenerator.get(tempPosition);
         desiredPelvisPosition.setByProjectionOntoXYPlane(tempPosition);
      }
   }

   public void handlePelvisTrajectoryMessage(PelvisTrajectoryMessage message)
   {
      initialPelvisPositionTime.set(yoTime.getDoubleValue());
      waypointPositionTrajectoryGenerator.clear();
      waypointPositionTrajectoryGenerator.changeFrame(worldFrame);

      if (message.getWaypoint(0).getTime() > 1.0e-5)
      {
         activeTrajectoryGenerator.get(tempPosition);
         tempPosition.changeFrame(worldFrame);
         tempVelocity.setToZero(worldFrame);

         waypointPositionTrajectoryGenerator.appendWaypoint(0.0, tempPosition, tempVelocity);
      }

      waypointPositionTrajectoryGenerator.appendWaypoints(message.getWaypoints());
      waypointPositionTrajectoryGenerator.changeFrame(worldFrame);
      waypointPositionTrajectoryGenerator.initialize();
      isUsingWaypointTrajectory.set(true);
      isTrajectoryStopped.set(false);
      activeTrajectoryGenerator = waypointPositionTrajectoryGenerator;
   }

   private void handleStopAllTrajectoryMessage()
   {
      if (stopAllTrajectoryMessageSubscriber == null || !stopAllTrajectoryMessageSubscriber.pollMessage(this))
         return;

      isTrajectoryStopped.set(true);
   }

   private void computeDesiredICPOffset()
   {
      pelvisPositionError.set(desiredPelvisPosition);
      tempPosition2d.setToZero(pelvisZUpFrame);
      tempPosition2d.changeFrame(worldFrame);
      pelvisPositionError.sub(tempPosition2d);

      pelvisPositionError.getFrameTuple2dIncludingFrame(tempError2d);
      tempError2d.scale(controlDT);
      pelvisPositionCumulatedError.add(tempError2d);

      double cumulativeErrorMagnitude = pelvisPositionCumulatedError.length();
      if (cumulativeErrorMagnitude > maximumIntegralError.getDoubleValue())
      {
         pelvisPositionCumulatedError.scale(maximumIntegralError.getDoubleValue() / cumulativeErrorMagnitude);
      }

      proportionalTerm.set(pelvisPositionError);
      proportionalTerm.scale(proportionalGain.getDoubleValue());

      integralTerm.set(pelvisPositionCumulatedError);
      integralTerm.scale(integralGain.getDoubleValue());

      desiredICPOffset.set(proportionalTerm);
      desiredICPOffset.add(integralTerm);
   }

   private final ConvexPolygonShrinker convexPolygonShrinker = new ConvexPolygonShrinker();
   private final FrameConvexPolygon2d safeSupportPolygonToConstrainICPOffset = new FrameConvexPolygon2d();

   private final FramePoint2d originalICPToModify = new FramePoint2d();

   public void addICPOffset(FramePoint2d desiredICPToModify, FrameVector2d desiredICPVelocityToModify, FrameConvexPolygon2d supportPolygon)
   {
      desiredICPToModify.changeFrame(supportPolygon.getReferenceFrame());
      desiredICPVelocityToModify.changeFrame(supportPolygon.getReferenceFrame());

      originalICPToModify.setIncludingFrame(desiredICPToModify);

      if (!isEnabled.getBooleanValue() || (!isRunning.getBooleanValue() && !manualMode.getBooleanValue()))
      {
         desiredICPOffset.setToZero();
         icpOffsetForFreezing.setToZero();
         desiredICPToModify.changeFrame(worldFrame);
         desiredICPVelocityToModify.changeFrame(worldFrame);
         return;
      }

      if (manualMode.getBooleanValue())
      {
         // Ignore the desiredICPOffset frame assuming the user wants to control the ICP in the supportFrame
         tempICPOffset.setIncludingFrame(supportFrame, desiredICPOffset.getX(), desiredICPOffset.getY());
      }

      else
      {
         desiredICPOffset.getFrameTuple2dIncludingFrame(tempICPOffset);
         tempICPOffset.changeFrame(supportFrame);
      }

      if (isFrozen.getBooleanValue())
      {
         desiredICPOffset.setAndMatchFrame(icpOffsetForFreezing);
         desiredICPToModify.add(icpOffsetForFreezing);
      }

      else
      {
         desiredICPToModify.add(tempICPOffset);

         convexPolygonShrinker.shrinkConstantDistanceInto(supportPolygon, supportPolygonSafeMargin.getDoubleValue(), safeSupportPolygonToConstrainICPOffset);
         safeSupportPolygonToConstrainICPOffset.orthogonalProjection(desiredICPToModify);

         icpOffsetForFreezing.setIncludingFrame(desiredICPToModify);
         icpOffsetForFreezing.sub(originalICPToModify);
      }

      desiredICPToModify.changeFrame(worldFrame);
      desiredICPVelocityToModify.changeFrame(worldFrame);
   }

   public void disable()
   {
      isEnabled.set(false);
      isRunning.set(false);
      isFrozen.set(false);
      isTrajectoryStopped.set(false);

      pelvisPositionError.setToZero();
      pelvisPositionCumulatedError.setToZero();

      proportionalTerm.setToZero();
      integralTerm.setToZero();

      desiredICPOffset.setToZero();
   }

   public void enable()
   {
      if (isEnabled.getBooleanValue())
         return;
      isEnabled.set(true);
      isFrozen.set(false);
      isTrajectoryStopped.set(false);
      initialize();
   }

   public void freeze()
   {
      isFrozen.set(true);
   }

   private void initialize()
   {
      initialPelvisPositionTime.set(yoTime.getDoubleValue());
      pelvisPositionTrajectoryTime.set(0.0);
      tempPosition.setToZero(pelvisZUpFrame);
      initialPelvisPosition.setAndMatchFrame(tempPosition);
      finalPelvisPosition.setAndMatchFrame(tempPosition);
      pelvisPositionTrajectoryGenerator.initialize();
      activeTrajectoryGenerator = pelvisPositionTrajectoryGenerator;
      isTrajectoryStopped.set(false);
   }
}
