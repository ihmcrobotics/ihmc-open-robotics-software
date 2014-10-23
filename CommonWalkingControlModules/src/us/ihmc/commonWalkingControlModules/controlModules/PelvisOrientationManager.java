package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.OrientationTrajectoryData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAngularAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.packetConsumers.PelvisPoseProvider;
import us.ihmc.utilities.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.utilities.math.geometry.AngleTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.OrientationProvider;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.controllers.YoOrientationPIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFrameQuaternion;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoQuaternionProvider;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;


public class PelvisOrientationManager
{
   private static final double defaultTrajectoryTime = 1.0;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFrameQuaternion desiredPelvisOrientation = new YoFrameQuaternion("desiredPelvis", worldFrame, registry);
   private final YoFrameVector desiredPelvisAngularVelocity = new YoFrameVector("desiredPelvisAngularVelocity", worldFrame, registry);
   private final YoFrameVector desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", worldFrame, registry);

   private final DoubleYoVariable swingPelvisYaw = new DoubleYoVariable("swingPelvisYaw", registry);
   private final DoubleYoVariable swingPelvisYawScale = new DoubleYoVariable("swingPelvisYawScale", registry);

   private final DoubleYoVariable initialPelvisOrientationTime = new DoubleYoVariable("initialPelvisOrientationTime", registry);
   private final YoFrameQuaternion initialPelvisOrientation = new YoFrameQuaternion("initialPelvis", worldFrame, registry);
   private final YoFrameQuaternion finalPelvisOrientation = new YoFrameQuaternion("finalPelvis", worldFrame, registry);
   private final OrientationTrajectoryGenerator pelvisOrientationTrajectoryGenerator;

   private final DoubleYoVariable initialPelvisOrientationOffsetTime = new DoubleYoVariable("initialPelvisOrientationOffsetTime", registry);
   private final YoFrameOrientation desiredPelvisOrientationOffset = new YoFrameOrientation("desiredPelvis", "Offset", worldFrame, registry);

   private final DoubleYoVariable offsetTrajectoryTime = new DoubleYoVariable("offsetTrajectoryTime", registry);
   private final YoFrameQuaternion initialPelvisOrientationOffset = new YoFrameQuaternion("initialPelvis", "Offset", worldFrame, registry);
   private final YoFrameQuaternion finalPelvisOrientationOffset = new YoFrameQuaternion("finalPelvis", "Offset", worldFrame, registry);
   private final OrientationTrajectoryGenerator pelvisOrientationOffsetTrajectoryGenerator;

   private final DoubleYoVariable yoTime;

   private final OrientationTrajectoryData orientationTrajectoryData = new OrientationTrajectoryData();
   private final RootJointAngularAccelerationControlModule rootJointAngularAccelerationControlModule;

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameVector tempAngularAcceleration = new FrameVector();
   private final double[] tempYawPitchRoll = new double[3];

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final ReferenceFrame midFeetZUpFrame;
   private final ReferenceFrame pelvisFrame;

   private final PelvisPoseProvider pelvisPoseProvider;

   public PelvisOrientationManager(WalkingControllerParameters walkingControllerParameters, DoubleProvider trajectoryTimeProvider,
         MomentumBasedController momentumBasedController, PelvisPoseProvider desiredPelvisPoseProvider, YoVariableRegistry parentRegistry)
   {
      yoTime = momentumBasedController.getYoTime();
      CommonHumanoidReferenceFrames referenceFrames = momentumBasedController.getReferenceFrames();
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      pelvisFrame = referenceFrames.getPelvisFrame();
      this.pelvisPoseProvider = desiredPelvisPoseProvider;

      desiredPelvisOrientationOffset.setYawPitchRoll(0.0, walkingControllerParameters.getDefaultDesiredPelvisPitch(), 0.0);

      YoOrientationPIDGains pelvisOrientationControlGains = walkingControllerParameters.createPelvisOrientationControlGains(registry);
      rootJointAngularAccelerationControlModule = new RootJointAngularAccelerationControlModule(momentumBasedController, pelvisOrientationControlGains,
            registry);
      OrientationProvider initialPelvisOrientationProvider = new YoQuaternionProvider(initialPelvisOrientation);
      OrientationProvider finalPelvisOrientationProvider = new YoQuaternionProvider(finalPelvisOrientation);
      pelvisOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvis", worldFrame, trajectoryTimeProvider,
            initialPelvisOrientationProvider, finalPelvisOrientationProvider, registry);

      offsetTrajectoryTime.set(defaultTrajectoryTime);
      OrientationProvider initialPelvisOrientationOffsetProvider = new YoQuaternionProvider(initialPelvisOrientationOffset);
      OrientationProvider finalPelvisOrientationOffsetProvider = new YoQuaternionProvider(finalPelvisOrientationOffset);
      DoubleProvider offsetTrajectoryTimeProvider = new YoVariableDoubleProvider(offsetTrajectoryTime);
      pelvisOrientationOffsetTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvisOffset", worldFrame, offsetTrajectoryTimeProvider,
            initialPelvisOrientationOffsetProvider, finalPelvisOrientationOffsetProvider, registry);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();

      parentRegistry.addChild(registry);
   }

   private void initialize()
   {
      initialPelvisOrientationTime.set(yoTime.getDoubleValue());

      pelvisOrientationTrajectoryGenerator.initialize();
   }

   public void compute()
   {
      updateDesireds();

      rootJointAngularAccelerationControlModule.doControl(orientationTrajectoryData);
   }

   private void updateDesireds()
   {
      double deltaTime = yoTime.getDoubleValue() - initialPelvisOrientationTime.getDoubleValue();
      pelvisOrientationTrajectoryGenerator.compute(deltaTime);
      pelvisOrientationTrajectoryGenerator.packAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      desiredPelvisOrientation.set(tempOrientation);
      desiredPelvisAngularVelocity.set(tempAngularVelocity);
      desiredPelvisAngularAcceleration.set(tempAngularAcceleration);

      if (pelvisPoseProvider != null && pelvisPoseProvider.checkForNewOrientation())
      {
         initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());
         offsetTrajectoryTime.set(pelvisPoseProvider.getTrajectoryTime());
         tempOrientation.setIncludingFrame(pelvisPoseProvider.getDesiredPelvisOrientation());
         initialPelvisOrientationOffset.set(finalPelvisOrientationOffset);
         finalPelvisOrientationOffset.set(tempOrientation);
         pelvisOrientationOffsetTrajectoryGenerator.initialize();
      }

      double deltaTimeOffset = yoTime.getDoubleValue() - initialPelvisOrientationOffsetTime.getDoubleValue();
      pelvisOrientationOffsetTrajectoryGenerator.compute(deltaTimeOffset);
      pelvisOrientationOffsetTrajectoryGenerator.packAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      desiredPelvisOrientationOffset.set(tempOrientation);

      addOffsetToOrientation(desiredPelvisOrientation, desiredPelvisOrientationOffset);
      desiredPelvisAngularVelocity.add(tempAngularVelocity);
      desiredPelvisAngularAcceleration.add(tempAngularAcceleration);

      desiredPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      desiredPelvisAngularVelocity.getFrameTupleIncludingFrame(tempAngularVelocity);
      desiredPelvisAngularAcceleration.getFrameTupleIncludingFrame(tempAngularAcceleration);
      orientationTrajectoryData.set(tempOrientation, tempAngularVelocity, tempAngularAcceleration);
   }

   public void setToHoldCurrent()
   {
      tempOrientation.setToZero(pelvisFrame);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);

      initialize();
   }

   public void setToHoldCurrentDesired()
   {
      desiredPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);

      initialize();
   }

   public void setToZeroInMidFeetZUp()
   {
      tempOrientation.setToZero(midFeetZUpFrame);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);

      initialize();
   }

   public void setToZeroInSupportFoot(RobotSide supportSide)
   {
      ReferenceFrame supportAnkleZUp = ankleZUpFrames.get(supportSide);
      tempOrientation.setToZero(supportAnkleZUp);
      tempOrientation.changeFrame(worldFrame);
      initialPelvisOrientation.set(tempOrientation);
      finalPelvisOrientation.set(tempOrientation);
      desiredPelvisOrientation.set(tempOrientation);

      initialize();
   }

   private final FramePoint upcomingFootstepLocation = new FramePoint();
   private final FrameOrientation upcomingFootstepOrientation = new FrameOrientation();

   public void setWithUpcomingFootstep(Footstep upcomingFootstep, RobotSide upcomingFootstepSide)
   {
      desiredPelvisOrientation.getFrameOrientationIncludingFrame(tempOrientation);
      initialPelvisOrientation.set(tempOrientation);

      upcomingFootstep.getOrientationIncludingFrame(upcomingFootstepOrientation);
      upcomingFootstepOrientation.changeFrame(worldFrame);
      tempOrientation.setToZero(ankleZUpFrames.get(upcomingFootstepSide.getOppositeSide()));
      tempOrientation.changeFrame(worldFrame);

      double finalDesiredPelvisYawAngle = AngleTools.computeAngleAverage(upcomingFootstepOrientation.getYaw(), tempOrientation.getYaw());

      upcomingFootstep.getPositionIncludingFrame(upcomingFootstepLocation);
      upcomingFootstepLocation.changeFrame(ankleZUpFrames.get(upcomingFootstepSide.getOppositeSide()));

      double desiredSwingPelvisYawAngle = 0.0;
      if (Math.abs(upcomingFootstepLocation.getX()) > 0.1)
      {
         desiredSwingPelvisYawAngle = Math.atan2(upcomingFootstepLocation.getY(), upcomingFootstepLocation.getX());
         desiredSwingPelvisYawAngle -= upcomingFootstepSide.negateIfRightSide(Math.PI / 2.0);
      }
      swingPelvisYaw.set(desiredSwingPelvisYawAngle);

      finalPelvisOrientation.set(finalDesiredPelvisYawAngle + swingPelvisYawScale.getDoubleValue() * desiredSwingPelvisYawAngle, 0.0, 0.0);

      initialize();
   }

   private void addOffsetToOrientation(YoFrameQuaternion originalOrientationToModify, YoFrameOrientation offset)
   {
      originalOrientationToModify.getYawPitchRoll(tempYawPitchRoll);

      tempYawPitchRoll[0] += offset.getYaw().getDoubleValue();
      tempYawPitchRoll[1] += offset.getPitch().getDoubleValue();
      tempYawPitchRoll[2] += offset.getRoll().getDoubleValue();

      originalOrientationToModify.set(tempYawPitchRoll);
   }
}
