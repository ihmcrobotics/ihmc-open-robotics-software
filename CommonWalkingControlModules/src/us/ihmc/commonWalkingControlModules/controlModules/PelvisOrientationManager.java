package us.ihmc.commonWalkingControlModules.controlModules;

import javax.vecmath.Quat4d;

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
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.OrientationProvider;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.yoUtilities.controllers.YoOrientationPIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.utilities.humanoidRobot.footstep.Footstep;
import us.ihmc.yoUtilities.math.frames.YoFrameQuaternion;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.WaypointOrientationTrajectoryData;
import us.ihmc.yoUtilities.math.trajectories.providers.YoQuaternionProvider;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class PelvisOrientationManager
{
   private static final double defaultTrajectoryTime = 1.0;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double TRAJECTORY_TIME_TO_PREPARE_FOR_LOCOMOTION = 0.1;

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
   private final YoFrameQuaternion desiredPelvisOrientationOffset;

   private final DoubleYoVariable offsetTrajectoryTime = new DoubleYoVariable("offsetTrajectoryTime", registry);
   private final YoFrameQuaternion initialPelvisOrientationOffset;
   private final YoFrameQuaternion finalPelvisOrientationOffset;

   private final BooleanYoVariable isUsingWaypointTrajectory;
   private OrientationTrajectoryGenerator activeOrientationOffsetTrajectoryGenerator;
   private final MultipleWaypointsOrientationTrajectoryGenerator pelvisWaypointsOrientationOffsetTrajectoryGenerator;
   private final OrientationTrajectoryGenerator pelvisOrientationOffsetTrajectoryGenerator;
   private final BooleanYoVariable prepareForLocomotion = new BooleanYoVariable(getClass().getSimpleName() + "PrepareForLocomotion", registry);

   private final YoFrameQuaternion desiredPelvisOrientationWithOffset = new YoFrameQuaternion("desiredPelvisOrientationWithOffset", worldFrame, registry);

   private final DoubleYoVariable yoTime;

   private final OrientationTrajectoryData orientationTrajectoryData = new OrientationTrajectoryData();
   private final RootJointAngularAccelerationControlModule rootJointAngularAccelerationControlModule;

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();
   private final FrameVector tempAngularAcceleration = new FrameVector();

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final ReferenceFrame midFeetZUpFrame;
   private final ReferenceFrame pelvisFrame;
   private final ReferenceFrame desiredPelvisFrame;

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


      YoOrientationPIDGains pelvisOrientationControlGains = walkingControllerParameters.createPelvisOrientationControlGains(registry);
      rootJointAngularAccelerationControlModule = new RootJointAngularAccelerationControlModule(momentumBasedController, pelvisOrientationControlGains, registry);
      OrientationProvider initialPelvisOrientationProvider = new YoQuaternionProvider(initialPelvisOrientation);
      OrientationProvider finalPelvisOrientationProvider = new YoQuaternionProvider(finalPelvisOrientation);
      pelvisOrientationTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvis", worldFrame, trajectoryTimeProvider,
            initialPelvisOrientationProvider, finalPelvisOrientationProvider, registry);

      desiredPelvisFrame = new ReferenceFrame("desiredPelvisFrame", worldFrame)
      {
         private static final long serialVersionUID = -1472151257649344278L;

         private final Quat4d rotationToParent = new Quat4d();

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            pelvisFrame.getTransformToDesiredFrame(transformToParent, parentFrame);
            desiredPelvisOrientation.get(rotationToParent);
            transformToParent.setRotation(rotationToParent);
         }
      };      

      desiredPelvisOrientationOffset = new YoFrameQuaternion("desiredPelvis", "Offset", desiredPelvisFrame, registry);

      offsetTrajectoryTime.set(defaultTrajectoryTime);
      initialPelvisOrientationOffset = new YoFrameQuaternion("initialPelvis", "Offset", desiredPelvisFrame, registry);
      finalPelvisOrientationOffset = new YoFrameQuaternion("finalPelvis", "Offset", desiredPelvisFrame, registry);
      OrientationProvider initialPelvisOrientationOffsetProvider = new YoQuaternionProvider(initialPelvisOrientationOffset);
      OrientationProvider finalPelvisOrientationOffsetProvider = new YoQuaternionProvider(finalPelvisOrientationOffset);
      DoubleProvider offsetTrajectoryTimeProvider = new YoVariableDoubleProvider(offsetTrajectoryTime);

      isUsingWaypointTrajectory = new BooleanYoVariable(getClass().getSimpleName() + "IsUsingWaypointTrajectory", registry);
      isUsingWaypointTrajectory.set(false);

      pelvisOrientationOffsetTrajectoryGenerator = new OrientationInterpolationTrajectoryGenerator("pelvisOffset", desiredPelvisFrame, offsetTrajectoryTimeProvider,
            initialPelvisOrientationOffsetProvider, finalPelvisOrientationOffsetProvider, registry);
      pelvisOrientationOffsetTrajectoryGenerator.initialize();
      activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;

      boolean doVelocityAtWaypoints = false;
      boolean allowMultipleFrames = false;
      pelvisWaypointsOrientationOffsetTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("pelvisWaypointsOffset", 15, doVelocityAtWaypoints, allowMultipleFrames, desiredPelvisFrame, registry);

      parentRegistry.addChild(registry);
   }

   private void initialize()
   {
      isUsingWaypointTrajectory.set(false);
      activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;

      initialPelvisOrientationTime.set(yoTime.getDoubleValue());

      pelvisOrientationTrajectoryGenerator.initialize();
      pelvisOrientationTrajectoryGenerator.packAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      desiredPelvisOrientation.set(tempOrientation);
      desiredPelvisAngularVelocity.set(tempAngularVelocity);
      desiredPelvisAngularAcceleration.set(tempAngularAcceleration);
   }

   public void compute()
   {
      if (isUsingWaypointTrajectory != null)
      {
         if (isUsingWaypointTrajectory.getBooleanValue())
            activeOrientationOffsetTrajectoryGenerator = pelvisWaypointsOrientationOffsetTrajectoryGenerator;
         else
            activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;
      }
      
      updateDesireds();
      
      if (!RotationFunctions.isRotationProper(orientationTrajectoryData.getOrientation().getMatrix3dCopy()))
         throw new RuntimeException(getClass().getSimpleName() + ": Desired pelvis orientation is screwed");

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
      desiredPelvisFrame.update();
      


      if (pelvisPoseProvider != null)
      {
         if (pelvisPoseProvider.checkForHomeOrientation())
         {
            initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());
            offsetTrajectoryTime.set(pelvisPoseProvider.getTrajectoryTime());
            activeOrientationOffsetTrajectoryGenerator.get(tempOrientation);
            initialPelvisOrientationOffset.set(tempOrientation);
            finalPelvisOrientationOffset.set(0.0, 0.0, 0.0);
            pelvisOrientationOffsetTrajectoryGenerator.initialize();
            isUsingWaypointTrajectory.set(false);
            activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;
         }
         else if (pelvisPoseProvider.checkForNewOrientation())
         {
            initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());
            offsetTrajectoryTime.set(pelvisPoseProvider.getTrajectoryTime());
            activeOrientationOffsetTrajectoryGenerator.get(tempOrientation);
            initialPelvisOrientationOffset.set(tempOrientation);
            FrameOrientation pelvisOrientationProvided = pelvisPoseProvider.getDesiredPelvisOrientation(desiredPelvisFrame);
            pelvisOrientationProvided.changeFrame(desiredPelvisFrame);
            tempOrientation.setIncludingFrame(pelvisOrientationProvided);
            finalPelvisOrientationOffset.set(tempOrientation);
            pelvisOrientationOffsetTrajectoryGenerator.initialize();
            isUsingWaypointTrajectory.set(false);
            activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;
         }
         else if (pelvisPoseProvider.checkForNewOrientationWithWaypoints())
         {
            initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());
            activeOrientationOffsetTrajectoryGenerator.get(tempOrientation);
            pelvisWaypointsOrientationOffsetTrajectoryGenerator.clear();
            pelvisWaypointsOrientationOffsetTrajectoryGenerator.appendWaypoint(0.0, tempOrientation);
            WaypointOrientationTrajectoryData trajectoryData = pelvisPoseProvider.getDesiredPelvisOrientationWithWaypoints();
            trajectoryData.changeFrame(desiredPelvisFrame);
            pelvisWaypointsOrientationOffsetTrajectoryGenerator.appendWaypoints(trajectoryData);
            pelvisWaypointsOrientationOffsetTrajectoryGenerator.initialize();
            isUsingWaypointTrajectory.set(true);
            activeOrientationOffsetTrajectoryGenerator = pelvisWaypointsOrientationOffsetTrajectoryGenerator;
         }
      }

      if (prepareForLocomotion.getBooleanValue())
      {
         initialPelvisOrientationOffsetTime.set(yoTime.getDoubleValue());
         offsetTrajectoryTime.set(TRAJECTORY_TIME_TO_PREPARE_FOR_LOCOMOTION);
         activeOrientationOffsetTrajectoryGenerator.get(tempOrientation);
         initialPelvisOrientationOffset.set(tempOrientation);
         tempOrientation.setToZero(desiredPelvisFrame);
         finalPelvisOrientationOffset.set(tempOrientation);
         pelvisOrientationOffsetTrajectoryGenerator.initialize();
         isUsingWaypointTrajectory.set(false);
         activeOrientationOffsetTrajectoryGenerator = pelvisOrientationOffsetTrajectoryGenerator;
         prepareForLocomotion.set(false);
      }

      double deltaTimeOffset = yoTime.getDoubleValue() - initialPelvisOrientationOffsetTime.getDoubleValue();
      activeOrientationOffsetTrajectoryGenerator.compute(deltaTimeOffset);
      activeOrientationOffsetTrajectoryGenerator.packAngularData(tempOrientation, tempAngularVelocity, tempAngularAcceleration);

      desiredPelvisOrientationOffset.set(tempOrientation);

      tempOrientation.changeFrame(worldFrame);
      tempAngularVelocity.changeFrame(worldFrame);
      tempAngularAcceleration.changeFrame(worldFrame);

      desiredPelvisOrientationWithOffset.set(tempOrientation);
      desiredPelvisAngularVelocity.add(tempAngularVelocity);
      desiredPelvisAngularAcceleration.add(tempAngularAcceleration);

      desiredPelvisOrientationWithOffset.getFrameOrientationIncludingFrame(tempOrientation);
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

   public void prepareForLocomotion()
   {
      prepareForLocomotion.set(true);
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
}
