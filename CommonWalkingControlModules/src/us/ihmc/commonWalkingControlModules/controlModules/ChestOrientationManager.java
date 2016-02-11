package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestOrientationProvider;
import us.ihmc.commonWalkingControlModules.packetConsumers.ChestTrajectoryMessageSubscriber;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.robotics.controllers.YoOrientationPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.OrientationTrajectoryGeneratorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.WaypointOrientationTrajectoryData;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class ChestOrientationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;
   private final ChestOrientationControlModule chestOrientationControlModule;
   private final MomentumBasedController momentumBasedController;
   private int jacobianId = -1;

   private final ChestTrajectoryMessageSubscriber chestTrajectoryMessageSubscriber;
   private final ChestOrientationProvider chestOrientationProvider;
   private final DoubleYoVariable yoTime;
   private final DoubleYoVariable receivedNewChestOrientationTime;

   private final BooleanYoVariable isTrackingOrientation;
   private final BooleanYoVariable isUsingWaypointTrajectory;
   private final YoFrameVector yoControlledAngularAcceleration;

   private OrientationTrajectoryGeneratorInMultipleFrames activeTrajectoryGenerator;
   private final SimpleOrientationTrajectoryGenerator simpleOrientationTrajectoryGenerator;
   private final MultipleWaypointsOrientationTrajectoryGenerator waypointOrientationTrajectoryGenerator;
   private final ReferenceFrame pelvisZUpFrame;
   private final ReferenceFrame chestFrame;

   private final BooleanYoVariable initializeToCurrent;

   public ChestOrientationManager(MomentumBasedController momentumBasedController, YoOrientationPIDGains chestControlGains,
         ChestOrientationProvider chestOrientationProvider, ChestTrajectoryMessageSubscriber chestTrajectoryMessageSubscriber, double trajectoryTime,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());

      this.momentumBasedController = momentumBasedController;
      this.yoTime = momentumBasedController.getYoTime();
      this.chestTrajectoryMessageSubscriber = chestTrajectoryMessageSubscriber;
      this.chestOrientationProvider = chestOrientationProvider;
      this.pelvisZUpFrame = momentumBasedController.getPelvisZUpFrame();

      FullHumanoidRobotModel fullRobotModel = momentumBasedController.getFullRobotModel();
      RigidBody chest = fullRobotModel.getChest();
      chestFrame = chest.getBodyFixedFrame();
      TwistCalculator twistCalculator = momentumBasedController.getTwistCalculator();
      double controlDT = momentumBasedController.getControlDT();
      chestOrientationControlModule = new ChestOrientationControlModule(pelvisZUpFrame, chest, twistCalculator, controlDT, chestControlGains, registry);

      yoControlledAngularAcceleration = new YoFrameVector("controlledChestAngularAcceleration", chestFrame, registry);

      if (chestOrientationProvider != null || chestTrajectoryMessageSubscriber != null)
      {
         isTrackingOrientation = new BooleanYoVariable("isTrackingOrientation", registry);
         receivedNewChestOrientationTime = new DoubleYoVariable("receivedNewChestOrientationTime", registry);

         isUsingWaypointTrajectory = new BooleanYoVariable(getClass().getSimpleName() + "IsUsingWaypointTrajectory", registry);
         isUsingWaypointTrajectory.set(false);

         boolean allowMultipleFrames = true;
         simpleOrientationTrajectoryGenerator = new SimpleOrientationTrajectoryGenerator("chest", allowMultipleFrames, pelvisZUpFrame, parentRegistry);
         simpleOrientationTrajectoryGenerator.setTrajectoryTime(trajectoryTime);
         simpleOrientationTrajectoryGenerator.initialize();
         activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;

         waypointOrientationTrajectoryGenerator = new MultipleWaypointsOrientationTrajectoryGenerator("chestWaypoint", 15, allowMultipleFrames, pelvisZUpFrame,
               registry);

         initializeToCurrent = new BooleanYoVariable("initializeChestOrientationToCurrent", registry);
      }
      else
      {
         isTrackingOrientation = null;
         receivedNewChestOrientationTime = null;
         isUsingWaypointTrajectory = null;
         simpleOrientationTrajectoryGenerator = null;
         waypointOrientationTrajectoryGenerator = null;

         initializeToCurrent = null;
      }

      parentRegistry.addChild(registry);
   }

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector feedForwardAngularAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

   private final FrameVector controlledAngularAcceleration = new FrameVector();

   public void compute()
   {
      if (isUsingWaypointTrajectory != null)
      {
         if (isUsingWaypointTrajectory.getBooleanValue())
            activeTrajectoryGenerator = waypointOrientationTrajectoryGenerator;
         else
            activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
      }

      checkForNewDesiredOrientationInformation();
      handleChestTrajectoryMessages();

      if (activeTrajectoryGenerator != null && isTrackingOrientation.getBooleanValue())
      {
         double deltaTime = yoTime.getDoubleValue() - receivedNewChestOrientationTime.getDoubleValue();
         activeTrajectoryGenerator.compute(deltaTime);
         boolean isTrajectoryDone = activeTrajectoryGenerator.isDone();

         if (isTrajectoryDone)
            activeTrajectoryGenerator.changeFrame(pelvisZUpFrame);

         activeTrajectoryGenerator.get(desiredOrientation);
         chestOrientationControlModule.setDesireds(desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration);
         isTrackingOrientation.set(!isTrajectoryDone);
      }

      if (jacobianId >= 0)
      {
         chestOrientationControlModule.compute();
         TaskspaceConstraintData taskspaceConstraintData = chestOrientationControlModule.getTaskspaceConstraintData();

         if (yoControlledAngularAcceleration != null)
         {
            SpatialAccelerationVector spatialAcceleration = taskspaceConstraintData.getSpatialAcceleration();
            if (spatialAcceleration.getExpressedInFrame() != null) // That happens when there is no joint to control.
            {
               spatialAcceleration.packAngularPart(controlledAngularAcceleration);
               yoControlledAngularAcceleration.set(controlledAngularAcceleration);
            }
         }

         momentumBasedController.setDesiredSpatialAcceleration(jacobianId, taskspaceConstraintData);
      }
   }

   public void holdCurrentOrientation()
   {
      if (initializeToCurrent != null)
         initializeToCurrent.set(true);
   }

   private final FrameOrientation tempOrientation = new FrameOrientation();
   private final FrameVector tempAngularVelocity = new FrameVector();

   private void handleChestTrajectoryMessages()
   {
      if (chestTrajectoryMessageSubscriber == null || !chestTrajectoryMessageSubscriber.isNewTrajectoryMessageAvailable())
         return;

      ChestTrajectoryMessage message = chestTrajectoryMessageSubscriber.pollMessage();

      waypointOrientationTrajectoryGenerator.switchTrajectoryFrame(worldFrame);
      waypointOrientationTrajectoryGenerator.clear();

      if (message.getWaypoint(0).getTime() > 1.0e-5)
      {
         tempOrientation.setToZero(chestFrame);
         tempOrientation.changeFrame(worldFrame);
         tempAngularVelocity.setToZero(worldFrame);

         waypointOrientationTrajectoryGenerator.appendWaypoint(0.0, tempOrientation, tempAngularVelocity);
      }

      waypointOrientationTrajectoryGenerator.appendWaypoints(message.getWaypoints());
      waypointOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
      waypointOrientationTrajectoryGenerator.initialize();
      isUsingWaypointTrajectory.set(true);
      activeTrajectoryGenerator = waypointOrientationTrajectoryGenerator;
      isTrackingOrientation.set(true);
   }

   private void checkForNewDesiredOrientationInformation()
   {
      if (chestOrientationProvider == null)
         return;

      if (initializeToCurrent.getBooleanValue())
      {
         initializeToCurrent.set(false);

         receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

         simpleOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
         desiredOrientation.setToZero(chestFrame);
         desiredOrientation.changeFrame(pelvisZUpFrame);
         simpleOrientationTrajectoryGenerator.setInitialOrientation(desiredOrientation);
         simpleOrientationTrajectoryGenerator.setFinalOrientation(desiredOrientation);
         simpleOrientationTrajectoryGenerator.setTrajectoryTime(0.0);
         simpleOrientationTrajectoryGenerator.initialize();
         isUsingWaypointTrajectory.set(false);
         activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
         isTrackingOrientation.set(true);
      }
      else if (chestOrientationProvider.checkForHomeOrientation())
      {
         receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

         simpleOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
         activeTrajectoryGenerator.changeFrame(pelvisZUpFrame);
         activeTrajectoryGenerator.get(desiredOrientation);
         simpleOrientationTrajectoryGenerator.setInitialOrientation(desiredOrientation);
         desiredOrientation.setToZero(pelvisZUpFrame);
         simpleOrientationTrajectoryGenerator.setFinalOrientation(desiredOrientation);
         simpleOrientationTrajectoryGenerator.setTrajectoryTime(chestOrientationProvider.getTrajectoryTime());
         simpleOrientationTrajectoryGenerator.initialize();
         isUsingWaypointTrajectory.set(false);
         activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
         isTrackingOrientation.set(true);
      }
      else if (chestOrientationProvider.checkForNewChestOrientation())
      {
         receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

         simpleOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
         activeTrajectoryGenerator.changeFrame(pelvisZUpFrame);
         activeTrajectoryGenerator.get(desiredOrientation);
         simpleOrientationTrajectoryGenerator.setInitialOrientation(desiredOrientation);
         FrameOrientation desiredChestOrientation = chestOrientationProvider.getDesiredChestOrientation();
         desiredChestOrientation.changeFrame(pelvisZUpFrame);
         simpleOrientationTrajectoryGenerator.setFinalOrientation(desiredChestOrientation);
         simpleOrientationTrajectoryGenerator.setTrajectoryTime(chestOrientationProvider.getTrajectoryTime());
         simpleOrientationTrajectoryGenerator.initialize();
         isUsingWaypointTrajectory.set(false);
         activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;
         isTrackingOrientation.set(true);
      }
      else if (chestOrientationProvider.checkForNewChestOrientationWithWaypoints())
      {
         receivedNewChestOrientationTime.set(yoTime.getDoubleValue());

         simpleOrientationTrajectoryGenerator.changeFrame(pelvisZUpFrame);
         activeTrajectoryGenerator.changeFrame(pelvisZUpFrame);
         activeTrajectoryGenerator.get(desiredOrientation);
         simpleOrientationTrajectoryGenerator.setInitialOrientation(desiredOrientation);

         WaypointOrientationTrajectoryData trajectoryData = chestOrientationProvider.getDesiredChestOrientationWithWaypoints();

         int lastIndex = trajectoryData.getTimeAtWaypoints().length - 1;

         double totalTime = trajectoryData.getTimeAtWaypoints()[lastIndex];

         FrameOrientation lastChestOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), trajectoryData.getOrientations()[lastIndex]);

         simpleOrientationTrajectoryGenerator.setFinalOrientation(lastChestOrientation);
         simpleOrientationTrajectoryGenerator.setTrajectoryTime(totalTime);

         simpleOrientationTrajectoryGenerator.initialize();
         isUsingWaypointTrajectory.set(false);
         activeTrajectoryGenerator = simpleOrientationTrajectoryGenerator;

         isTrackingOrientation.set(true);
      }
   }

   public void setDesireds(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector desiredAngularAcceleration)
   {
      chestOrientationControlModule.setDesireds(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
   }

   public int createJacobian(FullRobotModel fullRobotModel, String[] chestOrientationControlJointNames)
   {
      RigidBody rootBody = fullRobotModel.getRootJoint().getSuccessor();
      InverseDynamicsJoint[] allJoints = ScrewTools.computeSupportAndSubtreeJoints(rootBody);
      InverseDynamicsJoint[] chestOrientationControlJoints = ScrewTools.findJointsWithNames(allJoints, chestOrientationControlJointNames);

      ReferenceFrame chestFrame = chestOrientationControlModule.getChest().getBodyFixedFrame();
      int jacobianId = momentumBasedController.getOrCreateGeometricJacobian(chestOrientationControlJoints, chestFrame);
      return jacobianId;
   }

   public void setUp(RigidBody base, int jacobianId)
   {
      this.jacobianId = jacobianId;
      chestOrientationControlModule.setBase(base);
      chestOrientationControlModule.setJacobian(momentumBasedController.getJacobian(jacobianId));
   }

   public void setUp(RigidBody base, int jacobianId, double proportionalGainX, double proportionalGainY, double proportionalGainZ, double derivativeGainX,
         double derivativeGainY, double derivativeGainZ)
   {
      this.jacobianId = jacobianId;
      chestOrientationControlModule.setBase(base);
      chestOrientationControlModule.setJacobian(momentumBasedController.getJacobian(jacobianId));
      chestOrientationControlModule.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
      chestOrientationControlModule.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setControlGains(double proportionalGain, double derivativeGain)
   {
      chestOrientationControlModule.setProportionalGains(proportionalGain, proportionalGain, proportionalGain);
      chestOrientationControlModule.setDerivativeGains(derivativeGain, derivativeGain, derivativeGain);
   }

   public void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      chestOrientationControlModule.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public void turnOff()
   {
      setUp(null, -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }
}
