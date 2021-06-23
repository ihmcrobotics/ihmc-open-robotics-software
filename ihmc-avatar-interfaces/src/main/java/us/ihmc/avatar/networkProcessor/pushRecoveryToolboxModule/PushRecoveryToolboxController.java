package us.ihmc.avatar.networkProcessor.pushRecoveryToolboxModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.stepAdjustment.SimpleStep;
import us.ihmc.commonWalkingControlModules.captureRegion.CapturabilityBasedPlanarRegionDecider;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintMessageConverter;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.IntFunction;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

public class PushRecoveryToolboxController extends ToolboxController
{
   private final YoBoolean isDone = new YoBoolean("isDone", registry);

   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final HumanoidReferenceFrames referenceFrames;

   private final YoEnum<HighLevelControllerName> currentState;

   private final AtomicReference<RobotConfigurationData> configurationData = new AtomicReference<>();
   private final AtomicReference<CapturabilityBasedStatus> capturabilityBasedStatus = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> mostRecentPlanarRegions = new AtomicReference<>();
   private final AtomicReference<List<StepConstraintRegion>> constraintRegionsToUse = new AtomicReference<>();
   private final AtomicReference<FootstepStatusMessage> footstepStatusMessage = new AtomicReference<>();
   private final AtomicReference<HighLevelStateMessage> highLevelStateMessage = new AtomicReference<>();

   private final SideDependentList<AtomicBoolean> feetInContact = new SideDependentList<>(new AtomicBoolean(true), new AtomicBoolean(true));

   private final IsInContactProvider isInContactProvider = new IsInContactProvider();
   private final FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D();
   private final SideDependentList<FrameConvexPolygon2D> footSupportPolygonsInWorld = new SideDependentList<>(new FrameConvexPolygon2D(),
                                                                                                              new FrameConvexPolygon2D());

   private final MultiStepPushRecoveryModule pushRecoveryControlModule;

   private final IHMCRealtimeROS2Publisher<PushRecoveryResultMessage> pushRecoveryResultPublisher;

   private final FramePoint2D capturePoint = new FramePoint2D();
   private double omega;

   public PushRecoveryToolboxController(StatusMessageOutputManager statusOutputManager,
                                        IHMCRealtimeROS2Publisher<PushRecoveryResultMessage> pushRecoveryResultPublisher,
                                        FullHumanoidRobotModel fullRobotModel,
                                        ConvexPolygon2DReadOnly defaultSupportPolygon,
                                        PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                        double gravityZ,
                                        YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.pushRecoveryResultPublisher = pushRecoveryResultPublisher;
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      this.oneDoFJoints = getAllJointsExcludingHands(fullRobotModel);

      pushRecoveryControlModule = new MultiStepPushRecoveryModule(isInContactProvider,
                                                                  supportPolygonInWorld,
                                                                  footSupportPolygonsInWorld,
                                                                  referenceFrames.getSoleZUpFrames(),
                                                                  defaultSupportPolygon,
                                                                  pushRecoveryControllerParameters,
                                                                  parentRegistry,
                                                                  null);
      CapturabilityBasedPlanarRegionDecider<StepConstraintRegion> planarRegionDecider = new CapturabilityBasedPlanarRegionDecider<>(referenceFrames.getCenterOfMassFrame(),
                                                                                                                                    gravityZ,
                                                                                                                                    StepConstraintRegion::new,
                                                                                                                                    new YoRegistry(
                                                                                                                                          "registryToThrowAway"),
                                                                                                                                    null);

      currentState = new YoEnum<>("controllerCurrentState", registry, HighLevelControllerName.class);
      IntFunction<List<StepConstraintRegion>> constraintRegionProvider = (index) ->
      {
         if (constraintRegionsToUse.get() != null)
            return constraintRegionsToUse.get();
         else
            return null;
      };
      pushRecoveryControlModule.setPlanarRegionDecider(planarRegionDecider);
      pushRecoveryControlModule.setConstraintRegionProvider(constraintRegionProvider);
      isDone.set(false);
   }

   @Override
   public boolean initialize()
   {
      isDone.set(false);
      for (RobotSide robotSide : RobotSide.values)
         feetInContact.get(robotSide).set(true);
      return true;
   }

   @Override
   public void updateInternal()
   {
      try
      {
         updateStateFromMessages();

         pushRecoveryControlModule.updateForDoubleSupport(capturePoint, omega);

         if (shouldBroadcastResult())
            computeAndPublishResultMessage();
      }
      catch (Throwable e)
      {
         e.printStackTrace();

         try
         {
            reportMessage(MessageTools.createControllerCrashNotificationPacket(null, e));
         }
         catch (Exception e1)
         {
            e1.printStackTrace();
         }

         isDone.set(true);
      }
   }

   private void updateStateFromMessages()
   {
      if (highLevelStateMessage.get() != null)
         currentState.set(HighLevelControllerName.fromByte(highLevelStateMessage.getAndSet(null).getHighLevelControllerName()));

      RobotConfigurationData configurationData = this.configurationData.getAndSet(null);
      if (configurationData != null)
      {
         KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(configurationData, fullRobotModel.getRootJoint(), oneDoFJoints);
         referenceFrames.updateFrames();
      }

      CapturabilityBasedStatus capturabilityBasedStatus = this.capturabilityBasedStatus.getAndSet(null);
      if (capturabilityBasedStatus != null)
      {
         isInContactProvider.setCapturabilityBasedStatus(capturabilityBasedStatus);

         updateSupportPolygons(capturabilityBasedStatus);

         capturePoint.set(capturabilityBasedStatus.getCapturePoint2d());
         omega = capturabilityBasedStatus.getOmega();
      }

      PlanarRegionsListMessage planarRegions = this.mostRecentPlanarRegions.getAndSet(null);
      if (planarRegions != null)
      {
         constraintRegionsToUse.set(convertToStepConstraintRegionsList(planarRegions));
      }

      FootstepStatusMessage footstepStatusMessage = this.footstepStatusMessage.getAndSet(null);
      if (footstepStatusMessage != null)
      {
         RobotSide side = RobotSide.fromByte(footstepStatusMessage.getRobotSide());
         if (FootstepStatus.fromByte(footstepStatusMessage.getFootstepStatus()) == FootstepStatus.STARTED)
         {
            feetInContact.get(side).set(false);
         }
         else
         {
            feetInContact.get(side).set(true);
         }
      }
   }

   private void updateSupportPolygons(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      footSupportPolygonsInWorld.get(RobotSide.LEFT).clear();
      footSupportPolygonsInWorld.get(RobotSide.RIGHT).clear();
      supportPolygonInWorld.clear();
      capturabilityBasedStatus.getLeftFootSupportPolygon3d().forEach(point ->
                                                                     {
                                                                        footSupportPolygonsInWorld.get(RobotSide.LEFT).addVertex(point);
                                                                        supportPolygonInWorld.addVertex(point);
                                                                     });
      capturabilityBasedStatus.getRightFootSupportPolygon3d().forEach(point ->
                                                                      {
                                                                         footSupportPolygonsInWorld.get(RobotSide.RIGHT).addVertex(point);
                                                                         supportPolygonInWorld.addVertex(point);
                                                                      });
      footSupportPolygonsInWorld.get(RobotSide.LEFT).update();
      footSupportPolygonsInWorld.get(RobotSide.RIGHT).update();
      supportPolygonInWorld.update();
   }

   private boolean shouldBroadcastResult()
   {
      if (currentState.getEnumValue() != HighLevelControllerName.PUSH_RECOVERY)
         return false;

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!feetInContact.get(robotSide).get())
            return false;
      }

      return true;
   }

   private void computeAndPublishResultMessage()
   {
      PushRecoveryResultMessage message = new PushRecoveryResultMessage();

      if (pushRecoveryControlModule.isRecoveryImpossible())
      {
         message.setIsStepRecoverable(false);
      }
      else
      {
         message.setIsStepRecoverable(true);
         for (int i = 0; i < pushRecoveryControlModule.getNumberOfRecoverySteps(); i++)
         {
            FootstepTiming recoveryTiming = pushRecoveryControlModule.getRecoveryStepTiming(i);
            Footstep step = pushRecoveryControlModule.getRecoveryStep(i);
            FootstepDataMessage stepMessage = message.getRecoverySteps().getFootstepDataList().add();

            stepMessage.setTransferDuration(recoveryTiming.getTransferTime());
            stepMessage.setSwingDuration(recoveryTiming.getSwingTime());
            stepMessage.getLocation().set(step.getFootstepPose().getPosition());
            stepMessage.getOrientation().set(step.getFootstepPose().getOrientation());

            if (pushRecoveryControlModule.hasConstraintRegions())
            {
               message.getStepConstraintList()
                      .add()
                      .set(StepConstraintMessageConverter.convertToStepConstraintMessage(pushRecoveryControlModule.getConstraintRegion(i)));
            }
         }
      }

      pushRecoveryResultPublisher.publish(message);
   }

   @Override
   public void notifyToolboxStateChange(ToolboxState newState)
   {
   }

   @Override
   public boolean isDone()
   {
      return isDone.getValue();
   }

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      this.configurationData.set(newConfigurationData);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      this.capturabilityBasedStatus.set(newStatus);
   }

   public void updatePlanarRegions(PlanarRegionsListMessage planarRegionsListMessage)
   {
      this.mostRecentPlanarRegions.set(planarRegionsListMessage);
   }

   private static List<StepConstraintRegion> convertToStepConstraintRegionsList(PlanarRegionsListMessage message)
   {
      int vertexIndex = 0;
      IDLSequence.Object<Vector3D> normals = message.getRegionNormal();
      IDLSequence.Object<Point3D> origins = message.getRegionOrigin();

      IDLSequence.Object<Point3D> vertexBuffer = message.getVertexBuffer();

      List<StepConstraintRegion> constraintRegions = new ArrayList<>();

      int upperBound = 0;

      for (int regionIndex = 0; regionIndex < message.getConcaveHullsSize().size(); regionIndex++)
      {
         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         if (message.getRegionOrientation().isEmpty()
             || Math.abs(AngleTools.trimAngleMinusPiToPi(message.getRegionOrientation().get(regionIndex).getAngle())) < 1.0e-3)
         {
            AxisAngle regionOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(normals.get(regionIndex));
            transformToWorld.set(regionOrientation, origins.get(regionIndex));
         }
         else
         {
            transformToWorld.set(message.getRegionOrientation().get(regionIndex), message.getRegionOrigin().get(regionIndex));
         }

         upperBound += message.getConcaveHullsSize().get(regionIndex);
         List<Point2D> concaveHullVertices = new ArrayList<>();

         for (; vertexIndex < upperBound; vertexIndex++)
         {
            concaveHullVertices.add(new Point2D(vertexBuffer.get(vertexIndex)));
         }

         StepConstraintRegion planarRegion = new StepConstraintRegion(transformToWorld,
                                                                      Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices),
                                                                      new ArrayList<>());
         planarRegion.setRegionId(message.getRegionId().get(regionIndex));
         constraintRegions.add(planarRegion);
      }

      return constraintRegions;
   }

   private class IsInContactProvider implements Function<RobotSide, Boolean>
   {
      private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

      public void setCapturabilityBasedStatus(CapturabilityBasedStatus status)
      {
         this.capturabilityBasedStatus.set(status);
      }

      public Boolean apply(RobotSide robotSide)
      {
         if (!feetInContact.get(robotSide).get())
            return true;

         if (robotSide == RobotSide.LEFT)
            return capturabilityBasedStatus.getLeftFootSupportPolygon3d().isEmpty();
         else
            return capturabilityBasedStatus.getRightFootSupportPolygon3d().isEmpty();
      }
   }
}
