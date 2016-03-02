package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.ArrayList;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.AbortWalkingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.desiredFootStep.BlindWalkingPacketConsumer;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepPathConsumer;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepPathCoordinator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepTimingParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.ObsoleteBlindWalkingToDestinationDesiredFootstepCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.PauseWalkingMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetConsumers.FootTrajectoryMessageSubscriber;
import us.ihmc.commonWalkingControlModules.packetProducers.CapturabilityBasedStatusProducer;
import us.ihmc.commonWalkingControlModules.packetProviders.HighLevelStateMessageSubscriber;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantSwingTimeCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.ConstantTransferTimeCalculator;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.AbortWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.BlindWalkingPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.util.PeriodicThreadScheduler;

public class DataProducerVariousWalkingProviderFactory implements VariousWalkingProviderFactory
{
   private final HumanoidGlobalDataProducer objectCommunicator;
   private final FootstepTimingParameters footstepTimingParameters;
   private final PeriodicThreadScheduler scheduler;

   public DataProducerVariousWalkingProviderFactory(HumanoidGlobalDataProducer objectCommunicator, FootstepTimingParameters footstepTimingParameters,
         PeriodicThreadScheduler scheduler)
   {
      this.objectCommunicator = objectCommunicator;
      this.footstepTimingParameters = footstepTimingParameters;
      this.scheduler = scheduler;
   }

   public VariousWalkingProviders createVariousWalkingProviders(DoubleYoVariable yoTime, FullHumanoidRobotModel fullRobotModel,
         WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames,
         SideDependentList<? extends ContactablePlaneBody> feet, ConstantTransferTimeCalculator transferTimeCalculator,
         ConstantSwingTimeCalculator swingTimeCalculator, ArrayList<Updatable> updatables, YoVariableRegistry registry,
         YoGraphicsListRegistry yoGraphicsListRegistry, CloseableAndDisposableRegistry closeableAndDisposeableRegistry)
   {

      FootTrajectoryMessageSubscriber footTrajectoryMessageSubscriber = new FootTrajectoryMessageSubscriber(objectCommunicator);

      // This guy will redirect the messages contained in the WholeBodyTrajectoryMessage to the other subscribers. No need to hold on it.

      ObsoleteBlindWalkingToDestinationDesiredFootstepCalculator desiredFootstepCalculator = HighLevelHumanoidControllerFactoryHelper
            .getBlindWalkingToDestinationDesiredFootstepCalculator(walkingControllerParameters, referenceFrames, feet, registry);

      CapturabilityBasedStatusProducer capturabilityBasedStatusProducer = new CapturabilityBasedStatusProducer(closeableAndDisposeableRegistry, scheduler,
            objectCommunicator);

      FootstepPathCoordinator footstepPathCoordinator = new FootstepPathCoordinator(footstepTimingParameters, objectCommunicator, desiredFootstepCalculator,
            swingTimeCalculator, transferTimeCalculator, registry);

      FootstepPathConsumer footstepPathConsumer = new FootstepPathConsumer(feet, footstepPathCoordinator, objectCommunicator);
      BlindWalkingPacketConsumer blindWalkingPacketConsumer = new BlindWalkingPacketConsumer(footstepPathCoordinator);
      PauseWalkingMessageSubscriber pauseWalkingMessageSubscriber = new PauseWalkingMessageSubscriber(footstepPathCoordinator);
      HighLevelStateMessageSubscriber highLevelStateProvider = new HighLevelStateMessageSubscriber();

      AbortWalkingMessageSubscriber abortWalkingMessageSubscriber = new AbortWalkingMessageSubscriber();

      objectCommunicator.attachListener(FootstepDataListMessage.class, footstepPathConsumer);
      objectCommunicator.attachListener(BlindWalkingPacket.class, blindWalkingPacketConsumer);
      objectCommunicator.attachListener(PauseWalkingMessage.class, pauseWalkingMessageSubscriber);
      objectCommunicator.attachListener(HighLevelStateMessage.class, highLevelStateProvider);
      objectCommunicator.attachListener(AbortWalkingMessage.class, abortWalkingMessageSubscriber);

      VariousWalkingProviders variousWalkingProviders = new VariousWalkingProviders(footTrajectoryMessageSubscriber, footstepPathCoordinator,
            highLevelStateProvider, capturabilityBasedStatusProducer, abortWalkingMessageSubscriber);

      return variousWalkingProviders;
   }

}
