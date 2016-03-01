package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import us.ihmc.commonWalkingControlModules.packetConsumers.PacketValidityChecker;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;

public class ControllerNetworkSubscriber
{
   private final ControllerCommandInputManager controllerCommandInputManager;
   private final HumanoidGlobalDataProducer globalDataProducer;

   public ControllerNetworkSubscriber(ControllerCommandInputManager controllerCommandInputManager, HumanoidGlobalDataProducer globalDataProducer)
   {
      this.controllerCommandInputManager = controllerCommandInputManager;
      this.globalDataProducer = globalDataProducer;
      setupHandTrajectoryMessageSubscriber();
      setupArmTrajectoryMessageSubscriber();
      setupFootTrajectoryMessageSubscriber();
      setupHeadTrajectoryMessageSubscriber();
      setupChestTrajectoryMessageSubscriber();
      setupPelvisTrajectoryMessageSubscriber();
      setupPelvisHeightTrajectoryMessageSubscriber();
      setupPelvisOrientationTrajectoryMessageSubscriber();
      setupFootstepDataListMessageSubscriber();
      setupEndEffectorLoadBearingMessageSubscriber();
      setupStopAllTrajectoryMessageSubscriber();
      setupArmDesiredAccelerationsMessageSubscriber();
      setupWholeBodyTrajectoryMessageSubscriber();
   }

   private void setupHandTrajectoryMessageSubscriber()
   {
      PacketConsumer<HandTrajectoryMessage> messageSubscriber = new PacketConsumer<HandTrajectoryMessage>()
      {
         @Override
         public void receivedPacket(HandTrajectoryMessage message)
         {
            if (!PacketValidityChecker.validateHandTrajectoryMessage(message, globalDataProducer))
               return;

            controllerCommandInputManager.submitHandTrajectoryMessage(message);
         }
      };
      globalDataProducer.attachListener(HandTrajectoryMessage.class, messageSubscriber);
   }

   private void setupArmTrajectoryMessageSubscriber()
   {
      PacketConsumer<ArmTrajectoryMessage> messageSubscriber = new PacketConsumer<ArmTrajectoryMessage>()
      {
         @Override
         public void receivedPacket(ArmTrajectoryMessage message)
         {
            if (!PacketValidityChecker.validateArmTrajectoryMessage(message, globalDataProducer))
               return;

            controllerCommandInputManager.submitArmTrajectoryMessage(message);
         }
      };
      globalDataProducer.attachListener(ArmTrajectoryMessage.class, messageSubscriber);
   }

   private void setupFootTrajectoryMessageSubscriber()
   {
      PacketConsumer<FootTrajectoryMessage> messageSubscriber = new PacketConsumer<FootTrajectoryMessage>()
      {
         @Override
         public void receivedPacket(FootTrajectoryMessage message)
         {
            if (!PacketValidityChecker.validateFootTrajectoryMessage(message, globalDataProducer))
               return;

            controllerCommandInputManager.submitFootTrajectoryMessage(message);
         }
      };
      globalDataProducer.attachListener(FootTrajectoryMessage.class, messageSubscriber);
   }

   private void setupHeadTrajectoryMessageSubscriber()
   {
      PacketConsumer<HeadTrajectoryMessage> messageSubscriber = new PacketConsumer<HeadTrajectoryMessage>()
      {
         @Override
         public void receivedPacket(HeadTrajectoryMessage message)
         {
            if (!PacketValidityChecker.validateHeadTrajectoryMessage(message, globalDataProducer))
               return;

            controllerCommandInputManager.submitHeadTrajectoryMessage(message);
         }
      };
      globalDataProducer.attachListener(HeadTrajectoryMessage.class, messageSubscriber);
   }

   private void setupChestTrajectoryMessageSubscriber()
   {
      PacketConsumer<ChestTrajectoryMessage> messageSubscriber = new PacketConsumer<ChestTrajectoryMessage>()
      {
         @Override
         public void receivedPacket(ChestTrajectoryMessage message)
         {
            if (!PacketValidityChecker.validateChestTrajectoryMessage(message, globalDataProducer))
               return;

            controllerCommandInputManager.submitChestTrajectoryMessage(message);
         }
      };
      globalDataProducer.attachListener(ChestTrajectoryMessage.class, messageSubscriber);
   }

   private void setupPelvisTrajectoryMessageSubscriber()
   {
      PacketConsumer<PelvisTrajectoryMessage> messageSubscriber = new PacketConsumer<PelvisTrajectoryMessage>()
      {
         @Override
         public void receivedPacket(PelvisTrajectoryMessage message)
         {
            if (!PacketValidityChecker.validatePelvisTrajectoryMessage(message, globalDataProducer))
               return;

            controllerCommandInputManager.submitPelvisTrajectoryMessage(message);
         }
      };
      globalDataProducer.attachListener(PelvisTrajectoryMessage.class, messageSubscriber);
   }

   private void setupPelvisHeightTrajectoryMessageSubscriber()
   {
      PacketConsumer<PelvisHeightTrajectoryMessage> messageSubscriber = new PacketConsumer<PelvisHeightTrajectoryMessage>()
      {
         @Override
         public void receivedPacket(PelvisHeightTrajectoryMessage message)
         {
            if (!PacketValidityChecker.validatePelvisHeightTrajectoryMessage(message, globalDataProducer))
               return;

            controllerCommandInputManager.submitPelvisHeightTrajectoryMessage(message);
         }
      };
      globalDataProducer.attachListener(PelvisHeightTrajectoryMessage.class, messageSubscriber);
   }

   private void setupPelvisOrientationTrajectoryMessageSubscriber()
   {
      PacketConsumer<PelvisOrientationTrajectoryMessage> messageSubscriber = new PacketConsumer<PelvisOrientationTrajectoryMessage>()
      {
         @Override
         public void receivedPacket(PelvisOrientationTrajectoryMessage message)
         {
            if (!PacketValidityChecker.validatePelvisOrientationTrajectoryMessage(message, globalDataProducer))
               return;

            controllerCommandInputManager.submitPelvisOrientationTrajectoryMessage(message);
         }
      };
      globalDataProducer.attachListener(PelvisOrientationTrajectoryMessage.class, messageSubscriber);
   }
   
   private void setupWholeBodyTrajectoryMessageSubscriber()
   {
      PacketConsumer<WholeBodyTrajectoryMessage> messageSubscriber = new PacketConsumer<WholeBodyTrajectoryMessage>()
      {
         @Override
         public void receivedPacket(WholeBodyTrajectoryMessage message)
         {
            if (PacketValidityChecker.validatePacket(message, true) != null)
               return;
            if (!message.checkRobotSideConsistency())
            {
               globalDataProducer.notifyInvalidPacketReceived(message.getClass(), "The robotSide of a field is inconsistent with its name.");
               return;
            }
            
            controllerCommandInputManager.submitWholeBodyTrajectoryMessage(message);
         }
      };
      globalDataProducer.attachListener(WholeBodyTrajectoryMessage.class, messageSubscriber);
   }

   private void setupFootstepDataListMessageSubscriber()
   {
      PacketConsumer<FootstepDataListMessage> messageSubscriber = new PacketConsumer<FootstepDataListMessage>()
      {
         @Override
         public void receivedPacket(FootstepDataListMessage message)
         {
            String errorMessage = PacketValidityChecker.validateFootstepDataListMessage(message);
            if (errorMessage != null)
            {
               globalDataProducer.notifyInvalidPacketReceived(FootstepDataListMessage.class, errorMessage);
               return;
            }

            controllerCommandInputManager.submitFootstepDataListMessage(message);
         }
      };
      globalDataProducer.attachListener(FootstepDataListMessage.class, messageSubscriber);
   }

   private void setupEndEffectorLoadBearingMessageSubscriber()
   {
      PacketConsumer<EndEffectorLoadBearingMessage> messageSubscriber = new PacketConsumer<EndEffectorLoadBearingMessage>()
      {
         @Override
         public void receivedPacket(EndEffectorLoadBearingMessage message)
         {
            if (!PacketValidityChecker.validateEndEffectorLoadBearingMessage(message, globalDataProducer))
               return;

            controllerCommandInputManager.submitEndEffectorLoadBearingMessage(message);
         }
      };
      globalDataProducer.attachListener(EndEffectorLoadBearingMessage.class, messageSubscriber);
   }

   private void setupStopAllTrajectoryMessageSubscriber()
   {
      PacketConsumer<StopAllTrajectoryMessage> messageSubscriber = new PacketConsumer<StopAllTrajectoryMessage>()
      {
         @Override
         public void receivedPacket(StopAllTrajectoryMessage message)
         {
            if (message == null)
               return;

            controllerCommandInputManager.submitStopAllTrajectoryMessage(message);
         }
      };
      globalDataProducer.attachListener(StopAllTrajectoryMessage.class, messageSubscriber);
   }

   private void setupArmDesiredAccelerationsMessageSubscriber()
   {
      PacketConsumer<ArmDesiredAccelerationsMessage> messageSubscriber = new PacketConsumer<ArmDesiredAccelerationsMessage>()
      {
         @Override
         public void receivedPacket(ArmDesiredAccelerationsMessage message)
         {
            if (!PacketValidityChecker.validateArmDesiredAccelerationsMessage(message, globalDataProducer))
               return;

            controllerCommandInputManager.submitArmDesiredAccelerationsMessage(message);
         }
      };
      globalDataProducer.attachListener(ArmDesiredAccelerationsMessage.class, messageSubscriber);
   }
}
