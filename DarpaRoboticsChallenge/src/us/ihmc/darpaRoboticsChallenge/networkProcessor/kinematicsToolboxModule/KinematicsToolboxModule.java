package us.ihmc.darpaRoboticsChallenge.networkProcessor.kinematicsToolboxModule;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber.MessageFilter;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.CommandInputManager.HasReceivedInputListener;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.communication.packets.KinematicsToolboxStateMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.communication.packets.TrackablePacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.WholeBodyTrajectoryCommand;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotDataCommunication.logger.LogSettings;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;

public class KinematicsToolboxModule
{
   private static final boolean DEBUG = false;
   private static final double YO_VARIABLE_SERVER_DT = 0.01;
   private static final int IK_UPDATE_PERIOD_MILLISECONDS = 1;
   private static final int THIS_DESTINATION = PacketDestination.KINEMATICS_TOOLBOX_MODULE.ordinal();

   private final String name = getClass().getSimpleName();
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final DoubleYoVariable yoTime = new DoubleYoVariable("localTime", registry);
   private final YoVariableServer yoVariableServer;

   private final DoubleYoVariable timeWithoutInputsBeforeGoingToSleep = new DoubleYoVariable("timeWithoutInputsBeforeGoingToSleep", registry);
   private final DoubleYoVariable timeOfLastInput = new DoubleYoVariable("timeOfLastInput", registry);
   private final AtomicBoolean receivedInput = new AtomicBoolean();

   private final EnumYoVariable<PacketDestination> activeMessageSource = new EnumYoVariable<>("activeMessageSource", registry, PacketDestination.class, true);

   private final ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(name);
   private final ScheduledExecutorService executorService;
   private ScheduledFuture<?> ikScheduled = null;
   private ScheduledFuture<?> yoVariableServerScheduled = null;
   private Runnable inverseKinematicsRunnable = null;

   private final PacketCommunicator packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(NetworkPorts.KINEMATICS_TOOLBOX_MODULE_PORT, new IHMCCommunicationKryoNetClassList());

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final ControllerNetworkSubscriber controllerNetworkSubscriber;
   private final KinematicsToolboxController kinematicsToolBoxController;

   public KinematicsToolboxModule(FullHumanoidRobotModelFactory fullRobotModelFactory, LogModelProvider modelProvider,
         boolean startYoVariableServer) throws IOException
   {
      commandInputManager = new CommandInputManager(createListOfSupportedCommands());
      statusOutputManager = new StatusMessageOutputManager(createListOfSupportedStatus());
      controllerNetworkSubscriber = new ControllerNetworkSubscriber(commandInputManager, statusOutputManager, null, packetCommunicator);
      kinematicsToolBoxController = new KinematicsToolboxController(commandInputManager, statusOutputManager, fullRobotModelFactory, yoGraphicsListRegistry, registry);

      timeWithoutInputsBeforeGoingToSleep.set(0.5);
      commandInputManager.registerHasReceivedInputListener(new HasReceivedInputListener()
      {
         @Override
         public void hasReceivedInput()
         {
            receivedInput.set(true);
         }
      });

      activeMessageSource.set(null);

      controllerNetworkSubscriber.addMessageFilter(createMessageFilter());

      packetCommunicator.attachListener(RobotConfigurationData.class, kinematicsToolBoxController.createRobotConfigurationDataConsumer());
      packetCommunicator.attachListener(KinematicsToolboxStateMessage.class, createWholeBodyInverseKinematicsStateMessageListener());
      packetCommunicator.connect();

      if (startYoVariableServer)
      {
         executorService = Executors.newScheduledThreadPool(2, threadFactory);

         PeriodicThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler("WholeBodyIKScheduler");
         yoVariableServer = new YoVariableServer(getClass(), scheduler, modelProvider, LogSettings.KINEMATICS_TOOLBOX, YO_VARIABLE_SERVER_DT);
         yoVariableServer.setMainRegistry(registry, kinematicsToolBoxController.getDesiredFullRobotModel(), yoGraphicsListRegistry);
         new Thread(new Runnable()
         {
            @Override
            public void run()
            {
               yoVariableServer.start();
            }
         }).start();

         yoVariableServerScheduled = executorService.scheduleAtFixedRate(createYoVariableServerRunnable(), 0, IK_UPDATE_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
      else
      {
         executorService = Executors.newScheduledThreadPool(1, threadFactory);
         yoVariableServer = null;
      }
   }

   private PacketConsumer<KinematicsToolboxStateMessage> createWholeBodyInverseKinematicsStateMessageListener()
   {
      return new PacketConsumer<KinematicsToolboxStateMessage>()
      {
         @Override
         public void receivedPacket(KinematicsToolboxStateMessage message)
         {
            if (ikScheduled != null && activeMessageSource.getOrdinal() != message.getSource())
            {
               if (DEBUG)
                  PrintTools.error(this, "Expecting messages from " + activeMessageSource.getEnumValue() + " received message from: " + PacketDestination.values[message.getDestination()]);
               return;
            }

            switch (message.getRequestedState())
            {
            case WAKE_UP:
               wakeUp(message.getSource());
               break;
            case REINITIALIZE:
               reinitialize();
               break;
            case SLEEP:
               sleep();
               break;
            }
         }
      };
   }

   public MessageFilter createMessageFilter()
   {
      return new MessageFilter()
      {
         @Override
         public boolean isMessageValid(Packet<?> message)
         {
            if (message.getDestination() != THIS_DESTINATION)
               return false;

            if (message instanceof TrackablePacket)
            {
               TrackablePacket<?> trackableMessage = (TrackablePacket<?>) message;
               if (ikScheduled == null)
               {
                  wakeUp(trackableMessage.getSource());
               }
               else if (activeMessageSource.getOrdinal() != trackableMessage.getSource())
               {
                  if (DEBUG)
                     PrintTools.error(KinematicsToolboxModule.this, "Expecting messages from " + activeMessageSource.getEnumValue() + " received message from: " + PacketDestination.values[trackableMessage.getSource()]);
                  return false;
               }
            }
            else
            {
               if (DEBUG)
                  PrintTools.error(KinematicsToolboxModule.this, "Received a message from unknow source. Needs to implement: " + TrackablePacket.class.getSimpleName());
               return false;
            }

            return true;
         }
      };
   }

   public void wakeUp(int packetDestination)
   {
      wakeUp(PacketDestination.values[packetDestination]);
   }

   public void wakeUp(PacketDestination packetDestination)
   {
      if (ikScheduled != null)
      {
         if (DEBUG)
            PrintTools.error(this, "The IK controller is already running.");
         return;
      }
      createInverseKinematicsRunnable();
      ikScheduled = executorService.scheduleAtFixedRate(inverseKinematicsRunnable, 0, IK_UPDATE_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      reinitialize();
      activeMessageSource.set(packetDestination);
      kinematicsToolBoxController.setPacketDestination(packetDestination);
      receivedInput.set(true);
   }

   private void reinitialize()
   {
      kinematicsToolBoxController.requestInitialize();
   }

   public void sleep()
   {
      destroyInverseKinematicsRunnable();
      activeMessageSource.set(null);

      if (ikScheduled == null)
      {
         if (DEBUG)
            PrintTools.error(this, "There is no task running.");
         return;
      }

      ikScheduled.cancel(true);
      ikScheduled = null;
   }

   public void destroy()
   {
      sleep();
      yoVariableServerScheduled.cancel(true);
      yoVariableServerScheduled = null;
      executorService.shutdownNow();
   }

   private void createInverseKinematicsRunnable()
   {
      if (inverseKinematicsRunnable != null)
      {
         if (DEBUG)
            PrintTools.error(this, "inverseKinematicsRunnable is not null.");
         return;
      }

      inverseKinematicsRunnable = new Runnable()
      {
         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;

            try
            {
               kinematicsToolBoxController.update();
               controllerNetworkSubscriber.run();
               yoTime.add(TimeTools.milliSecondsToSeconds(IK_UPDATE_PERIOD_MILLISECONDS));

               if (receivedInput.getAndSet(false))
                  timeOfLastInput.set(yoTime.getDoubleValue());
               if (yoTime.getDoubleValue() - timeOfLastInput.getDoubleValue() >= timeWithoutInputsBeforeGoingToSleep.getDoubleValue())
                  sleep();
            }
            catch (Exception e)
            {
               e.printStackTrace();
               sleep();
               throw e;
            }
         }
      };
   }

   private Runnable createYoVariableServerRunnable()
   {
      return new Runnable()
      {
         double serverTime = 0.0;
         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;

            serverTime += IK_UPDATE_PERIOD_MILLISECONDS;
            yoVariableServer.update(TimeTools.secondsToNanoSeconds(serverTime));
         }
      };
   }

   private void destroyInverseKinematicsRunnable()
   {
      inverseKinematicsRunnable = null;
   }

   public static List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(ArmTrajectoryCommand.class);
      commands.add(HandTrajectoryCommand.class);
      commands.add(FootTrajectoryCommand.class);
      commands.add(ChestTrajectoryCommand.class);
      commands.add(PelvisTrajectoryCommand.class);
      commands.add(WholeBodyTrajectoryCommand.class);
      return commands;
   }

   public static List<Class<? extends StatusPacket<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends StatusPacket<?>>> status = new ArrayList<>();
      status.add(KinematicsToolboxOutputStatus.class);
      return status;
   }
}
