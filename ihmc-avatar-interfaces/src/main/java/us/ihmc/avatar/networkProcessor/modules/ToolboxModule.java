package us.ihmc.avatar.networkProcessor.modules;

import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber.MessageFilter;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationOptions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.CommandInputManager.HasReceivedInputListener;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.util.PeriodicThreadSchedulerFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.io.IOException;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * This is a base class for any toolbox in the network manager. See the KinematicsToolboxModule as an example.
 */
public abstract class ToolboxModule
{
   protected static final boolean DEBUG = false;
   protected static final double YO_VARIABLE_SERVER_DT = 0.01;
   protected static final int DEFAULT_UPDATE_PERIOD_MILLISECONDS = 1;

   protected final String name = getClass().getSimpleName();
   protected final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   protected final YoVariableRegistry registry = new YoVariableRegistry(name);
   protected final YoDouble yoTime = new YoDouble("localTime", registry);
   protected final FullHumanoidRobotModel fullRobotModel;

   protected final PacketCommunicator packetCommunicator;
   protected final RealtimeRos2Node realtimeRos2Node;
   protected final CommandInputManager commandInputManager;
   protected final StatusMessageOutputManager statusOutputManager;
   protected final ControllerNetworkSubscriber controllerNetworkSubscriber;
   private final int thisDesitination;

   protected final YoEnum<PacketDestination> activeMessageSource = new YoEnum<>("activeMessageSource", registry, PacketDestination.class, true);

   protected final ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(name);
   protected final ScheduledExecutorService executorService;
   protected ScheduledFuture<?> toolboxTaskScheduled = null;
   protected ScheduledFuture<?> yoVariableServerScheduled = null;
   protected Runnable toolboxRunnable = null;
   protected final int updatePeriodMilliseconds = 1;

   protected final YoDouble timeWithoutInputsBeforeGoingToSleep = new YoDouble("timeWithoutInputsBeforeGoingToSleep", registry);
   protected final YoDouble timeOfLastInput = new YoDouble("timeOfLastInput", registry);
   protected final AtomicBoolean receivedInput = new AtomicBoolean();
   private final LogModelProvider modelProvider;
   private final boolean startYoVariableServer;
   private YoVariableServer yoVariableServer;

   public ToolboxModule(FullHumanoidRobotModel fullRobotModelToLog, LogModelProvider modelProvider, boolean startYoVariableServer,
         PacketDestination toolboxDestination, NetworkPorts toolboxPort) throws IOException
   {
      this(fullRobotModelToLog, modelProvider, startYoVariableServer, toolboxDestination, toolboxPort, DEFAULT_UPDATE_PERIOD_MILLISECONDS);
   }

   public ToolboxModule(FullHumanoidRobotModel fullRobotModelToLog, LogModelProvider modelProvider, boolean startYoVariableServer,
         PacketDestination toolboxDestination, NetworkPorts toolboxPort, int updatePeriodMilliseconds) throws IOException
   {
      this.modelProvider = modelProvider;
      this.startYoVariableServer = startYoVariableServer;
      this.thisDesitination = toolboxDestination.ordinal();
      this.fullRobotModel = fullRobotModelToLog;
      packetCommunicator = CommunicationOptions.USE_KRYO ?
            PacketCommunicator.createIntraprocessPacketCommunicator(toolboxPort, new IHMCCommunicationKryoNetClassList()) :
            null;
      realtimeRos2Node = CommunicationOptions.USE_ROS2 ?
            ROS2Tools.createRealtimeRos2Node(PubSubImplementation.INTRAPROCESS, this.getClass().getSimpleName(), ROS2Tools.RUNTIME_EXCEPTION) :
            null;
      commandInputManager = new CommandInputManager(name, createListOfSupportedCommands());
      statusOutputManager = new StatusMessageOutputManager(createListOfSupportedStatus());
      controllerNetworkSubscriber = new ControllerNetworkSubscriber(commandInputManager, statusOutputManager, null, packetCommunicator, realtimeRos2Node);


      executorService = Executors.newScheduledThreadPool(1, threadFactory);

      activeMessageSource.set(null);
      timeWithoutInputsBeforeGoingToSleep.set(0.5);
      commandInputManager.registerHasReceivedInputListener(new HasReceivedInputListener()
      {
         private final Set<Class<? extends Command<?, ?>>> silentCommands = silentCommands();

         @Override
         public void hasReceivedInput(Class<? extends Command<?, ?>> commandClass)
         {
            if (!silentCommands.contains(commandClass))
               receivedInput.set(true);
         }
      });

      controllerNetworkSubscriber.addMessageFilter(createMessageFilter());

      packetCommunicator.attachListener(ToolboxStateMessage.class, createToolboxStateMessageListener());
      packetCommunicator.connect();
   }

   protected void setTimeWithoutInputsBeforeGoingToSleep(double time)
   {
      timeWithoutInputsBeforeGoingToSleep.set(time);
   }

   protected void startYoVariableServer()
   {
      if (!startYoVariableServer)
         return;

      PeriodicThreadSchedulerFactory scheduler = new PeriodicNonRealtimeThreadSchedulerFactory();
      yoVariableServer = new YoVariableServer(getClass(), scheduler, modelProvider, LogSettings.TOOLBOX, YO_VARIABLE_SERVER_DT);
      yoVariableServer.setMainRegistry(registry, fullRobotModel.getElevator(), yoGraphicsListRegistry);
      startYoVariableServerOnAThread(yoVariableServer);

      yoVariableServerScheduled = executorService.scheduleAtFixedRate(createYoVariableServerRunnable(yoVariableServer), 0, updatePeriodMilliseconds,
            TimeUnit.MILLISECONDS);
   }

   private void startYoVariableServerOnAThread(final YoVariableServer yoVariableServer)
   {
      new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            yoVariableServer.start();
         }
      }).start();
   }

   private Runnable createYoVariableServerRunnable(final YoVariableServer yoVariableServer)
   {
      return new Runnable()
      {
         double serverTime = 0.0;

         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;

            serverTime += Conversions.millisecondsToSeconds(updatePeriodMilliseconds);
            yoVariableServer.update(Conversions.secondsToNanoseconds(serverTime));
         }
      };
   }

   public MessageFilter createMessageFilter()
   {
      return new MessageFilter()
      {
         private final Set<Class<? extends Packet<?>>> exceptions = filterExceptions();

         @Override
         public boolean isMessageValid(Packet<?> message)
         {
            if (exceptions.contains(message.getClass()))
            {
               if (toolboxTaskScheduled == null)
               {
                  if (DEBUG)
                     PrintTools.info(ToolboxModule.this, name + " is sleeping: " + message.getClass().getSimpleName() + " is ignored.");
                  return false;
               }
               else
               {
                  return true;
               }
            }

            if (message.getDestination() != thisDesitination)
            {
               if (DEBUG)
                  PrintTools.error(ToolboxModule.this, name + ": isMessageValid " + message.getDestination() + "!=" + thisDesitination);
               return false;
            }

            if (toolboxTaskScheduled == null)
            {
               wakeUp(message.getSource());
            }
            else if (activeMessageSource.getOrdinal() != message.getSource())
            {
               if (DEBUG)
                  PrintTools.error(ToolboxModule.this, "Expecting messages from " + activeMessageSource.getEnumValue() + " received message from: "
                        + PacketDestination.values[message.getSource()]);
               return false;
            }

            return true;
         }
      };
   }

   private PacketConsumer<ToolboxStateMessage> createToolboxStateMessageListener()
   {
      return new PacketConsumer<ToolboxStateMessage>()
      {
         @Override
         public void receivedPacket(ToolboxStateMessage message)
         {
            if (toolboxTaskScheduled != null && activeMessageSource.getOrdinal() != message.getSource())
            {
               if (DEBUG)
                  PrintTools.error(ToolboxModule.this, "Expecting messages from " + activeMessageSource.getEnumValue() + " received message from: "
                        + PacketDestination.values[message.getDestination()]);
               return;
            }

            switch (ToolboxState.fromByte(message.getRequestedToolboxState()))
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

   public void wakeUp(int packetDestination)
   {
      wakeUp(PacketDestination.values[packetDestination]);
   }

   public void wakeUp(PacketDestination packetDestination)
   {
      if (toolboxTaskScheduled != null)
      {
         if (DEBUG)
            PrintTools.error(this, "This toolbox is already running.");
         return;
      }

      if (DEBUG)
         PrintTools.debug(this, "Waking up");

      createToolboxRunnable();
      toolboxTaskScheduled = executorService.scheduleAtFixedRate(toolboxRunnable, 0, updatePeriodMilliseconds, TimeUnit.MILLISECONDS);
      reinitialize();
      activeMessageSource.set(packetDestination);
      getToolboxController().setPacketDestination(packetDestination);
      receivedInput.set(true);
   }

   private void reinitialize()
   {
      getToolboxController().requestInitialize();
   }

   public void sleep()
   {

      if (DEBUG)
         PrintTools.debug(this, "Going to sleep");

      destroyToolboxRunnable();
      activeMessageSource.set(null);

      if (toolboxTaskScheduled == null)
      {
         if (DEBUG)
            PrintTools.error(this, "There is no task running.");
         return;
      }

      toolboxTaskScheduled.cancel(true);
      toolboxTaskScheduled = null;
   }

   public void destroy()
   {
      sleep();

      if (yoVariableServerScheduled != null)
      {
         yoVariableServerScheduled.cancel(true);
         yoVariableServerScheduled = null;
      }
      executorService.shutdownNow();
      packetCommunicator.closeConnection();
      packetCommunicator.disconnect();

      if (yoVariableServer != null)
      {
         yoVariableServer.close();
         yoVariableServer = null;
      }
      controllerNetworkSubscriber.closeAndDispose();

      if (DEBUG)
         PrintTools.debug(this, "Destroyed");
   }

   private void createToolboxRunnable()
   {
      if (toolboxRunnable != null)
      {
         if (DEBUG)
            PrintTools.error(this, "toolboxRunnable is not null.");
         return;
      }

      toolboxRunnable = new Runnable()
      {
         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;

            try
            {
               getToolboxController().update();
               controllerNetworkSubscriber.run();
               yoTime.add(Conversions.millisecondsToSeconds(updatePeriodMilliseconds));

               if (receivedInput.getAndSet(false))
                  timeOfLastInput.set(yoTime.getDoubleValue());
               if (yoTime.getDoubleValue() - timeOfLastInput.getDoubleValue() >= timeWithoutInputsBeforeGoingToSleep.getDoubleValue())
                  sleep();
               else if (getToolboxController().isDone())
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

   private void destroyToolboxRunnable()
   {
      toolboxRunnable = null;
   }

   abstract public ToolboxController getToolboxController();

   /**
    * @return used to create the {@link CommandInputManager} and to defines the input API.
    */
   abstract public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands();

   /**
    * @return used to create the {@link StatusMessageOutputManager} and to defines the output API.
    */
   abstract public List<Class<? extends Packet<?>>> createListOfSupportedStatus();
   
   /**
    * @return the collection of commands that cannot wake up this module.
    */
   public Set<Class<? extends Command<?, ?>>> silentCommands()
   {
      return Collections.emptySet();
   }

   /**
    * @return the collection of messages that are allowed to go through the message filter.
    */
   public Set<Class<? extends Packet<?>>> filterExceptions()
   {
      return Collections.emptySet();
   }
}
