package us.ihmc.avatar.networkProcessor.modules;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber.MessageFilter;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.CommandInputManager.HasReceivedInputListener;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.communication.packets.TrackablePacket;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.graphics3DDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;

/**
 * This is a base class for any toolbox in the network manager. See the KinematicsToolboxModule as an example.
 */
public abstract class ToolboxModule
{
   protected static final boolean DEBUG = false;
   protected static final double YO_VARIABLE_SERVER_DT = 0.01;
   protected static final int UPDATE_PERIOD_MILLISECONDS = 1;

   protected final String name = getClass().getSimpleName();
   protected final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   protected final YoVariableRegistry registry = new YoVariableRegistry(name);
   protected final DoubleYoVariable yoTime = new DoubleYoVariable("localTime", registry);
   protected final FullHumanoidRobotModel desiredFullRobotModel;

   protected final PacketCommunicator packetCommunicator;
   protected final CommandInputManager commandInputManager;
   protected final StatusMessageOutputManager statusOutputManager;
   protected final ControllerNetworkSubscriber controllerNetworkSubscriber;
   private final int thisDesitination;

   protected final EnumYoVariable<PacketDestination> activeMessageSource = new EnumYoVariable<>("activeMessageSource", registry, PacketDestination.class, true);

   protected final ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(name);
   protected final ScheduledExecutorService executorService;
   protected ScheduledFuture<?> toolboxTaskScheduled = null;
   protected ScheduledFuture<?> yoVariableServerScheduled = null;
   protected Runnable toolboxRunnable = null;

   protected final DoubleYoVariable timeWithoutInputsBeforeGoingToSleep = new DoubleYoVariable("timeWithoutInputsBeforeGoingToSleep", registry);
   protected final DoubleYoVariable timeOfLastInput = new DoubleYoVariable("timeOfLastInput", registry);
   protected final AtomicBoolean receivedInput = new AtomicBoolean();
   private final LogModelProvider modelProvider;
   private final boolean startYoVariableServer;

   public ToolboxModule(FullHumanoidRobotModel desiredFullRobotModel, LogModelProvider modelProvider, boolean startYoVariableServer,
         PacketDestination toolboxDestination, NetworkPorts toolboxPort) throws IOException
   {
      this.modelProvider = modelProvider;
      this.startYoVariableServer = startYoVariableServer;
      this.thisDesitination = toolboxDestination.ordinal();
      this.desiredFullRobotModel = desiredFullRobotModel;
      packetCommunicator = PacketCommunicator.createIntraprocessPacketCommunicator(toolboxPort, new IHMCCommunicationKryoNetClassList());
      commandInputManager = new CommandInputManager(createListOfSupportedCommands());
      statusOutputManager = new StatusMessageOutputManager(createListOfSupportedStatus());
      controllerNetworkSubscriber = new ControllerNetworkSubscriber(commandInputManager, statusOutputManager, null, packetCommunicator);

      if (startYoVariableServer)
         executorService = Executors.newScheduledThreadPool(2, threadFactory);
      else
         executorService = Executors.newScheduledThreadPool(1, threadFactory);

      activeMessageSource.set(null);
      timeWithoutInputsBeforeGoingToSleep.set(0.5);
      commandInputManager.registerHasReceivedInputListener(new HasReceivedInputListener()
      {
         @Override
         public void hasReceivedInput()
         {
            receivedInput.set(true);
         }
      });

      controllerNetworkSubscriber.addMessageFilter(createMessageFilter());

      packetCommunicator.attachListener(ToolboxStateMessage.class, createToolboxStateMessageListener());
      packetCommunicator.connect();
   }

   protected void startYoVariableServer()
   {
      if (!startYoVariableServer)
         return;

      PeriodicThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler("WholeBodyIKScheduler");
      final YoVariableServer yoVariableServer = new YoVariableServer(getClass(), scheduler, modelProvider, LogSettings.TOOLBOX, YO_VARIABLE_SERVER_DT);
      yoVariableServer.setMainRegistry(registry, desiredFullRobotModel, yoGraphicsListRegistry);
      new Thread(new Runnable()
      {
         @Override
         public void run()
         {
            yoVariableServer.start();
         }
      }).start();

      yoVariableServerScheduled = executorService.scheduleAtFixedRate(createYoVariableServerRunnable(yoVariableServer), 0, UPDATE_PERIOD_MILLISECONDS,
            TimeUnit.MILLISECONDS);
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

            serverTime += UPDATE_PERIOD_MILLISECONDS;
            yoVariableServer.update(TimeTools.secondsToNanoSeconds(serverTime));
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
            if (message.getDestination() != thisDesitination)
            {
               System.err.println("ToolboxModule: isMessageValid " + message.getDestination() + "!=" + thisDesitination);
               return false;
            }

            if (message instanceof TrackablePacket)
            {
               TrackablePacket<?> trackableMessage = (TrackablePacket<?>) message;
               if (toolboxTaskScheduled == null)
               {
                  wakeUp(trackableMessage.getSource());
               }
               else if (activeMessageSource.getOrdinal() != trackableMessage.getSource())
               {
                  if (DEBUG)
                     PrintTools.error(ToolboxModule.this, "Expecting messages from " + activeMessageSource.getEnumValue() + " received message from: "
                           + PacketDestination.values[trackableMessage.getSource()]);
                  return false;
               }
            }
            else
            {
               if (DEBUG)
                  PrintTools.error(ToolboxModule.this, "Received a message from unknow source. Needs to implement: " + TrackablePacket.class.getSimpleName());
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
                  PrintTools.error(this, "Expecting messages from " + activeMessageSource.getEnumValue() + " received message from: "
                        + PacketDestination.values[message.getDestination()]);
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

   public void wakeUp(int packetDestination)
   {
      wakeUp(PacketDestination.values[packetDestination]);
   }

   public void wakeUp(PacketDestination packetDestination)
   {
      if (toolboxTaskScheduled != null)
      {
         if (DEBUG)
            PrintTools.error(this, "The IK controller is already running.");
         return;
      }
      createToolboxRunnable();
      toolboxTaskScheduled = executorService.scheduleAtFixedRate(toolboxRunnable, 0, UPDATE_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
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
      yoVariableServerScheduled.cancel(true);
      yoVariableServerScheduled = null;
      executorService.shutdownNow();
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
               yoTime.add(TimeTools.milliSecondsToSeconds(UPDATE_PERIOD_MILLISECONDS));

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

   abstract public ToolboxController<? extends StatusPacket<?>> getToolboxController();

   abstract public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands();

   abstract public List<Class<? extends StatusPacket<?>>> createListOfSupportedStatus();
}
