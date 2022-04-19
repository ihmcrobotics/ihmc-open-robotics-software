package us.ihmc.robotEnvironmentAwareness.communication.converters;

import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.Collections;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.esotericsoftware.minlog.Log;
import org.apache.commons.lang3.mutable.MutableBoolean;

import com.esotericsoftware.kryonet.Client;
import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import com.esotericsoftware.kryonet.Server;

import org.apache.logging.log4j.Level;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.RunnableThatThrows;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.messager.kryo.UnmodifiableListSerializer;

/**
 * <p>
 * Unifies the API of Kryonet Server and Client. Users should aim to create one server and one
 * client.
 * </p>
 * <p>
 * Uses lamdas and callbacks pretty heavily since Kryonet Client and Server do not share any
 * interfaces.
 * </p>
 */
public class KryoAdapter
{
   private Server server;
   private Client client;

   private Listener kryoListener = new KryoListener();
   private Consumer receivedConsumer;
   private final ArrayList<Consumer> connectionStateListeners = new ArrayList<>();

   private final BooleanSupplier isConnectedSupplier;
   private final RunnableThatThrows updater;
   private final RunnableThatThrows connector;
   private final RunnableThatThrows disconnector;
   private final Consumer tcpSender;
   private final Supplier<InetSocketAddress> remoteAddressSupplier;

   private enum Type { Server, Client }
   private final Type type;

   static
   {
      Level log4jLevel = LogTools.getLevel();
      int minLogLevel = 4;
      switch (log4jLevel.getStandardLevel())
      {
         case OFF:
            minLogLevel = 6;
            break;
         case FATAL:
         case ERROR:
            minLogLevel = 5;
            break;
         case WARN:
         case INFO: // Kryonet info is pretty verbose
            minLogLevel = 4;
            break;
         case DEBUG:
            minLogLevel = 2;
            break;
         case TRACE:
         case ALL:
            minLogLevel = 1;
            break;
      }
      Log.set(minLogLevel); // Set minlog level
   }

   /**
    * Create a Kryonet server.
    *
    * @param tcpPort
    * @return server
    */
   public static KryoAdapter createServer(int tcpPort)
   {
      return new KryoAdapter(tcpPort);
   }

   /**
    * Create a Kryonet client.
    *
    * @param serverAddress
    * @param tcpPort
    * @return client
    */
   public static KryoAdapter createClient(String serverAddress, int tcpPort)
   {
      return new KryoAdapter(serverAddress, tcpPort);
   }

   private KryoAdapter(int tcpPort)
   {
      server = new Server(Conversions.megabytesToBytes(2048), Conversions.megabytesToBytes(256));
      server.addListener(kryoListener);
      server.getKryo().setRegistrationRequired(false);
      server.getKryo().addDefaultSerializer(Collections.unmodifiableList(Collections.emptyList()).getClass(), UnmodifiableListSerializer.class);
      isConnectedSupplier = () -> server.getConnections().size() > 0;
      updater = () -> server.update(250);
      connector = () -> server.bind(tcpPort);
      disconnector = () -> server.close();
      tcpSender = message -> server.sendToAllTCP(message);
      remoteAddressSupplier = () -> server.getConnections().stream().findFirst().get().getRemoteAddressTCP();
      type = Type.Server;
   }

   public Server getServer()
   {
      return server;
   }

   private KryoAdapter(String serverAddress, int tcpPort)
   {
      client = new Client(Conversions.megabytesToBytes(64), Conversions.megabytesToBytes(32));
      client.addListener(kryoListener);
      client.getKryo().setRegistrationRequired(false);
      client.getKryo().addDefaultSerializer(Collections.unmodifiableList(Collections.emptyList()).getClass(), UnmodifiableListSerializer.class);
      isConnectedSupplier = () -> client.isConnected();
      updater = () -> client.update(250);
      connector = () -> client.connect(5000, serverAddress, tcpPort);
      disconnector = () -> client.close();
      tcpSender = message -> client.sendTCP(message);
      remoteAddressSupplier = () -> client.getRemoteAddressTCP();
      type = Type.Client;
   }

   class KryoListener implements Listener
   {
      @Override
      public void received(Connection connection, Object object)
      {
         receivedConsumer.accept(object);
      }

      @Override
      public void connected(Connection connection)
      {
         connectionStateListeners.forEach(connectionStateListener -> connectionStateListener.accept(true));
      }

      @Override
      public void disconnected(Connection connection)
      {
         connectionStateListeners.forEach(connectionStateListener -> connectionStateListener.accept(false));
      }
   }

   /**
    * Connect across to an expected instance of Kryo that is of the opposite type. If you created a
    * server, you need to connect to a client.
    */
   public void connect()
   {
      // this is the "kickstart" method required to get Kryo to connect
      ThreadTools.startAsDaemon(this::startNonBlockingConnect, getClass().getSimpleName() + "NonBlockingConnect");
      ThreadTools.startAsDaemon(this::waitForConnection, getClass().getSimpleName() + "WaitForConnection");
   }

   private void startNonBlockingConnect()
   {
      LogTools.debug("Connecting...");
      MutableBoolean successful = new MutableBoolean(false);
      while (!successful.getValue())
      {
         successful.setValue(true);
         ExceptionTools.handle(connector, e ->
         {
            if (e.getMessage().contains("Address already in use"))
            {
               LogTools.error(e.getMessage());
               LogTools.info("Trying to connect again...");
            }
            else
            {
               LogTools.trace(e.getMessage());
               LogTools.trace("Trying to connect again...");
            }
            successful.setFalse();
         });

         if (!successful.getValue())
         {
            ThreadTools.sleep(200); // prevent free spinning loop
         }
      }
   }

   private void waitForConnection()
   {
      while (!isConnectedSupplier.getAsBoolean())
      {
         LogTools.trace("Updating...");
         ExceptionTools.handle(updater, DefaultExceptionHandler.RUNTIME_EXCEPTION);
      }

      LogTools.info(type.name() + " connected to " + remoteAddressSupplier.get());
   }

   /**
    * Closes all open connections and the server port(s) if applicable. Doesn't seem to free up much
    * memory.
    */
   public void disconnect()
   {
      ExceptionTools.handle(disconnector, DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   /**
    * <p>
    * Update that must be called to receive any data and, for a server, to accept new connections.
    * </p>
    * <p>
    * For a server: Accepts any new connections and reads or writes any pending data for the current
    * connections. Wait for up to 250 milliseconds for a connection to be ready to process. May be zero
    * to return immediately if there are no connections to process.
    * </p>
    * <br>
    * <p>
    * For a client: Reads or writes any pending data for this client. Multiple threads should not call
    * this method at the same time. Wait for up to 250 milliseconds for data to be ready to process.
    * May be zero to return immediately if there is no data to process.
    * </p>
    */
   public void update()
   {
      ExceptionTools.handle(updater, DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   /**
    * Serializes and sends the object over the network using TCP. Non-blocking.
    *
    * @param object to send
    */
   public void sendTCP(Object object)
   {
      tcpSender.accept(object);
   }

   /**
    * If this adapter is connected to a server or client.
    *
    * @return connected
    */
   public boolean isConnected()
   {
      return isConnectedSupplier.getAsBoolean();
   }

   /**
    * Subscribe to received messages.
    *
    * @param receivedConsumer
    */
   public void setReceivedListener(Consumer receivedConsumer)
   {
      this.receivedConsumer = receivedConsumer;
   }

   /**
    * Add a connection state listener. Will callback on connected and disconnected events.
    *
    * @param connectionStateListener
    */
   public void addConnectionStateListener(Consumer<Boolean> connectionStateListener)
   {
      connectionStateListeners.add(connectionStateListener);
   }

   public boolean removeConnectionStateListener(Consumer<Boolean> connectionStateListener)
   {
      return connectionStateListeners.remove(connectionStateListener);
   }
}
