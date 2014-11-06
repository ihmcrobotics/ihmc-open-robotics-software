package us.ihmc.robotDataCommunication.logger;

import java.io.IOException;
import java.net.InetAddress;

import org.zeromq.ZMQ;
import org.zeromq.ZMQ.Socket;
import org.zeromq.ZMQException;

import us.ihmc.robotDataCommunication.generated.YoVariableLoggerRequestProto.YoVariableLoggerRequest;
import us.ihmc.utilities.net.NetworkTools;

import com.google.protobuf.InvalidProtocolBufferException;
import com.martiansoftware.jsap.JSAPException;

public class YoVariableLoggerDispatcher
{
   private static final int port = 5560;

   private static byte STARTED = 0x12;
   private static byte FAILED = 0x13;


   public YoVariableLoggerDispatcher(YoVariableLoggerOptions options)
   {
      System.out.println("Starting YoVariableLoggerDispatcher");
      System.out.println("Opening socket on port " + port);
      ZMQ.Context context = ZMQ.context(1);
      ZMQ.Socket socket = context.socket(ZMQ.REP);
      socket.bind("tcp://*:" + port);

      System.out.println("Socket opened, waiting for requests");
      while (true)
      {
         try
         {
            System.out.println("Waiting for new request");
            byte[] req = socket.recv();
            System.out.println("Received request");
            if (req == null)
            {
               break;
            }
            else
            {
               try
               {
                  YoVariableLoggerRequest request = YoVariableLoggerRequest.parseFrom(req);
                  System.out.println("Request new logging session for " + request.getHost() + ":" + request.getLogName());
                  try
                  {
                     YoVariableLogger logger = new YoVariableLogger(request.getLogName(), request.getHost(), YoVariableLogger.defaultPort, options);
                     socket.send(new byte[] { STARTED });
                     System.out.println("Logging session started");
                     logger.waitFor();
                     
                  }
                  catch (IOException e)
                  {
                     socket.send(new byte[] { FAILED });
                     System.err.println("Failed to start logging session");
                  }

               }
               catch (InvalidProtocolBufferException e)
               {
                  System.err.println("Invalid request " + e.getMessage());
                  socket.send(new byte[0]);
               }

            }
         }
         catch (ZMQException e)
         {
            if (e.getErrorCode() == ZMQ.Error.ETERM.getCode())
            {
               break;
            }
            else
            {
               throw e;
            }
         }

         System.err.println("Logger has shut down");
      }
   }

   public static void requestLogSession(String host, String logName)
   {
      System.out.println("Requesting logging session from " + host);
      InetAddress myIP;
      try
      {
         myIP = NetworkTools.getMyIP(host, port);
      }
      catch (IOException e)
      {
         System.err.println("Cannot connect to logging computer " + host + ", continuing without logging");
         return;
      }
      
      ZMQ.Context context = ZMQ.context(1);
      Socket socket = context.socket(ZMQ.REQ);
      socket.connect("tcp://" + host + ":" + port);
      
      YoVariableLoggerRequest.Builder request = YoVariableLoggerRequest.newBuilder();
      request.setHost(myIP.getHostAddress());
      request.setLogName(logName);
      socket.send(request.build().toByteArray());
      
      System.out.println("Waiting for logging session");
      byte[] result = socket.recv();
      
      if(result[0] == STARTED)
      {
         System.out.println("Logging session started");
      }
      else
      {
         System.err.println("Cannot start logging session, continuing without.");
      }
   }

   public static void main(String[] args) throws JSAPException
   {
      YoVariableLoggerOptions options = YoVariableLoggerOptions.parse(args);      
      new YoVariableLoggerDispatcher(options);
   }

}
