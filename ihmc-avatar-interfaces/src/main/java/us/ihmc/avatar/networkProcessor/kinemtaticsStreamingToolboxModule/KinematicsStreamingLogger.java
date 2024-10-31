package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import controller_msgs.msg.dds.WholeBodyStreamingMessagePubSubType;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessagePubSubType;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class KinematicsStreamingLogger
{
   private final YoBoolean isLogging;
   private final YoInteger messagesLogged;

   private final List<Object> messagesToLog = new ArrayList<>();

   private final AtomicBoolean requestLogStart = new AtomicBoolean(false);

   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private final JSONSerializer<WholeBodyTrajectoryMessage> wholeBodyTrajectorySerializer = new JSONSerializer<>(new WholeBodyTrajectoryMessagePubSubType());
   private final JSONSerializer<WholeBodyStreamingMessage> wholeBodyStreamingSerializer = new JSONSerializer<>(new WholeBodyStreamingMessagePubSubType());
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;

   public KinematicsStreamingLogger(YoRegistry registry)
   {
      isLogging = new YoBoolean("isLogging", registry);
      messagesLogged = new YoInteger("messagesLogged", registry);
   }

   public void update(Object messageToLog)
   {
      if (requestLogStart.getAndSet(false) && !isLogging.getValue())
      {
         LogTools.info("Starting to log...");
         messagesToLog.clear();
         isLogging.set(true);
         messagesLogged.set(0);
      }

      if (isLogging.getValue() && messageToLog != null)
      {
         if (messageToLog instanceof WholeBodyTrajectoryMessage  wholeBodyTrajectoryMessage)
            messagesToLog.add(new WholeBodyTrajectoryMessage(wholeBodyTrajectoryMessage));
         else if (messageToLog instanceof WholeBodyStreamingMessage wholeBodyStreamingMessage)
            messagesToLog.add(new WholeBodyStreamingMessage(wholeBodyStreamingMessage));
         messagesLogged.set(messagesToLog.size());
      }
   }

   private void export()
   {
      new Thread(() ->
                 {
                    try
                    {
                       String fileName = logDirectory + dateFormat.format(new Date()) + "_" + "KinematicsStreamingToolbox.json";
                       FileOutputStream outputStream = new FileOutputStream(fileName);
                       PrintStream printStream = new PrintStream(outputStream);

                       JsonFactory jsonFactory = new JsonFactory();
                       ObjectMapper objectMapper = new ObjectMapper(jsonFactory);
                       ArrayNode root = objectMapper.createArrayNode();

                       LogTools.info("Exporting " + messagesToLog.size() + " messages");

                       for (int i = 0; i < messagesToLog.size(); i++)
                       {
                          Object message = messagesToLog.get(i);
                          if (message instanceof WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage)
                          {
                             root.add(objectMapper.readTree(wholeBodyTrajectorySerializer.serializeToString(wholeBodyTrajectoryMessage)));
                          }
                          else if (message instanceof WholeBodyStreamingMessage wholeBodyStreamingMessage)
                          {
                             root.add(objectMapper.readTree(wholeBodyStreamingSerializer.serializeToString(wholeBodyStreamingMessage)));
                          }
                       }

                       objectMapper.writerWithDefaultPrettyPrinter().writeValue(printStream, root);

                       printStream.flush();
                       outputStream.flush();
                       printStream.close();
                       outputStream.close();
                    }
                    catch (Exception e)
                    {
                       LogTools.info("Log unsuccessful");
                       e.printStackTrace();
                    }
                 }).start();
   }

   public void onLogRequestStart()
   {
      LogTools.info("Requesting to start log");
      requestLogStart.set(true);
   }

   public void onLogRequestFinish()
   {
      LogTools.info("Stopping log and exporting");
      isLogging.set(false);
      export();
   }

   public static void main(String[] args)
   {
      // run a test export

      KinematicsStreamingLogger logger = new KinematicsStreamingLogger(new YoRegistry("test"));
      logger.onLogRequestStart();

      WholeBodyTrajectoryMessage messageA = new WholeBodyTrajectoryMessage();
      WholeBodyStreamingMessage messageB = new WholeBodyStreamingMessage();

      messageA.setSequenceId(0);
      messageB.setSequenceId(1);

      logger.update(messageA);
      logger.update(messageB);

      logger.onLogRequestFinish();
      logger.update(null);
   }
}
