package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessagePubSubType;
import us.ihmc.idl.serializers.extra.JSONSerializer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

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
   private final List<WholeBodyTrajectoryMessage> wholeBodyTrajectoryMessages = new ArrayList<>();

   private final AtomicBoolean requestLogStart = new AtomicBoolean(false);
   private final AtomicBoolean requestLogStop = new AtomicBoolean(false);

   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private final JSONSerializer<WholeBodyTrajectoryMessage> wholeBodyTrajectorySerializer = new JSONSerializer<>(new WholeBodyTrajectoryMessagePubSubType());
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;

   public KinematicsStreamingLogger(YoRegistry registry)
   {
      isLogging = new YoBoolean("isLogging", registry);
   }

   public void update(WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage)
   {
      if (requestLogStart.getAndSet(false) && !isLogging.getValue())
      {
         wholeBodyTrajectoryMessages.clear();
         isLogging.set(true);
      }
      if (requestLogStop.getAndSet(false) && isLogging.getValue())
      {
         isLogging.set(false);
         export();
      }
      if (isLogging.getValue() && wholeBodyTrajectoryMessage != null)
      {
         wholeBodyTrajectoryMessages.add(new WholeBodyTrajectoryMessage(wholeBodyTrajectoryMessage));
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

                       for (int i = 0; i < wholeBodyTrajectoryMessages.size(); i++)
                       {
                          WholeBodyTrajectoryMessage message = wholeBodyTrajectoryMessages.get(i);
                          JsonNode jsonNode = objectMapper.readTree(wholeBodyTrajectorySerializer.serializeToString(message));
                          root.add(jsonNode);
                       }

                       objectMapper.writerWithDefaultPrettyPrinter().writeValue(printStream, root);
                    }
                    catch (Exception e)
                    {
                       e.printStackTrace();
                    }
                 }).start();
   }

   public void onLogRequestStart()
   {
      requestLogStart.set(true);
   }

   public void onLogRequestFinish()
   {
      requestLogStop.set(true);
   }

   public static void main(String[] args)
   {
      // run a test export

      KinematicsStreamingLogger logger = new KinematicsStreamingLogger(new YoRegistry("test"));
      logger.onLogRequestStart();

      WholeBodyTrajectoryMessage messageA = new WholeBodyTrajectoryMessage();
      WholeBodyTrajectoryMessage messageB = new WholeBodyTrajectoryMessage();

      messageA.setSequenceId(0);
      messageB.setSequenceId(1);

      logger.update(messageA);
      logger.update(messageB);

      logger.onLogRequestFinish();
      logger.update(null);
   }
}
