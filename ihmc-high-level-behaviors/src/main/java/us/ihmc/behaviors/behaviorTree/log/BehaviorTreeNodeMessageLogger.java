package us.ihmc.behaviors.behaviorTree.log;

import behavior_msgs.msg.dds.BehaviorTreeLogMessage;
import org.apache.logging.log4j.Level;
import org.apache.logging.log4j.message.ParameterizedMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalField;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.log.LogTools;
import us.ihmc.log.LogToolsWriteOnly;

import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.util.LinkedList;
import java.util.function.Supplier;

/**
 * An unidirectional CRDT for log messages.
 */
public class BehaviorTreeNodeMessageLogger extends CRDTUnidirectionalField implements LogToolsWriteOnly
{
   public record LogMessage(Instant instant, Level level, String message) { }
   private final LinkedList<LogMessage> recentMessages = new LinkedList<>();
   private Instant lastPrintedTimestamp = Instant.now();

   public BehaviorTreeNodeMessageLogger(CRDTInfo crdtInfo)
   {
      super(ROS2ActorDesignation.ROBOT, crdtInfo);
   }

   private void queueLogMessage(Level level, String message)
   {
      checkActorCanModify();

      BehaviorTreeLogMessage logMessage = new BehaviorTreeLogMessage();
      MessageTools.toMessage(Instant.now(), logMessage.getInstant());
      logMessage.setLogLevel(MessageTools.toMessage(level));
      MessageTools.packLongStringToByteSequence(message, logMessage.getLogMessage());

      recentMessages.add(new LogMessage(Instant.now(), level, message));
   }

   public void toMessage(us.ihmc.idl.IDLSequence.Object<BehaviorTreeLogMessage> recentLogMessages)
   {
      if (isRobotSide())
      {
         // Clear old message from sync
         Instant fiveSecondsAgo = Instant.now().minus(5, ChronoUnit.SECONDS);
         while (!recentMessages.isEmpty() && recentMessages.peek().instant().isBefore(fiveSecondsAgo))
         {
            recentMessages.poll();
         }

         recentLogMessages.clear();
         for (int i = 0; i < recentMessages.size(); i++)
         {
            LogMessage localMessage = recentMessages.get(i);
            BehaviorTreeLogMessage ddsMessage = recentLogMessages.add();
            MessageTools.toMessage(localMessage.instant(), ddsMessage.getInstant());
            ddsMessage.setLogLevel(MessageTools.toMessage(localMessage.level()));
            MessageTools.packLongStringToByteSequence(localMessage.message(), ddsMessage.getLogMessage());
         }
      }
   }

   public void fromMessage(us.ihmc.idl.IDLSequence.Object<BehaviorTreeLogMessage> recentLogMessages)
   {
      if (!isRobotSide()) // Ignore on robot side
      {
         for (int i = recentLogMessages.size() - 1; i >= 0; i--)
         {
            BehaviorTreeLogMessage logMessage = recentLogMessages.get(i);
            Instant messageInstant = MessageTools.toInstant(logMessage.getInstant());
            if (messageInstant.isAfter(lastPrintedTimestamp))
            {
               Level level = MessageTools.fromMessage(logMessage.getLogLevel());
               String message = MessageTools.unpackLongStringFromByteSequence(logMessage.getLogMessage());
//               LogTools.log(level, 2, message);
               recentMessages.add(new LogMessage(messageInstant, level, message));

               lastPrintedTimestamp = messageInstant;
            }
         }
      }
   }

   protected boolean isRobotSide()
   {
      return !isModificationDisallowed();
   }

   public LinkedList<LogMessage> getRecentMessages()
   {
      return recentMessages;
   }

   public void log(Level level, Object message) { logIfEnabled(level, message); }
   public void log(Level level, int additionalStackTraceHeight, Object message) { logIfEnabled(level, additionalStackTraceHeight, message); }
   public void log(Level level, Supplier<?> msgSupplier) { logIfEnabled(level, msgSupplier); }
   public void log(Level level, Object message, Supplier<?> msgSupplier) { logIfEnabled(level, message, msgSupplier); }
   public void log(Level level, Object message, Object p0) { logIfEnabled(level, message, p0); }
   public void log(Level level, Object message, Object p0, Object p1) { logIfEnabled(level, message, p0, p1); }
   public void log(Level level, Object message, Object p0, Object p1, Object p2) { logIfEnabled(level, message, p0, p1, p2); }
   public void fatal(Object message) { logIfEnabled(Level.FATAL, message); }
   public void fatal(int additionalStackTraceHeight, Object message) { logIfEnabled(Level.FATAL, additionalStackTraceHeight, message); }
   public void fatal(Supplier<?> msgSupplier) { logIfEnabled(Level.FATAL, msgSupplier); }
   public void fatal(Object message, Supplier<?> msgSupplier) { logIfEnabled(Level.FATAL, msgSupplier); }
   public void fatal(Object message, Object p0) { logIfEnabled(Level.FATAL, message, p0); }
   public void fatal(Object message, Object p0, Object p1) { logIfEnabled(Level.FATAL, message, p0, p1); }
   public void fatal(Object message, Object p0, Object p1, Object p2) { logIfEnabled(Level.FATAL, message, p0, p1, p2); }
   public void error(Object message) { logIfEnabled(Level.ERROR, message); }
   public void error(int additionalStackTraceHeight, Object message) { logIfEnabled(Level.ERROR, additionalStackTraceHeight, message); }
   public void error(Supplier<?> msgSupplier) { logIfEnabled(Level.ERROR, msgSupplier); }
   public void error(Object message, Supplier<?> msgSupplier) { logIfEnabled(Level.ERROR, msgSupplier); }
   public void error(Object message, Object p0) { logIfEnabled(Level.ERROR, message, p0); }
   public void error(Object message, Object p0, Object p1) { logIfEnabled(Level.ERROR, message, p0, p1); }
   public void error(Object message, Object p0, Object p1, Object p2) { logIfEnabled(Level.ERROR, message, p0, p1, p2); }
   public void warn(Object message) { logIfEnabled(Level.WARN, message); }
   public void warn(int additionalStackTraceHeight, Object message) { logIfEnabled(Level.WARN, additionalStackTraceHeight, message); }
   public void warn(Supplier<?> msgSupplier) { logIfEnabled(Level.WARN, msgSupplier); }
   public void warn(Object message, Supplier<?> msgSupplier) { logIfEnabled(Level.WARN, msgSupplier); }
   public void warn(Object message, Object p0) { logIfEnabled(Level.WARN, message, p0); }
   public void warn(Object message, Object p0, Object p1) { logIfEnabled(Level.WARN, message, p0, p1); }
   public void warn(Object message, Object p0, Object p1, Object p2) { logIfEnabled(Level.WARN, message, p0, p1, p2); }
   public void info(Object message) { logIfEnabled(Level.INFO, message); }
   public void info(int additionalStackTraceHeight, Object message) { logIfEnabled(Level.INFO, additionalStackTraceHeight, message); }
   public void info(Supplier<?> msgSupplier) { logIfEnabled(Level.INFO, msgSupplier); }
   public void info(Object message, Supplier<?> msgSupplier) { logIfEnabled(Level.INFO, msgSupplier); }
   public void info(Object message, Object p0) { logIfEnabled(Level.INFO, message, p0); }
   public void info(Object message, Object p0, Object p1) { logIfEnabled(Level.INFO, message, p0, p1); }
   public void info(Object message, Object p0, Object p1, Object p2) { logIfEnabled(Level.INFO, message, p0, p1, p2); }
   public void debug(Object message) { logIfEnabled(Level.DEBUG, message); }
   public void debug(int additionalStackTraceHeight, Object message) { logIfEnabled(Level.DEBUG, additionalStackTraceHeight, message); }
   public void debug(Supplier<?> msgSupplier) { logIfEnabled(Level.DEBUG, msgSupplier); }
   public void debug(Object message, Supplier<?> msgSupplier) { logIfEnabled(Level.DEBUG, msgSupplier); }
   public void debug(Object message, Object p0) { logIfEnabled(Level.DEBUG, message, p0); }
   public void debug(Object message, Object p0, Object p1) { logIfEnabled(Level.DEBUG, message, p0, p1); }
   public void debug(Object message, Object p0, Object p1, Object p2) { logIfEnabled(Level.DEBUG, message, p0, p1, p2); }
   public void trace(Object message) { logIfEnabled(Level.TRACE, message); }
   public void trace(int additionalStackTraceHeight, Object message) { logIfEnabled(Level.TRACE, additionalStackTraceHeight, message); }
   public void trace(Supplier<?> msgSupplier) { logIfEnabled(Level.TRACE, msgSupplier); }
   public void trace(Object message, Supplier<?> msgSupplier) { logIfEnabled(Level.TRACE, msgSupplier); }
   public void trace(Object message, Object p0) { logIfEnabled(Level.TRACE, message, p0); }
   public void trace(Object message, Object p0, Object p1) { logIfEnabled(Level.TRACE, message, p0, p1); }
   public void trace(Object message, Object p0, Object p1, Object p2) { logIfEnabled(Level.TRACE, message, p0, p1, p2); }

   private void logIfEnabled(Level level, Object message)
   {
      if (LogTools.isEnabled(level, 2))
      {
         LogTools.log(level, 2, message);
         queueLogMessage(level, message.toString());
      }
   }
   private void logIfEnabled(Level level, int additionalStackTraceHeight, Object message)
   {
      if (LogTools.isEnabled(level, 2))
      {
         LogTools.log(level, 2 + additionalStackTraceHeight, message);
         queueLogMessage(level, message.toString());
      }
   }
   private void logIfEnabled(Level level, Supplier<?> msgSupplier)
   {
      if (LogTools.isEnabled(level, 2))
      {
         String suppliedMessage = msgSupplier.get().toString();
         LogTools.log(level, 2, suppliedMessage);
         queueLogMessage(level, suppliedMessage);
      }
   }
   private void logIfEnabled(Level level, Object message, Supplier<?> msgSupplier)
   {
      if (LogTools.isEnabled(level, 2))
      {
         String parameterProcessed = new ParameterizedMessage(message.toString(), msgSupplier.get()).getFormattedMessage();
         LogTools.log(level, 2, parameterProcessed);
         queueLogMessage(level, parameterProcessed);
      }
   }
   private void logIfEnabled(Level level, Object message, Object p0)
   {
      if (LogTools.isEnabled(level, 2))
      {
         String parameterProcessed = new ParameterizedMessage(message.toString(), p0).getFormattedMessage();
         LogTools.log(level, 2, parameterProcessed);
         queueLogMessage(level, parameterProcessed);
      }
   }
   private void logIfEnabled(Level level, Object message, Object p0, Object p1)
   {
      if (LogTools.isEnabled(level, 2))
      {
         String parameterProcessed = new ParameterizedMessage(message.toString(), p0, p1).getFormattedMessage();
         LogTools.log(level, 2, parameterProcessed);
         queueLogMessage(level, parameterProcessed);
      }
   }
   private void logIfEnabled(Level level, Object message, Object p0, Object p1, Object p2)
   {
      if (LogTools.isEnabled(level, 2))
      {
         String parameterProcessed = new ParameterizedMessage(message.toString(), p0, p1, p2).getFormattedMessage();
         LogTools.log(level, 2, parameterProcessed);
         queueLogMessage(level, parameterProcessed);
      }
   }
}
