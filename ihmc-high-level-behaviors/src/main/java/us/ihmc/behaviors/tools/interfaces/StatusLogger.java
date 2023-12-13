package us.ihmc.behaviors.tools.interfaces;

import behavior_msgs.msg.dds.StatusLogMessage;
import org.apache.logging.log4j.Level;
import org.apache.logging.log4j.message.ParameterizedMessage;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.log.LogToolsWriteOnly;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.function.Supplier;

/**
 * Useful so when we have modules delivering messages, we can also send those messages
 * out via Messager so a feed can be displayed to the operator.
 */
public class StatusLogger implements LogToolsWriteOnly
{
   private final DateTimeFormatter dateFormat = DateTimeFormatter.ofPattern("ss:SSS");

   private final UIPublisher uiPublisher;

   public StatusLogger(UIPublisher uiPublisher)
   {
      this.uiPublisher = uiPublisher;
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
         publishToUI(level, message.toString());
      }
   }
   private void logIfEnabled(Level level, int additionalStackTraceHeight, Object message)
   {
      if (LogTools.isEnabled(level, 2))
      {
         LogTools.log(level, 2 + additionalStackTraceHeight, message);
         publishToUI(level, message.toString());
      }
   }
   private void logIfEnabled(Level level, Supplier<?> msgSupplier)
   {
      if (LogTools.isEnabled(level, 2))
      {
         String suppliedMessage = msgSupplier.get().toString();
         LogTools.log(level, 2, suppliedMessage);
         publishToUI(level, suppliedMessage);
      }
   }
   private void logIfEnabled(Level level, Object message, Supplier<?> msgSupplier)
   {
      if (LogTools.isEnabled(level, 2))
      {
         String parameterProcessed = new ParameterizedMessage(message.toString(), msgSupplier.get()).getFormattedMessage();
         LogTools.log(level, 2, parameterProcessed);
         publishToUI(level, parameterProcessed);
      }
   }
   private void logIfEnabled(Level level, Object message, Object p0)
   {
      if (LogTools.isEnabled(level, 2))
      {
         String parameterProcessed = new ParameterizedMessage(message.toString(), p0).getFormattedMessage();
         LogTools.log(level, 2, parameterProcessed);
         publishToUI(level, parameterProcessed);
      }
   }
   private void logIfEnabled(Level level, Object message, Object p0, Object p1)
   {
      if (LogTools.isEnabled(level, 2))
      {
         String parameterProcessed = new ParameterizedMessage(message.toString(), p0, p1).getFormattedMessage();
         LogTools.log(level, 2, parameterProcessed);
         publishToUI(level, parameterProcessed);
      }
   }
   private void logIfEnabled(Level level, Object message, Object p0, Object p1, Object p2)
   {
      if (LogTools.isEnabled(level, 2))
      {
         String parameterProcessed = new ParameterizedMessage(message.toString(), p0, p1, p2).getFormattedMessage();
         LogTools.log(level, 2, parameterProcessed);
         publishToUI(level, parameterProcessed);
      }
   }

   private void publishToUI(Level level, String  message)
   {
      StatusLogMessage statusLogMessage = new StatusLogMessage();
      statusLogMessage.setLogLevel(level.intLevel());
      MessageTools.packLongStringToByteSequence(LocalDateTime.now().format(dateFormat) + " " + message, statusLogMessage.getLogMessage());
      uiPublisher.publishToUI(BehaviorModule.API.STATUS_LOG, statusLogMessage);
   }
}
