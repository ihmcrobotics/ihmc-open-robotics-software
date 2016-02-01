package us.ihmc.wholeBodyController.diagnostics.logging;

import java.util.logging.Filter;
import java.util.logging.Formatter;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.StreamHandler;

public class DiagnosticLoggerSystemErrHandler extends StreamHandler
{
   public DiagnosticLoggerSystemErrHandler(Formatter formatter, Level level)
   {
      setLevel(level);
      setFormatter(formatter);
      setFilter(new Filter()
      {
         @Override
         public boolean isLoggable(LogRecord record)
         {
            return record.getLevel().intValue() > Level.INFO.intValue();
         }
      });
      setOutputStream(System.err);
   }

   /**
    * Publish a <tt>LogRecord</tt>.
    * <p>
    * The logging request was made initially to a <tt>Logger</tt> object,
    * which initialized the <tt>LogRecord</tt> and forwarded it here.
    * <p>
    * @param  record  description of the log event. A null record is
    *                 silently ignored and is not published
    */
   @Override
   public void publish(LogRecord record)
   {
      super.publish(record);
      flush();
   }

   /**
    * Override <tt>StreamHandler.close</tt> to do a flush but not
    * to close the output stream.  That is, we do <b>not</b>
    * close <tt>System.out</tt>.
    */
   @Override
   public void close()
   {
      flush();
   }
}
