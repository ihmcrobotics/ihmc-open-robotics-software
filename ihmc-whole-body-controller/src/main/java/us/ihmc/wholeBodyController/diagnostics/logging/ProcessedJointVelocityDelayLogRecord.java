package us.ihmc.wholeBodyController.diagnostics.logging;

import java.util.logging.Level;
import java.util.logging.LogRecord;

public class ProcessedJointVelocityDelayLogRecord extends LogRecord
{
   private static final long serialVersionUID = -6381601941891881835L;

   public ProcessedJointVelocityDelayLogRecord(Level level, String msg)
   {
      super(level, msg);
   }
}
