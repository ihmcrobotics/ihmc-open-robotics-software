package us.ihmc.wholeBodyController.diagnostics.logging;

import java.util.logging.Level;
import java.util.logging.LogRecord;

public class ProcessedJointPositionDelayLogRecord extends LogRecord
{
   private static final long serialVersionUID = 2216279348109121221L;

   public ProcessedJointPositionDelayLogRecord(Level level, String msg)
   {
      super(level, msg);
   }
}
