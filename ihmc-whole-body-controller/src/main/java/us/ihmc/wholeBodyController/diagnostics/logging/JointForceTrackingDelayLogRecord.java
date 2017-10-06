package us.ihmc.wholeBodyController.diagnostics.logging;

import java.util.logging.Level;
import java.util.logging.LogRecord;

public class JointForceTrackingDelayLogRecord extends LogRecord
{
   private static final long serialVersionUID = -2944848570321819221L;

   public JointForceTrackingDelayLogRecord(Level level, String msg)
   {
      super(level, msg);
   }
}
