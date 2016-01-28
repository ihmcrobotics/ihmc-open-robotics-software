package us.ihmc.wholeBodyController.diagnostics.logging;

import java.util.logging.Level;
import java.util.logging.LogRecord;

public class RawJointVelocityDelayLogRecord extends LogRecord
{
   private static final long serialVersionUID = -9148473420233935745L;

   public RawJointVelocityDelayLogRecord(Level level, String msg)
   {
      super(level, msg);
   }
}
