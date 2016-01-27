package us.ihmc.wholeBodyController.diagnostics.logging;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.logging.Formatter;
import java.util.logging.Level;
import java.util.logging.LogRecord;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class DiagnosticLoggerFormatter extends Formatter
{
   private final DoubleYoVariable yoTime;
   private final NumberFormat doubleFormat = new DecimalFormat("0.000");
   private final String separator = " - ";

   private final int maxLevelNameLength;

   public DiagnosticLoggerFormatter(DoubleYoVariable yoTime)
   {
      this.yoTime = yoTime;
      Level[] allLevels = new Level[]{Level.SEVERE, Level.WARNING, Level.INFO, Level.CONFIG, Level.FINE, Level.FINER, Level.FINEST};
      int maxLength = 0;
      for (Level level : allLevels)
         maxLength = Math.max(maxLength, level.getName().length());
      maxLevelNameLength = maxLength;
   }

   @Override
   public String format(LogRecord record)
   {
      String levelName = record.getLevel().getName();
      int nblankSpaces = maxLevelNameLength - levelName.length() + 1;
      String blanks = String.format("%1$" + nblankSpaces + "s", "");
      String level = "[" + levelName + blanks + "]";
      String time = "[robotTime = " + doubleFormat.format(yoTime.getDoubleValue()) + "]";
      String message = formatMessage(record);

      return level + separator + time + separator + message + "\n";
   }
}
