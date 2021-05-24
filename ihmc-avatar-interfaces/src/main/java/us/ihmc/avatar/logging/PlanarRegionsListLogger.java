package us.ihmc.avatar.logging;

import us.ihmc.log.LogTools;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

public class PlanarRegionsListLogger
{
   private final String logName;
   private final int maxTicksToRecord;

   private boolean started = false;

   private BufferedWriter out = null;

   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;

   public PlanarRegionsListLogger(String logName, int maxTicksToRecord) {
      this.logName = logName;
      this.maxTicksToRecord = maxTicksToRecord;
   }

   public void start() {
      File file = new File(logDirectory + dateFormat.format(new Date()) + "_" + this.getClass().getSimpleName() + ".prllog");
      try {
         out = new BufferedWriter(new FileWriter(file));
      } catch (IOException ex) {
         LogTools.error("Could not start PlanarRegionsListLogger!");
         LogTools.error(ex.getMessage());
         for (StackTraceElement e : ex.getStackTrace())
            LogTools.error(e.toString());
      }

      try {
         out.write("");
      } catch (IOException ex) {
         LogTools.warn("Could not write to BufferedWriter - " + ex.getMessage());
      }

      started = true;
   }

   public void update(long time) {
      if (!started)
         return;
   }

   @Override
   public void finalize() {
      try {
         if (out != null)
            out.close();
      }
      catch (IOException ignored) {}
   }
}
