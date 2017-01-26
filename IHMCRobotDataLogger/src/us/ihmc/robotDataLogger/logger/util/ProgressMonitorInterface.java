package us.ihmc.robotDataLogger.logger.util;

import java.io.PrintStream;

public interface ProgressMonitorInterface
{

   void setNote(String note);

   void setProgress(int i);

   PrintStream getPrintStream();

   void close();

   void setError(String string);

   void initialize(String message, String note, int min, int max);

}