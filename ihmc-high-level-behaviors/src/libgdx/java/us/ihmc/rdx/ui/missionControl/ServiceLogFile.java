package us.ihmc.rdx.ui.missionControl;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.List;

public class ServiceLogFile extends File
{
   public ServiceLogFile(String pathname)
   {
      super(pathname);
   }

   public void saveLogLine(String line) throws IOException
   {
      Files.writeString(Paths.get(getAbsolutePath()), line + System.lineSeparator(), StandardOpenOption.APPEND);
   }

   public List<String> loadLogLines() throws IOException
   {
      return Files.readAllLines(Paths.get(getAbsolutePath()));
   }
}
