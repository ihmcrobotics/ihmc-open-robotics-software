package us.ihmc.avatar.logProcessor.leRobot;

import com.jerolba.carpet.CarpetWriter;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Random;

public class LeRobotDatasetDataWriter
{
   record XYZ(double x, double y, double z) { }

   private final OutputStream outputStream;
   private final CarpetWriter<XYZ> carpetWriter;
   private Random random = new Random();

   public LeRobotDatasetDataWriter(Path parquetPath)
   {
      outputStream = ExceptionTools.handle(() -> Files.newOutputStream(parquetPath), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      carpetWriter = ExceptionTools.handle(() -> new CarpetWriter<>(outputStream, XYZ.class), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public void write()
   {
      XYZ value = new XYZ(random.nextDouble(), random.nextDouble(), random.nextDouble());
      ExceptionTools.handle(() -> carpetWriter.write(value), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public void close()
   {
      if (carpetWriter != null)
         ExceptionTools.handle(carpetWriter::close, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      if (outputStream != null)
         ExceptionTools.handle(outputStream::close, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }
}
