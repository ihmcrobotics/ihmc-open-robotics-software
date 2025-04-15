package us.ihmc.avatar.logProcessor.leRobot;

import com.jerolba.carpet.CarpetWriter;
import com.jerolba.carpet.ColumnNamingStrategy;
import com.jerolba.carpet.annotation.Alias;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class LeRobotDatasetDataWriter
{
   record Observation(List<Float> state, // position of arm joints
                      List<Float> effort // not used
   ) { }
//   record Next(@Alias("next.done") boolean done // indicates the end of en episode; true for the last frame in each episode
//   ) { }
   record EpisodeRecord(Observation observation,
                        List<Float> action, // goal position of arm joints
                        long episodeIndex, // index of the episode for this sample
                        long frameIndex, // index of the frame for this sample in the episode; starts at 0 for each episode
                        float timestamp, // in the episode
                        @Alias("next.done") boolean nextDone,
                        long index, // general index in the whole dataset
                        long taskIndex
   ) { }

   private final OutputStream outputStream;
   private final CarpetWriter<EpisodeRecord> carpetWriter;
   private Random random = new Random();

   public LeRobotDatasetDataWriter(Path parquetPath)
   {
      outputStream = ExceptionTools.handle(() -> Files.newOutputStream(parquetPath), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      carpetWriter = ExceptionTools.handle(() ->
         new CarpetWriter.Builder<>(outputStream, EpisodeRecord.class).withColumnNamingStrategy(ColumnNamingStrategy.SNAKE_CASE).build()
      , DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public void write()
   {
      List<Float> observationState = new ArrayList<>();
      List<Float> observationEffort = new ArrayList<>();
      List<Float> action = new ArrayList<>();
      for (int i = 0; i < 14; i++)
      {
         observationState.add(random.nextFloat());
         observationEffort.add(random.nextFloat());
         action.add(random.nextFloat());
      }

      Observation observation = new Observation(observationState, observationEffort);
//      Next next = new Next(random.nextBoolean());
      EpisodeRecord value = new EpisodeRecord(observation,
                                              action,
                                              random.nextLong(),
                                              random.nextLong(),
                                              random.nextFloat(),
                                              random.nextBoolean(),
                                              random.nextLong(),
                                              random.nextLong());

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
