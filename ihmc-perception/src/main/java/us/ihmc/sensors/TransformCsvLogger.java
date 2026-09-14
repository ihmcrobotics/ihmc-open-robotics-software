package us.ihmc.sensors;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;

/**
 * Minimal CSV logger for dumping {@link RigidBodyTransformReadOnly}s to disk, for offline plotting
 * (e.g. in Python/pandas) when debugging things like frame-sync smoothness that are too fine-grained
 * to eyeball from log prints.
 * <p>
 * Every row is tagged with a caller-provided {@code label}, so multiple sources (e.g. an interpolated
 * pose vs. the live pose it was compared against) can be logged to the same file and separated later
 * by filtering on that column. Thread-safe: {@link #log} may be called concurrently from multiple threads.
 */
public class TransformCsvLogger implements AutoCloseable
{
   private final BufferedWriter writer;
   private final int extraColumnCount;

   /**
    * @param outputFile    File to write to. Overwritten if it already exists.
    * @param extraColumns  Names of any additional columns that will be passed as {@code extraValues} to {@link #log}.
    */
   public TransformCsvLogger(Path outputFile, String... extraColumns)
   {
      extraColumnCount = extraColumns.length;

      try
      {
         Files.createDirectories(outputFile.toAbsolutePath().getParent());
         writer = Files.newBufferedWriter(outputFile, StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING);

         StringBuilder header = new StringBuilder("label,timestampNanos,x,y,z,qx,qy,qz,qw");
         for (String extraColumn : extraColumns)
            header.append(',').append(extraColumn);

         writer.write(header.toString());
         writer.newLine();
         writer.flush();
      }
      catch (IOException e)
      {
         throw new UncheckedIOException(e);
      }
   }

   /**
    * Appends one row to the CSV.
    *
    * @param label          Arbitrary tag identifying the source of this row (e.g. {@code "sample"}, {@code "grab_interpolated"}).
    * @param timestampNanos Nanosecond timestamp associated with the transform, e.g. from {@code Instant} epoch nanos.
    * @param transform      Transform to log. Rotation is logged as a quaternion (converted from whatever representation it's stored in).
    * @param extraValues    Values for the extra columns declared in the constructor, same order. Count must match.
    */
   public void log(String label, long timestampNanos, RigidBodyTransformReadOnly transform, Object... extraValues)
   {
      if (extraValues.length != extraColumnCount)
         throw new IllegalArgumentException("Expected " + extraColumnCount + " extra values, got " + extraValues.length);

      Quaternion quaternion = new Quaternion(transform.getRotation());

      StringBuilder row = new StringBuilder();
      row.append(label).append(',')
         .append(timestampNanos).append(',')
         .append(transform.getTranslationX()).append(',')
         .append(transform.getTranslationY()).append(',')
         .append(transform.getTranslationZ()).append(',')
         .append(quaternion.getX()).append(',')
         .append(quaternion.getY()).append(',')
         .append(quaternion.getZ()).append(',')
         .append(quaternion.getS());

      for (Object extraValue : extraValues)
         row.append(',').append(extraValue);

      try
      {
         synchronized (writer)
         {
            writer.write(row.toString());
            writer.newLine();
            writer.flush();
         }
      }
      catch (IOException e)
      {
         throw new UncheckedIOException(e);
      }
   }

   @Override
   public void close()
   {
      try
      {
         writer.close();
      }
      catch (IOException e)
      {
         throw new UncheckedIOException(e);
      }
   }
}
