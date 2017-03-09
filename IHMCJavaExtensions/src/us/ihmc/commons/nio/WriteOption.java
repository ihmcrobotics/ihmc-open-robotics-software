package us.ihmc.commons.nio;

import java.nio.file.OpenOption;
import java.nio.file.StandardOpenOption;

/**
 * Write options for files. Subset of {@link StandardOpenOption} for convenience.
 */
public enum WriteOption
{
   /** Append to the end of the file. */
   APPEND(StandardOpenOption.CREATE, StandardOpenOption.APPEND),
   
   /** Start overwriting at the beginning of the file. */
   TRUNCATE(StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING);

   private final StandardOpenOption[] openOptions;

   private WriteOption(StandardOpenOption... openOptions)
   {
      this.openOptions = openOptions;
   }

   /* package-private */ OpenOption[] getOptions()
   {
      return openOptions;
   }
}
