package us.ihmc.commons.nio;

import java.nio.file.OpenOption;
import java.nio.file.StandardOpenOption;

public enum WriteOption
{
   APPEND(StandardOpenOption.CREATE, StandardOpenOption.APPEND), TRUNCATE(StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING);

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
