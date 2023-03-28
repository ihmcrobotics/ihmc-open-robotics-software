package us.ihmc.rdx.perception;

import com.github.swrirobotics.bags.reader.BagFile;
import com.github.swrirobotics.bags.reader.BagReader;
import com.github.swrirobotics.bags.reader.exceptions.BagReaderException;

import java.io.File;

public class ROS1BagBasedTestHelper
{
   public void read()
   {
      try
      {
         BagFile rosBag = BagReader.readFile(new File("meep"));
      }
      catch (BagReaderException e)
      {
         e.printStackTrace();
      }
   }
}
