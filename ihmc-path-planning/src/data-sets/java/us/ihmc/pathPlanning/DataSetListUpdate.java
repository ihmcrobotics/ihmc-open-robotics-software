package us.ihmc.pathPlanning;

import org.apache.commons.io.IOUtils;

import java.io.IOException;
import java.io.InputStream;
import java.util.List;

import static java.nio.charset.StandardCharsets.UTF_8;

/**
 * When adding/removing datasets, run this and copy output to DataSetList.txt
 */
public class DataSetListUpdate
{
   private DataSetListUpdate() throws IOException
   {
      InputStream dataSetStream = getClass().getResourceAsStream("/" + DataSetIOTools.DATA_SET_DIRECTORY_PATH);
      List<String> dataSetNames = IOUtils.readLines(dataSetStream, UTF_8);

      for (int i = 0; i < dataSetNames.size(); i++)
      {
         System.out.println(dataSetNames.get(i));
      }
   }

   public static void main(String[] args) throws IOException
   {
      new DataSetListUpdate();
   }
}
