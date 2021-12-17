package us.ihmc.gdx.perception;

import com.github.swrirobotics.bags.reader.BagFile;
import com.github.swrirobotics.bags.reader.BagReader;
import com.github.swrirobotics.bags.reader.exceptions.BagReaderException;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;

import java.io.File;
import java.nio.file.Paths;

public class GDXGPUPlanarRegionExtractionDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");


   public GDXGPUPlanarRegionExtractionDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            try
            {
               BagFile rosBag = BagReader.readFile(new File("meep"));
            }
            catch (BagReaderException e)
            {
               e.printStackTrace();
            }
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXGPUPlanarRegionExtractionDemo();
   }
}
