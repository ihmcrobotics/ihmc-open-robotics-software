package us.ihmc.robotics;

import org.junit.jupiter.api.Test;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class PlanarRegionFileToolsTest
{

   private static final Random rand = new Random();
   private static Path tempDirectory = null;

   private static Path getTestDirectory()
   {
      Path dir = null;

      try
      {
         if (tempDirectory == null)
            tempDirectory = Files.createTempDirectory(PlanarRegionFileToolsTest.class.getSimpleName() + "_");

         String callingMethod = new Throwable().getStackTrace()[1].getMethodName(); //Gets name of the test currently being run

         dir = tempDirectory.resolve(callingMethod);

         if (!Files.exists(dir))
            Files.createDirectory(dir);

         System.out.println("Test taking place in directory " + dir.toAbsolutePath());
      }
      catch (IOException ex)
      {
         ex.printStackTrace();
         fail("The test could not be performed, as getTestDirectory() threw an IOException.");
      }

      return dir;
   }

   @Test
   public void exportAsFolderAndImportTest()
   {
      Path path = getTestDirectory();
      PlanarRegionsList list = PlanarRegionsList.generatePlanarRegionsListFromRandomPolygonsWithRandomTransform(rand, 5, 5, 5, 5);

      PlanarRegionFileTools.exportPlanarRegionData(path, list);
      PlanarRegionsList test = PlanarRegionFileTools.importPlanarRegionData(path.toFile());

      assertEquals(list, test);
   }

   @Test
   public void exportAsFileAndImportTest()
   {
      Path path = getTestDirectory().resolve("regions.prl");
      PlanarRegionsList list = PlanarRegionsList.generatePlanarRegionsListFromRandomPolygonsWithRandomTransform(rand, 5, 5, 5, 5);

      PlanarRegionFileTools.exportPlanarRegionDataAsFile(path, list);
      PlanarRegionsList test = PlanarRegionFileTools.importPlanarRegionData(path.toFile());

      assertEquals(list, test);
   }

   @Test
   public void exportToStreamAndImportTest()
   {
      Path path = getTestDirectory().resolve("regions.prl");
      PlanarRegionsList list = PlanarRegionsList.generatePlanarRegionsListFromRandomPolygonsWithRandomTransform(rand, 5, 5, 5, 5);

      try
      {
         FileOutputStream ostream = new FileOutputStream(path.toFile());
         PlanarRegionFileTools.exportPlanarRegionDataToStream(ostream, list);
         ostream.close();
      }
      catch (IOException ex)
      {
         fail("IOException thrown - " + ex.getMessage());
      }

      PlanarRegionsList test = PlanarRegionFileTools.importPlanarRegionData(path.toFile());

      assertEquals(list, test);
   }

   @Test
   public void isPlanarRegionDirectoryTest()
   {
      Path path = getTestDirectory();
      PlanarRegionsList list = PlanarRegionsList.generatePlanarRegionsListFromRandomPolygonsWithRandomTransform(rand, 1, 5, 5, 5);

      assertFalse(PlanarRegionFileTools.isPlanarRegionFile(path.toFile()));

      PlanarRegionFileTools.exportPlanarRegionData(path, list);

      assertTrue(PlanarRegionFileTools.isPlanarRegionFile(path.toFile()));
   }

   @Test
   public void isPlanarRegionFileTest()
   {
      Path path = getTestDirectory().resolve("regions.prl");
      PlanarRegionsList list = PlanarRegionsList.generatePlanarRegionsListFromRandomPolygonsWithRandomTransform(rand, 1, 5, 5, 5);

      assertFalse(PlanarRegionFileTools.isPlanarRegionFile(path.toFile()));

      PlanarRegionFileTools.exportPlanarRegionDataAsFile(path, list);

      assertTrue(PlanarRegionFileTools.isPlanarRegionFile(path.toFile()));
   }
}
