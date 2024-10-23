package us.ihmc.communication;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Properties;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

public class NetworkParametersTest
{
   protected static final Random RANDOM = new Random(0L);

   private static final Path TEST_DIR_PATH = IHMCCommonPaths.DOT_IHMC_DIRECTORY.resolve("network-parameters-test");
   private static final Path TEST_FILE_PATH = TEST_DIR_PATH.resolve("TestIHMCNetworkParameters.ini");
   private static final File TEST_DIRECTORY = TEST_DIR_PATH.toFile();
   private static final File TEST_FILE = TEST_FILE_PATH.toFile();

   private static String PARAMETER_FILE_PATH;

   @BeforeAll
   public static void createTestingDirectory()
   {
      PARAMETER_FILE_PATH = System.getProperty("us.ihmc.networkParameterFile");

      if (TEST_DIRECTORY.exists())
         assumeTrue(TEST_DIRECTORY.delete(), "Could not delete an existing test directory" + TEST_DIR_PATH);
      assumeTrue(TEST_DIRECTORY.mkdirs(), "Could not make a test directory: " + TEST_DIR_PATH);
      TEST_DIRECTORY.deleteOnExit();

      System.setProperty("us.ihmc.networkParameterFile", TEST_FILE_PATH.toString());
   }

   @AfterAll
   public static void resetParameterFilePath()
   {
      if (PARAMETER_FILE_PATH != null)
         System.setProperty("us.ihmc.networkParameterFile", PARAMETER_FILE_PATH);
   }

   @Test
   public void testNoFile()
   {
      // Delete any existing test file
      deleteTestFile();

      // Reload existing network parameters
      NetworkParameters.reload();

      for (NetworkParameterKeys key : NetworkParameterKeys.values())
      {
         assertFalse(NetworkParameters.hasKey(key));
         assertThrows(Exception.class, () -> NetworkParameters.getHost(key));
      }

      assertEquals(1, NetworkParameters.getRTPSDomainID());
   }

   @Test
   public void testEmptyFile()
   {
      // Delete any existing test file
      deleteTestFile();

      // Create a new (empty) test file
      createEmptyTestFile();

      // Reload existing network parameters
      NetworkParameters.reload();

      for (NetworkParameterKeys key : NetworkParameterKeys.values())
      {
         assertFalse(NetworkParameters.hasKey(key));
         assertThrows(Exception.class, () -> NetworkParameters.getHost(key));
      }

      assertEquals(1, NetworkParameters.getRTPSDomainID());
   }

   @Test
   public void testLoadingRTPSDomainID()
   {
      testLoadingRTPSDomainID(RANDOM.nextInt(230));
   }

   protected void testLoadingRTPSDomainID(int testDomainID)
   {
      // Create parameters file with random domain id
      createTestNetworkParametersFile(testDomainID);

      // Reload the file
      NetworkParameters.reload();

      assertEquals(testDomainID, NetworkParameters.getRTPSDomainID());
   }

   @Test
   public void testLoadingRTPSDomainRestrictions()
   {
      testLoadingRTPSDomainRestrictions("127.0.0.1/8", "192.0.2.0/24", "198.51.100.0/24", "0.0.0.0/24");
   }

   protected void testLoadingRTPSDomainRestrictions(String... testAddressRestrictions)
   {
      // Create parameters file with rtps subnet restrictions
      createTestNetworkParametersFile(230, testAddressRestrictions);

      // Reload the parameters
      NetworkParameters.reload();

      // Ensure parameters contain same restrictions
      String combinedRestrictions = combineSubnetRestrictions(testAddressRestrictions);
      assertEquals(combinedRestrictions, NetworkParameters.getHost(NetworkParameterKeys.RTPSSubnet));
   }

   protected void deleteTestFile()
   {
      if (TEST_FILE.exists())
         assumeTrue(TEST_FILE.delete(), "Could not delete the test file: " + TEST_FILE_PATH);
   }

   protected void createEmptyTestFile()
   {
      boolean success = true;
      try
      {
         assumeTrue(TEST_FILE.createNewFile(), "Could not create a test file: " + TEST_FILE_PATH);
         TEST_FILE.deleteOnExit();
      }
      catch (IOException exception)
      {
         LogTools.error(exception);
         success = false;
      }

      assumeTrue(success, "Exception thrown while creating a test file: " + TEST_FILE_PATH);
   }

   protected String combineSubnetRestrictions(String... subnetRestrictions)
   {
      StringBuilder combinedString = new StringBuilder(subnetRestrictions[0]);
      for (int i = 1; i < subnetRestrictions.length; ++i)
         combinedString.append(", ").append(subnetRestrictions[i]);
      return combinedString.toString();
   }

   protected void createTestNetworkParametersFile(int rtpsDomainID, String... rtpsSubnetRestrictions)
   {
      boolean success = true;

      deleteTestFile();

      try
      {
         createEmptyTestFile();
         FileOutputStream outputStream = new FileOutputStream(TEST_FILE);

         Properties networkProperties = new Properties();
         networkProperties.setProperty(NetworkParameterKeys.RTPSDomainID.toString(), String.valueOf(rtpsDomainID));

         if (rtpsSubnetRestrictions != null && rtpsSubnetRestrictions.length > 0)
            networkProperties.setProperty(NetworkParameterKeys.RTPSSubnet.toString(), combineSubnetRestrictions(rtpsSubnetRestrictions));

         networkProperties.store(outputStream, "Test properties!");
         outputStream.close();
      }
      catch (IOException exception)
      {
         success = false;
      }

      assumeTrue(success, "Failed to write to test file: " + TEST_FILE_PATH);
   }
}
