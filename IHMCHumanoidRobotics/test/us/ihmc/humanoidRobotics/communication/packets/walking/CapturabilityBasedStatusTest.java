package us.ihmc.humanoidRobotics.communication.packets.walking;

import static org.junit.Assert.assertTrue;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.net.KryoStreamDeSerializer;
import us.ihmc.communication.net.KryoStreamSerializer;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.io.files.FileTools;
import us.ihmc.tools.testing.JUnitTools;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class CapturabilityBasedStatusTest
{
   private static final Path TEST_ROOT_PATH = JUnitTools.deriveTestResourcesPath(CapturabilityBasedStatusTest.class);

   @Before
   public void setUp()
   {
      FileTools.ensureDirectoryExists(TEST_ROOT_PATH);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testSerializeAndDeserialize() throws IOException
   {
      KryoStreamSerializer kryoStreamSerializer = new KryoStreamSerializer(UnitConversions.megabytesToBytes(10));
      kryoStreamSerializer.registerClasses(new IHMCCommunicationKryoNetClassList());

      KryoStreamDeSerializer kryoStreamDeSerializer = new KryoStreamDeSerializer(UnitConversions.megabytesToBytes(10));
      kryoStreamDeSerializer.registerClasses(new IHMCCommunicationKryoNetClassList());

      CapturabilityBasedStatus cbs = new CapturabilityBasedStatus(new Random());

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      kryoStreamSerializer.write(outputStream, cbs);

      ByteArrayInputStream byteArrayInputStream = new ByteArrayInputStream(outputStream.toByteArray());
      CapturabilityBasedStatus cbsOut = (CapturabilityBasedStatus) kryoStreamDeSerializer.read(byteArrayInputStream);

      assertPacketsEqual(cbs, cbsOut);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSerializeToFileAndDeserialize() throws IOException
   {
      Random random = new Random();
      KryoStreamSerializer kryoStreamSerializer = new KryoStreamSerializer(UnitConversions.megabytesToBytes(10));
      kryoStreamSerializer.registerClasses(new IHMCCommunicationKryoNetClassList());

      KryoStreamDeSerializer kryoStreamDeSerializer = new KryoStreamDeSerializer(UnitConversions.megabytesToBytes(10));
      kryoStreamDeSerializer.registerClasses(new IHMCCommunicationKryoNetClassList());

      CapturabilityBasedStatus cbs1 = new CapturabilityBasedStatus(random);
      CapturabilityBasedStatus cbs2 = new CapturabilityBasedStatus(random);
      CapturabilityBasedStatus cbs3 = new CapturabilityBasedStatus(random);

      Path testFilePath = TEST_ROOT_PATH.resolve("TestSerialize" + CapturabilityBasedStatus.class.getSimpleName() + ".ibag");

      DataOutputStream fileDataOutputStream = FileTools.getFileDataOutputStream(testFilePath);
      kryoStreamSerializer.write(fileDataOutputStream, cbs1);
      kryoStreamSerializer.write(fileDataOutputStream, cbs2);
      kryoStreamSerializer.write(fileDataOutputStream, cbs3);
      fileDataOutputStream.close();

      DataInputStream fileDataInputStream = FileTools.getFileDataInputStream(testFilePath);
      CapturabilityBasedStatus cbs1Out = (CapturabilityBasedStatus) kryoStreamDeSerializer.read(fileDataInputStream);
      CapturabilityBasedStatus cbs2Out = (CapturabilityBasedStatus) kryoStreamDeSerializer.read(fileDataInputStream);
      CapturabilityBasedStatus cbs3Out = (CapturabilityBasedStatus) kryoStreamDeSerializer.read(fileDataInputStream);
      fileDataInputStream.close();

      assertPacketsEqual(cbs1, cbs1Out);
      assertPacketsEqual(cbs2, cbs2Out);
      assertPacketsEqual(cbs3, cbs3Out);
   }

   private void assertPacketsEqual(CapturabilityBasedStatus cbs, CapturabilityBasedStatus cbsOut)
   {
      assertTrue(cbs != null);
      assertTrue(cbs.getClass().equals(CapturabilityBasedStatus.class));
      assertTrue(cbsOut.getClass().equals(CapturabilityBasedStatus.class));
   }
}
