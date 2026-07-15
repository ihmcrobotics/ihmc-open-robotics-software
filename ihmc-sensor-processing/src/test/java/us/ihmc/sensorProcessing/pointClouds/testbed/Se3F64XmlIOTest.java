package us.ihmc.sensorProcessing.pointClouds.testbed;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;

import org.junit.jupiter.api.Test;

import georegression.struct.se.Se3_F64;

public class Se3F64XmlIOTest
{
   @Test
   public void testReadBundledEstimatedToModelXml()
   {
      Se3_F64 transform = Se3F64XmlIO.read(getClass().getResourceAsStream("/testbed/estimatedToModel.xml"));

      assertEquals(1.120123904615572, transform.T.x, 1.0e-12);
      assertEquals(0.634701050154963, transform.T.y, 1.0e-12);
      assertEquals(-0.0059950694713371, transform.T.z, 1.0e-12);
      assertEquals(-0.9998155100873913, transform.R.get(0, 0), 1.0e-12);
   }

   @Test
   public void testRoundTripWriteAndRead()
   {
      Se3_F64 original = Se3F64XmlIO.read(getClass().getResourceAsStream("/testbed/estimatedToModel.xml"));

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      Se3F64XmlIO.write(original, outputStream);

      Se3_F64 roundTripped = Se3F64XmlIO.read(new ByteArrayInputStream(outputStream.toByteArray()));

      for (int row = 0; row < 3; row++)
      {
         for (int col = 0; col < 3; col++)
            assertEquals(original.R.get(row, col), roundTripped.R.get(row, col), 1.0e-12);
      }
      assertEquals(original.T.x, roundTripped.T.x, 1.0e-12);
      assertEquals(original.T.y, roundTripped.T.y, 1.0e-12);
      assertEquals(original.T.z, roundTripped.T.z, 1.0e-12);
   }
}
