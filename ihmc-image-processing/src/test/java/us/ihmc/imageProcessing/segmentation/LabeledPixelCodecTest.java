package us.ihmc.imageProcessing.segmentation;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

/**
 * @author Peter Abeles
 */
public class LabeledPixelCodecTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void write_read() throws IOException
   {
      String expectedName = "foo";
      String expectedColor = "voo";
      List<double[]> expectedValues = new ArrayList<double[]>();
      expectedValues.add(new double[] {1, 2, 3});
      expectedValues.add(new double[] {1.57, 2.79, 3.123});

      ByteArrayOutputStream out = new ByteArrayOutputStream(1024);
      LabeledPixelCodec.write(out, expectedName, expectedColor, expectedValues);

      InputStream in = new ByteArrayInputStream(out.toByteArray());


      LabeledPixelCodec.Set found = LabeledPixelCodec.read(in);

      assertTrue(found.label.compareTo(expectedName) == 0);
      assertTrue(found.colorModel.compareTo(expectedColor) == 0);
      assertEquals(expectedValues.size(), found.values.size());

      for (int i = 0; i < expectedValues.size(); i++)
      {
         double e[] = expectedValues.get(i);
         double f[] = found.values.get(i);

         assertEquals(e.length, f.length);

         for (int j = 0; j < e.length; j++)
         {
            assertEquals(e[j], f[j], 1e-8);
         }
      }
   }
}
