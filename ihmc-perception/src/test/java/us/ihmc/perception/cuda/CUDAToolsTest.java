package us.ihmc.perception.cuda;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class CUDAToolsTest
{
   @Test
   public void testHasCUDADeviceOfAtLeast()
   {
      final String minimumDeviceName = "RTX 2080 Ti";

      // Slower GPUs than 2080 Ti
      Assertions.assertFalse(CUDATools.hasCUDADeviceOfAtLeast("GT 710", minimumDeviceName));
      Assertions.assertFalse(CUDATools.hasCUDADeviceOfAtLeast("GTX 650 Ti", minimumDeviceName));
      Assertions.assertFalse(CUDATools.hasCUDADeviceOfAtLeast("GTX 970", minimumDeviceName));
      Assertions.assertFalse(CUDATools.hasCUDADeviceOfAtLeast("GTX 980 Ti", minimumDeviceName));
      Assertions.assertFalse(CUDATools.hasCUDADeviceOfAtLeast("GTX 1080 Ti", minimumDeviceName));
      Assertions.assertFalse(CUDATools.hasCUDADeviceOfAtLeast("RTX 2060", minimumDeviceName));
      Assertions.assertFalse(CUDATools.hasCUDADeviceOfAtLeast("RTX 2060 SUPER", minimumDeviceName));
      Assertions.assertFalse(CUDATools.hasCUDADeviceOfAtLeast("RTX 2070 SUPER", minimumDeviceName));
      Assertions.assertFalse(CUDATools.hasCUDADeviceOfAtLeast("RTX 3060 Ti", minimumDeviceName));
      Assertions.assertFalse(CUDATools.hasCUDADeviceOfAtLeast("RTX 3070", minimumDeviceName));
      Assertions.assertFalse(CUDATools.hasCUDADeviceOfAtLeast("RTX 4060", minimumDeviceName));

      // Faster GPUs than 2080 Ti
      Assertions.assertTrue(CUDATools.hasCUDADeviceOfAtLeast("RTX 2080 Ti", minimumDeviceName));
      Assertions.assertTrue(CUDATools.hasCUDADeviceOfAtLeast("RTX 3080", minimumDeviceName));
      Assertions.assertTrue(CUDATools.hasCUDADeviceOfAtLeast("RTX 3080 Ti", minimumDeviceName));
      Assertions.assertTrue(CUDATools.hasCUDADeviceOfAtLeast("RTX 4080", minimumDeviceName));
      Assertions.assertTrue(CUDATools.hasCUDADeviceOfAtLeast("RTX 4080 Ti", minimumDeviceName));
      Assertions.assertTrue(CUDATools.hasCUDADeviceOfAtLeast("RTX 5090", minimumDeviceName));

      // Check that a 1080 Ti is less powerful than an RTX 4070
      Assertions.assertFalse(CUDATools.hasCUDADeviceOfAtLeast("GTX 1080 Ti", "RTX 4070"));
   }
}
