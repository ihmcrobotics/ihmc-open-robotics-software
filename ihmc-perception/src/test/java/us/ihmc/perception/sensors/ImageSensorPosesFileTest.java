package us.ihmc.perception.sensors;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.perception.RawImageTest;
import us.ihmc.sensors.ImageSensorPosesFile;

import java.net.URISyntaxException;
import java.nio.file.Path;
import java.time.Instant;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.*;

public class ImageSensorPosesFileTest
{
   @Test
   public void testReadingSensorPosesFile() throws URISyntaxException
   {
      ImageSensorPosesFile imageSensorPosesFile = new ImageSensorPosesFile(Path.of(RawImageTest.class.getResource("/test.sensorposes").toURI()));

      int numberOfFrames = imageSensorPosesFile.getNumberOfFrames();

      assertEquals(383, numberOfFrames);

      for (int i = 0; i < numberOfFrames; i++)
      {
         AtomicReference<Instant> acquisitionTime = new AtomicReference<>();
         Point3D position = new Point3D();
         Quaternion orientation = new Quaternion();

         imageSensorPosesFile.readFrameData(i, acquisitionTime::set, position, orientation);
      }
   }
}
