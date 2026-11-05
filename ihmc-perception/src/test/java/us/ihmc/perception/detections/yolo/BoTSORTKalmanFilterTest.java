package us.ihmc.perception.detections.yolo;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

@Tag("fast")
public class BoTSORTKalmanFilterTest
{
   @Test
   public void testFirstMeasurementCorrection()
   {
      KalmanFilter filter = new KalmanFilter();
      filter.initiate(new float[] {100.0f, 200.0f, 80.0f, 40.0f});
      filter.predict();
      filter.update(new float[] {110.0f, 180.0f, 80.0f, 40.0f});

      // For the first prediction, the position gain is 105/121 and velocity gain is 25/121.
      assertArrayEquals(new float[] {100.0f + 10.0f * 105.0f / 121.0f,
                                     200.0f - 20.0f * 105.0f / 121.0f,
                                     80.0f,
                                     40.0f,
                                     10.0f * 25.0f / 121.0f,
                                     -20.0f * 25.0f / 121.0f,
                                     0.0f,
                                     0.0f}, filter.getMean(), 1.0e-4f);
   }

   @Test
   public void testConstantVelocityAndMissedDetections()
   {
      KalmanFilter filter = new KalmanFilter();
      filter.initiate(new float[] {0.0f, 0.0f, 80.0f, 40.0f});
      for (int frame = 1; frame <= 100; frame++)
      {
         filter.predict();
         filter.update(new float[] {2.0f * frame, -3.0f * frame, 80.0f, 40.0f});
      }
      for (int frame = 0; frame < 10; frame++)
         filter.predict();

      assertArrayEquals(new float[] {220.0f, -330.0f, 80.0f, 40.0f, 2.0f, -3.0f, 0.0f, 0.0f}, filter.getMean(), 1.0e-2f);
   }

   @Test
   public void testSmallBoxesAndReinitialization()
   {
      KalmanFilter filter = new KalmanFilter();
      float[] measurement = {10.0f, 20.0f, 0.0f, 0.25f};
      filter.initiate(measurement);
      for (int frame = 0; frame < 1000; frame++)
      {
         filter.predict();
         filter.update(measurement);
      }
      assertArrayEquals(new float[] {10.0f, 20.0f, 0.0f, 0.25f, 0.0f, 0.0f, 0.0f, 0.0f}, filter.getMean(), 1.0e-5f);

      filter.setXY(30.0f, 40.0f);
      float[] copy = filter.getMean();
      copy[0] = -1.0f;
      assertEquals(30.0f, filter.getMean()[0]);
      assertEquals(40.0f, filter.getMean()[1]);

      float[] newMeasurement = {50.0f, 60.0f, 100.0f, 200.0f};
      KalmanFilter freshFilter = new KalmanFilter();
      freshFilter.initiate(newMeasurement);
      filter.initiate(newMeasurement);
      freshFilter.predict();
      filter.predict();
      freshFilter.update(measurement);
      filter.update(measurement);
      assertArrayEquals(freshFilter.getMean(), filter.getMean(), 1.0e-5f);
   }
}
