package us.ihmc.robotics.math.filters;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class OnlineSplineFitter3DTest
{

   public static final double ITERATIONS = 10000;

   @Test
   public void testAgainst3OnlineSplineFitter1D()
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int numberOfPoints = random.nextInt(100) + 10;

         double[] t = new double[numberOfPoints];
         double[] x = new double[numberOfPoints];
         double[] y = new double[numberOfPoints];
         double[] z = new double[numberOfPoints];

         for (int j = 0; j < numberOfPoints; j++)
         {
            t[j] = (j > 0 ? t[j - 1] : 0.0) + 0.01 * random.nextDouble();
            x[j] = random.nextDouble();
            y[j] = random.nextDouble();
            z[j] = random.nextDouble();
         }

         OnlineSplineFitter3D onlineSplineFitter3D = new OnlineSplineFitter3D(3, 20, 0.5);
         OnlineSplineFitter1D xFitter = new OnlineSplineFitter1D(3, 20, 0.5);
         OnlineSplineFitter1D yFitter = new OnlineSplineFitter1D(3, 20, 0.5);
         OnlineSplineFitter1D zFitter = new OnlineSplineFitter1D(3, 20, 0.5);

         for (int j = 0; j < numberOfPoints; j++)
         {
            onlineSplineFitter3D.recordNewPoint(t[j], x[j], y[j], z[j]);
            xFitter.recordNewPoint(t[j], x[j]);
            yFitter.recordNewPoint(t[j], y[j]);
            zFitter.recordNewPoint(t[j], z[j]);

            assertEquals(xFitter.getNumberOfPoints(), onlineSplineFitter3D.getNumberOfPoints());
            assertEquals(xFitter.isSplineInitialized(), onlineSplineFitter3D.isSplineInitialized());
            assertEquals(xFitter.getNewestPointTime(), onlineSplineFitter3D.getNewestPointTime());

            double evaluationTime = t[j] - 0.5 * random.nextDouble();
            double epsilon = 1.0e-12;
            assertEquals(xFitter.evaluateValueAt(evaluationTime), onlineSplineFitter3D.evaluateValueAt(evaluationTime).getX(), epsilon);
            assertEquals(yFitter.evaluateValueAt(evaluationTime), onlineSplineFitter3D.evaluateValueAt(evaluationTime).getY(), epsilon);
            assertEquals(zFitter.evaluateValueAt(evaluationTime), onlineSplineFitter3D.evaluateValueAt(evaluationTime).getZ(), epsilon);
            assertEquals(xFitter.evaluateRateAt(evaluationTime), onlineSplineFitter3D.evaluateRateAt(evaluationTime).getX(), epsilon);
            assertEquals(yFitter.evaluateRateAt(evaluationTime), onlineSplineFitter3D.evaluateRateAt(evaluationTime).getY(), epsilon);
            assertEquals(zFitter.evaluateRateAt(evaluationTime), onlineSplineFitter3D.evaluateRateAt(evaluationTime).getZ(), epsilon);
            assertEquals(xFitter.evaluateAccelerationAt(evaluationTime), onlineSplineFitter3D.evaluateAccelerationAt(evaluationTime).getX(), epsilon);
            assertEquals(yFitter.evaluateAccelerationAt(evaluationTime), onlineSplineFitter3D.evaluateAccelerationAt(evaluationTime).getY(), epsilon);
            assertEquals(zFitter.evaluateAccelerationAt(evaluationTime), onlineSplineFitter3D.evaluateAccelerationAt(evaluationTime).getZ(), epsilon);
         }
      }
   }
}
