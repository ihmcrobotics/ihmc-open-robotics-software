package us.ihmc.robotics.graphics;

import static org.junit.Assert.assertArrayEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePose3D;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.graphics.YoGraphicPolynomial3D;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;

public class YoGraphicPolynomial3DTest
{
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 30000)
   public void testRemoteYoGraphicVariableOrdering()
   {
      Random random = new Random(345345L);

      for (int iteration = 0; iteration < 10; iteration++)
      {
         String name = "writer";
         YoVariableRegistry registry = new YoVariableRegistry("writerRegistry");
         int numberOfPolynomials = random.nextInt(20) + 3;

         YoFramePose3D poseToPolynomialFrame = new YoFramePose3D(name + "Pose", ReferenceFrame.getWorldFrame(), registry);

         List<YoPolynomial3D> yoPolynomial3Ds = new ArrayList<>();
         List<YoDouble> waypointTimes = new ArrayList<>();

         for (int i = 0; i < numberOfPolynomials; i++)
         {
            YoPolynomial xPolynomial = new YoPolynomial(name + "XPoly" + i, random.nextInt(20) + 1, registry);
            YoPolynomial yPolynomial = new YoPolynomial(name + "YPoly" + i, random.nextInt(20) + 1, registry);
            YoPolynomial zPolynomial = new YoPolynomial(name + "ZPoly" + i, random.nextInt(20) + 1, registry);
            yoPolynomial3Ds.add(new YoPolynomial3D(xPolynomial, yPolynomial, zPolynomial));
            waypointTimes.add(new YoDouble(name + "WaypointTime" + i, registry));
         }

         double radius = random.nextDouble();
         int resolution = random.nextInt(50);
         int radialResolution = random.nextInt(50);
         YoGraphicPolynomial3D yoGraphicWriter = new YoGraphicPolynomial3D(name, poseToPolynomialFrame, yoPolynomial3Ds, waypointTimes, radius, resolution,
                                                                           radialResolution, registry);

         YoVariable<?>[] allWriterYoVariables = yoGraphicWriter.getVariables();
         double[] allWriterConstants = new double[yoGraphicWriter.getConstants().length];
         for (int i = 0; i < yoGraphicWriter.getConstants().length; i++)
            allWriterConstants[i] = yoGraphicWriter.getConstants()[i];

         YoGraphicPolynomial3D yoGraphicReader = YoGraphicPolynomial3D.createAsRemoteYoGraphic("reader", allWriterYoVariables, allWriterConstants);
         YoVariable<?>[] allReaderYoVariables = yoGraphicReader.getVariables();
         double[] allReaderConstants = new double[yoGraphicReader.getConstants().length];
         for (int i = 0; i < yoGraphicReader.getConstants().length; i++)
            allReaderConstants[i] = yoGraphicReader.getConstants()[i];

         assertArrayEquals(allWriterYoVariables, allReaderYoVariables);
         assertArrayEquals(allWriterConstants, allReaderConstants, 1e-7);
      }
   }
}
