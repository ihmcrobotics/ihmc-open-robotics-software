package us.ihmc.robotics.math.filters;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

public class WeightedAverageYoFrameVector3DTest
{
   private static final int iters = 1000;
   private static final double epsilon = 1e-8;

   private YoVariableRegistry registry;
   private YoFrameVector3D yoVariable1ToAverage;
   private YoFrameVector3D yoVariable2ToAverage;
   private YoDouble yoVariable1Weight;
   private YoDouble yoVariable2Weight;
   private WeightedAverageYoFrameVector3D averageVariable;

   @BeforeEach
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      yoVariable1ToAverage = new YoFrameVector3D("variable1ToFilter", ReferenceFrame.getWorldFrame(), registry);
      yoVariable2ToAverage = new YoFrameVector3D("variable2ToFilter", ReferenceFrame.getWorldFrame(), registry);
      yoVariable1Weight = new YoDouble("variable1Weight", registry);
      yoVariable2Weight = new YoDouble("variable2Weight", registry);


      averageVariable = new WeightedAverageYoFrameVector3D("averageVariable", ReferenceFrame.getWorldFrame(), registry);
      averageVariable.addFrameVector3DToAverage(yoVariable1Weight, yoVariable1ToAverage);
      averageVariable.addFrameVector3DToAverage(yoVariable2Weight, yoVariable2ToAverage);
   }

   @AfterEach
   public void tearDown()
   {
      registry = null;
      yoVariable1ToAverage= null;
      yoVariable2ToAverage = null;
      yoVariable1Weight = null;
      yoVariable2Weight = null;
      averageVariable = null;
   }

	@Test
   public void testConstructors_Set_Get()
   {
      WeightedAverageYoBoolean bool = new WeightedAverageYoBoolean("stringInt", registry);
      assertFalse(bool.getBooleanValue());

      bool.set(true);
      assertTrue(bool.getBooleanValue());

      bool.set(false);
      assertFalse(bool.getBooleanValue());
   }

	@Test
   public void testUpdate()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < iters; iter++)
      {
         double weight1 = RandomNumbers.nextDouble(random, 0.0, 1000.0);
         double weight2 = RandomNumbers.nextDouble(random, 0.0, 1000.0);
         double totalWeight = weight1 + weight2;

         Vector3DReadOnly variable1 = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3DReadOnly variable2 = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);

         yoVariable1Weight.set(weight1);
         yoVariable2Weight.set(weight2);

         yoVariable1ToAverage.set(variable1);
         yoVariable2ToAverage.set(variable2);

         FrameVector3D expectedAverage = new FrameVector3D();
         FrameVector3D scaled1 = new FrameVector3D(ReferenceFrame.getWorldFrame(), variable1);
         FrameVector3D scaled2 = new FrameVector3D(ReferenceFrame.getWorldFrame(), variable2);
         scaled1.scale(weight1 / totalWeight);
         scaled2.scale(weight2 / totalWeight);

         expectedAverage.add(scaled1);
         expectedAverage.add(scaled2);

         averageVariable.update();

         EuclidCoreTestTools.assertVector3DGeometricallyEquals(expectedAverage, averageVariable, epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(variable1, yoVariable1ToAverage, epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(variable2, yoVariable2ToAverage, epsilon);
      }

      YoDouble yoVariable3Weight = new YoDouble("variable3Weight", registry);
      YoFrameVector3D yoVariable3ToAverage = new YoFrameVector3D("variable3ToFilter", ReferenceFrame.getWorldFrame(), registry);

      averageVariable.addFrameVector3DToAverage(yoVariable3Weight, yoVariable3ToAverage);

      for (int iter = 0; iter < iters; iter++)
      {
         double weight1 = RandomNumbers.nextDouble(random, 0.0, 1000.0);
         double weight2 = RandomNumbers.nextDouble(random, 0.0, 1000.0);
         double weight3 = RandomNumbers.nextDouble(random, 0.0, 1000.0);
         double totalWeight = weight1 + weight2 + weight3;

         Vector3DReadOnly variable1 = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3DReadOnly variable2 = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         Vector3DReadOnly variable3 = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);

         yoVariable1Weight.set(weight1);
         yoVariable2Weight.set(weight2);
         yoVariable3Weight.set(weight3);

         yoVariable1ToAverage.set(variable1);
         yoVariable2ToAverage.set(variable2);
         yoVariable3ToAverage.set(variable3);

         FrameVector3D expectedAverage = new FrameVector3D();
         FrameVector3D scaled1 = new FrameVector3D(ReferenceFrame.getWorldFrame(), variable1);
         FrameVector3D scaled2 = new FrameVector3D(ReferenceFrame.getWorldFrame(), variable2);
         FrameVector3D scaled3 = new FrameVector3D(ReferenceFrame.getWorldFrame(), variable3);
         scaled1.scale(weight1 / totalWeight);
         scaled2.scale(weight2 / totalWeight);
         scaled3.scale(weight3 / totalWeight);

         expectedAverage.add(scaled1);
         expectedAverage.add(scaled2);
         expectedAverage.add(scaled3);

         averageVariable.update();

         EuclidCoreTestTools.assertVector3DGeometricallyEquals(expectedAverage, averageVariable, epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(variable1, yoVariable1ToAverage, epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(variable2, yoVariable2ToAverage, epsilon);
         EuclidCoreTestTools.assertVector3DGeometricallyEquals(variable3, yoVariable3ToAverage, epsilon);
      }

   }


}