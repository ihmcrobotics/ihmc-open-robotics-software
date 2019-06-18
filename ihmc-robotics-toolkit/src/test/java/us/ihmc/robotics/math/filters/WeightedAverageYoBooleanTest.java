package us.ihmc.robotics.math.filters;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import static us.ihmc.robotics.Assert.*;

public class WeightedAverageYoBooleanTest
{
   private YoVariableRegistry registry;
   private YoBoolean yoVariable1ToAverage;
   private YoBoolean yoVariable2ToAverage;
   private YoDouble yoVariable1Weight;
   private YoDouble yoVariable2Weight;
   private WeightedAverageYoBoolean averagePositiveVariable;
   private WeightedAverageYoBoolean averageVariable;

   @BeforeEach
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      yoVariable1ToAverage = new YoBoolean("variable1ToFilter", registry);
      yoVariable2ToAverage = new YoBoolean("variable2ToFilter", registry);
      yoVariable1Weight = new YoDouble("variable1Weight", registry);
      yoVariable2Weight = new YoDouble("variable2Weight", registry);

      averagePositiveVariable = new WeightedAverageYoBoolean("averagePositiveVariable", registry, true);
      averagePositiveVariable.addBooleanToAverage(yoVariable1Weight, yoVariable1ToAverage);
      averagePositiveVariable.addBooleanToAverage(yoVariable2Weight, yoVariable2ToAverage);

      averageVariable = new WeightedAverageYoBoolean("averageVariable", registry);
      averageVariable.addBooleanToAverage(yoVariable1Weight, yoVariable1ToAverage);
      averageVariable.addBooleanToAverage(yoVariable2Weight, yoVariable2ToAverage);
   }

   @AfterEach
   public void tearDown()
   {
      registry = null;
      yoVariable1ToAverage= null;
      yoVariable2ToAverage = null;
      yoVariable1Weight = null;
      yoVariable2Weight = null;
      averagePositiveVariable = null;
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
      yoVariable1Weight.set(1.0);
      yoVariable2Weight.set(1.0);

      yoVariable1ToAverage.set(true);
      yoVariable2ToAverage.set(false);

      averagePositiveVariable.update();
      averageVariable.update();

      assertTrue(averagePositiveVariable.getBooleanValue());
      assertFalse(averageVariable.getBooleanValue());

      yoVariable1ToAverage.set(true);
      yoVariable2ToAverage.set(true);

      averagePositiveVariable.update();
      averageVariable.update();

      assertTrue(averagePositiveVariable.getBooleanValue());
      assertTrue(averageVariable.getBooleanValue());

      yoVariable1ToAverage.set(false);
      yoVariable2ToAverage.set(false);

      averagePositiveVariable.update();
      averageVariable.update();

      assertFalse(averagePositiveVariable.getBooleanValue());
      assertFalse(averageVariable.getBooleanValue());

      YoDouble yoVariable3Weight = new YoDouble("variable3Weight", registry);
      YoBoolean yoVariable3ToAverage = new YoBoolean("variable3ToFilter", registry);

      averagePositiveVariable.addBooleanToAverage(yoVariable3Weight, yoVariable3ToAverage);
      averageVariable.addBooleanToAverage(yoVariable3Weight, yoVariable3ToAverage);

      yoVariable3Weight.set(1.0);

      yoVariable1ToAverage.set(true);
      yoVariable2ToAverage.set(false);
      yoVariable3ToAverage.set(false);

      averagePositiveVariable.update();
      averageVariable.update();

      assertFalse(averagePositiveVariable.getBooleanValue());
      assertFalse(averageVariable.getBooleanValue());

      yoVariable2ToAverage.set(true);

      averagePositiveVariable.update();
      averageVariable.update();

      assertTrue(averagePositiveVariable.getBooleanValue());
      assertTrue(averageVariable.getBooleanValue());

      yoVariable3Weight.set(2.0);

      yoVariable1ToAverage.set(true);
      yoVariable2ToAverage.set(true);
      yoVariable3ToAverage.set(false);

      averagePositiveVariable.update();
      averageVariable.update();

      assertTrue(averagePositiveVariable.getBooleanValue());
      assertFalse(averageVariable.getBooleanValue());
   }


}