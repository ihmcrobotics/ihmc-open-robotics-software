package us.ihmc.sensorProcessing.sensorProcessors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class ElasticityCompensatorYoVariableTest
{
   private static final double EPSILON = 1e-10;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testZeroJointTau1()
   {
      Random random = new Random(1561651L);
      
      String name = "compensator";
      YoVariableRegistry registry = new YoVariableRegistry("");
      DoubleYoVariable stiffness = new DoubleYoVariable("stiffness", registry);
      DoubleYoVariable rawJointPosition = new DoubleYoVariable("rawJointPosition", registry);
      DoubleYoVariable jointTau = new DoubleYoVariable("jointTau", registry);
      ElasticityCompensatorYoVariable elasticityCompensatorYoVariable = new ElasticityCompensatorYoVariable(name, registry);
      
      try
      {
         elasticityCompensatorYoVariable.update();
         fail("Should have thrown an exception.");
      }
      catch (NullPointerException e)
      {
         // Good
      }
      
      for (int i = 0; i < 1000; i++)
      {
         stiffness.set(RandomNumbers.nextDouble(random, 1.0, 10000000.0));
         elasticityCompensatorYoVariable.setStiffness(stiffness.getDoubleValue());
         rawJointPosition.set(RandomNumbers.nextDouble(random, 100.0));
         elasticityCompensatorYoVariable.update(rawJointPosition.getDoubleValue(), jointTau.getDoubleValue());
         
         assertEquals(rawJointPosition.getDoubleValue(), elasticityCompensatorYoVariable.getDoubleValue(), EPSILON);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testZeroJointTau2()
   {
      Random random = new Random(1561651L);
      
      String name = "compensator";
      YoVariableRegistry registry = new YoVariableRegistry("");
      DoubleYoVariable stiffness = new DoubleYoVariable("stiffness", registry);
      DoubleYoVariable rawJointPosition = new DoubleYoVariable("rawJointPosition", registry);
      DoubleYoVariable jointTau = new DoubleYoVariable("jointTau", registry);
      ElasticityCompensatorYoVariable elasticityCompensatorYoVariable = new ElasticityCompensatorYoVariable(name, stiffness, registry);
      try
      {
         elasticityCompensatorYoVariable.update();
         fail("Should have thrown an exception.");
      }
      catch (NullPointerException e)
      {
         // Good
      }
      
      for (int i = 0; i < 1000; i++)
      {
         stiffness.set(RandomNumbers.nextDouble(random, 1.0, 10000000.0));
         rawJointPosition.set(RandomNumbers.nextDouble(random, 100.0));
         elasticityCompensatorYoVariable.update(rawJointPosition.getDoubleValue(), jointTau.getDoubleValue());
         
         assertEquals(rawJointPosition.getDoubleValue(), elasticityCompensatorYoVariable.getDoubleValue(), EPSILON);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testZeroJointTau3()
   {
      Random random = new Random(1561651L);
      
      String name = "compensator";
      YoVariableRegistry registry = new YoVariableRegistry("");
      DoubleYoVariable stiffness = new DoubleYoVariable("stiffness", registry);
      DoubleYoVariable rawJointPosition = new DoubleYoVariable("rawJointPosition", registry);
      DoubleYoVariable jointTau = new DoubleYoVariable("jointTau", registry);
      ElasticityCompensatorYoVariable elasticityCompensatorYoVariable = new ElasticityCompensatorYoVariable(name, rawJointPosition, jointTau, registry);
      
      for (int i = 0; i < 1000; i++)
      {
         stiffness.set(RandomNumbers.nextDouble(random, 1.0, 10000000.0));
         elasticityCompensatorYoVariable.setStiffness(stiffness.getDoubleValue());
         rawJointPosition.set(RandomNumbers.nextDouble(random, 100.0));
         elasticityCompensatorYoVariable.update();
         
         assertEquals(rawJointPosition.getDoubleValue(), elasticityCompensatorYoVariable.getDoubleValue(), EPSILON);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testZeroJointTau4()
   {
      Random random = new Random(1561651L);
      
      String name = "compensator";
      YoVariableRegistry registry = new YoVariableRegistry("");
      DoubleYoVariable stiffness = new DoubleYoVariable("stiffness", registry);
      DoubleYoVariable rawJointPosition = new DoubleYoVariable("rawJointPosition", registry);
      DoubleYoVariable jointTau = new DoubleYoVariable("jointTau", registry);
      ElasticityCompensatorYoVariable elasticityCompensatorYoVariable = new ElasticityCompensatorYoVariable(name, stiffness, rawJointPosition, jointTau, registry);
      
      for (int i = 0; i < 1000; i++)
      {
         stiffness.set(RandomNumbers.nextDouble(random, 1.0, 10000000.0));
         rawJointPosition.set(RandomNumbers.nextDouble(random, 100.0));
         elasticityCompensatorYoVariable.update();
         
         assertEquals(rawJointPosition.getDoubleValue(), elasticityCompensatorYoVariable.getDoubleValue(), EPSILON);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testZeroStiffness4()
   {
      Random random = new Random(1561651L);
      
      String name = "compensator";
      YoVariableRegistry registry = new YoVariableRegistry("");
      DoubleYoVariable stiffness = new DoubleYoVariable("stiffness", registry);
      DoubleYoVariable rawJointPosition = new DoubleYoVariable("rawJointPosition", registry);
      DoubleYoVariable jointTau = new DoubleYoVariable("jointTau", registry);
      ElasticityCompensatorYoVariable elasticityCompensatorYoVariable = new ElasticityCompensatorYoVariable(name, stiffness, rawJointPosition, jointTau, registry);
      stiffness.set(0.0);
      
      for (int i = 0; i < 1000; i++)
      {
         rawJointPosition.set(RandomNumbers.nextDouble(random, 100.0));
         jointTau.set(RandomNumbers.nextDouble(random, 100.0));
         
         try
         {
            elasticityCompensatorYoVariable.update();
            fail("Should have thrown an exception");
         }
         catch (RuntimeException e)
         {
            // Good
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testZeroMaximumDeflection4()
   {
      Random random = new Random(1561651L);
      
      String name = "compensator";
      YoVariableRegistry registry = new YoVariableRegistry("");
      DoubleYoVariable stiffness = new DoubleYoVariable("stiffness", registry);
      DoubleYoVariable rawJointPosition = new DoubleYoVariable("rawJointPosition", registry);
      DoubleYoVariable jointTau = new DoubleYoVariable("jointTau", registry);
      ElasticityCompensatorYoVariable elasticityCompensatorYoVariable = new ElasticityCompensatorYoVariable(name, stiffness, rawJointPosition, jointTau, registry);
      elasticityCompensatorYoVariable.setMaximuDeflection(0.0);
      
      for (int i = 0; i < 1000; i++)
      {
         stiffness.set(RandomNumbers.nextDouble(random, 1.0, 10000000.0));
         rawJointPosition.set(RandomNumbers.nextDouble(random, 100.0));
         jointTau.set(RandomNumbers.nextDouble(random, 100.0));
         elasticityCompensatorYoVariable.update();
         
         assertEquals(rawJointPosition.getDoubleValue(), elasticityCompensatorYoVariable.getDoubleValue(), EPSILON);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDefaultMaximumDeflection4()
   {
      Random random = new Random(1561651L);
      
      String name = "compensator";
      YoVariableRegistry registry = new YoVariableRegistry("");
      DoubleYoVariable stiffness = new DoubleYoVariable("stiffness", registry);
      DoubleYoVariable rawJointPosition = new DoubleYoVariable("rawJointPosition", registry);
      DoubleYoVariable jointTau = new DoubleYoVariable("jointTau", registry);
      ElasticityCompensatorYoVariable elasticityCompensatorYoVariable = new ElasticityCompensatorYoVariable(name, stiffness, rawJointPosition, jointTau, registry);
      double maximumDeflection = 0.1;
      
      for (int i = 0; i < 10000; i++)
      {
         stiffness.set(RandomNumbers.nextDouble(random, 1.0, 100000.0));
         rawJointPosition.set(RandomNumbers.nextDouble(random, 100.0));
         jointTau.set(RandomNumbers.nextDouble(random, 10000.0));
         elasticityCompensatorYoVariable.update();
         
         double deflectedJointPosition = rawJointPosition.getDoubleValue() - MathTools.clipToMinMax(jointTau.getDoubleValue() / stiffness.getDoubleValue(), maximumDeflection);
         
         assertEquals(deflectedJointPosition, elasticityCompensatorYoVariable.getDoubleValue(), EPSILON);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInfiniteMaximumDeflection4()
   {
      Random random = new Random(1561651L);
      
      String name = "compensator";
      YoVariableRegistry registry = new YoVariableRegistry("");
      DoubleYoVariable stiffness = new DoubleYoVariable("stiffness", registry);
      DoubleYoVariable rawJointPosition = new DoubleYoVariable("rawJointPosition", registry);
      DoubleYoVariable jointTau = new DoubleYoVariable("jointTau", registry);
      ElasticityCompensatorYoVariable elasticityCompensatorYoVariable = new ElasticityCompensatorYoVariable(name, stiffness, rawJointPosition, jointTau, registry);
      elasticityCompensatorYoVariable.setMaximuDeflection(Double.POSITIVE_INFINITY);
      
      for (int i = 0; i < 10000; i++)
      {
         stiffness.set(RandomNumbers.nextDouble(random, 1.0, 100000.0));
         rawJointPosition.set(RandomNumbers.nextDouble(random, 100.0));
         jointTau.set(RandomNumbers.nextDouble(random, 10000.0));
         elasticityCompensatorYoVariable.update();
         
         double deflectedJointPosition = rawJointPosition.getDoubleValue() - jointTau.getDoubleValue() / stiffness.getDoubleValue();
         assertEquals(deflectedJointPosition, elasticityCompensatorYoVariable.getDoubleValue(), EPSILON);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRandomMaximumDeflection4()
   {
      Random random = new Random(1561651L);
      
      String name = "compensator";
      YoVariableRegistry registry = new YoVariableRegistry("");
      DoubleYoVariable stiffness = new DoubleYoVariable("stiffness", registry);
      DoubleYoVariable rawJointPosition = new DoubleYoVariable("rawJointPosition", registry);
      DoubleYoVariable jointTau = new DoubleYoVariable("jointTau", registry);
      ElasticityCompensatorYoVariable elasticityCompensatorYoVariable = new ElasticityCompensatorYoVariable(name, stiffness, rawJointPosition, jointTau, registry);
      
      for (int i = 0; i < 10000; i++)
      {
         stiffness.set(RandomNumbers.nextDouble(random, 1.0, 100000.0));
         rawJointPosition.set(RandomNumbers.nextDouble(random, 100.0));
         jointTau.set(RandomNumbers.nextDouble(random, 10000.0));
         double maximumDeflection = RandomNumbers.nextDouble(random, 0.0, 10.0);
         elasticityCompensatorYoVariable.setMaximuDeflection(maximumDeflection);
         elasticityCompensatorYoVariable.update();
         
         double deflectedJointPosition = rawJointPosition.getDoubleValue() - MathTools.clipToMinMax(jointTau.getDoubleValue() / stiffness.getDoubleValue(), maximumDeflection);
         
         assertEquals(deflectedJointPosition, elasticityCompensatorYoVariable.getDoubleValue(), EPSILON);
      }
   }
}
