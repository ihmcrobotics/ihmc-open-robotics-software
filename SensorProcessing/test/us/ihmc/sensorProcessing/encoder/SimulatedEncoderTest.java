package us.ihmc.sensorProcessing.encoder;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class SimulatedEncoderTest
{
   @Before
   public void setUp() throws Exception
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleTest()
   {
      assertEquals(1, 1);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor()
   {
      double ticksPerPosition = 1.0;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");
      new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetTicksOne()
   {
      double ticksPerPosition = 10.0;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 50.0;

      int expecetedTicks = 500;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expecetedTicks, simulatedEncoder.getEncoderTicks());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetTicksTwo()
   {
      double ticksPerPosition = 0.5;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 50.0;

      int expecetedTicks = 25;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expecetedTicks, simulatedEncoder.getEncoderTicks());

      jointPosition = 0.0;

      expecetedTicks = 0;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expecetedTicks, simulatedEncoder.getEncoderTicks());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetTicksThree()
   {
      double ticksPerPosition = 0.5;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 0.9;

      int expecetedTicks = 0;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expecetedTicks, simulatedEncoder.getEncoderTicks());

      jointPosition = 1.1;

      expecetedTicks = 1;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expecetedTicks, simulatedEncoder.getEncoderTicks());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetTicksFour()
   {
      double ticksPerPosition = 10.0;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 0.5;

      int expecetedTicks = 5;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expecetedTicks, simulatedEncoder.getEncoderTicks());
   }



   public void testGetPositionFromEncoder()
   {
      double ticksPerPosition = 0.5;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 0.9;

      int expecetedTicks = 0;
      double expectedPosition = expecetedTicks / ticksPerPosition;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder(), 0.01);

      jointPosition = 1.1;

      expecetedTicks = 1;
      expectedPosition = expecetedTicks / ticksPerPosition;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder(), 0.01);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetPositionFromEncoderTwo()
   {
      double ticksPerPosition = 10;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 4.0;

      int expecetedTicks = 40;
      double expectedPosition = expecetedTicks / ticksPerPosition;
      simulatedEncoder.setActualPosition(jointPosition);


      assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder(), 0.01);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetPositionFromEncoderThree()
   {
      double ticksPerPosition = 100;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 1.0;

      double expectedPosition = 1.0;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder(), 0.01);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetPositionFromEncoderFour()
   {
      double ticksPerPosition = 100;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 5.0;

      double expectedPosition = 5.0;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder(), 0.01);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConverTicksToDistance()
   {
      double ticksPerPosition = 100;
      YoVariableRegistry parentRegistry = new YoVariableRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      int ticks = 100;
      assertEquals(1.0, simulatedEncoder.converTicksToDistance(ticks), 0.01);
   }

}
