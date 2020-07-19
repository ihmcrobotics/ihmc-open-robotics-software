package us.ihmc.sensorProcessing.encoder;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimulatedEncoderTest
{
   @BeforeEach
   public void setUp() throws Exception
   {
   }

   @AfterEach
   public void tearDown() throws Exception
   {
   }

	@Test
   public void testSimpleTest()
   {
      assertEquals(1, 1);
   }

	@Test
   public void testConstructor()
   {
      double ticksPerPosition = 1.0;
      YoRegistry parentRegistry = new YoRegistry("parent");
      new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);
   }

	@Test
   public void testGetTicksOne()
   {
      double ticksPerPosition = 10.0;
      YoRegistry parentRegistry = new YoRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 50.0;

      int expecetedTicks = 500;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expecetedTicks, simulatedEncoder.getEncoderTicks());
   }

	@Test
   public void testGetTicksTwo()
   {
      double ticksPerPosition = 0.5;
      YoRegistry parentRegistry = new YoRegistry("parent");
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

	@Test
   public void testGetTicksThree()
   {
      double ticksPerPosition = 0.5;
      YoRegistry parentRegistry = new YoRegistry("parent");
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

	@Test
   public void testGetTicksFour()
   {
      double ticksPerPosition = 10.0;
      YoRegistry parentRegistry = new YoRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 0.5;

      int expecetedTicks = 5;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expecetedTicks, simulatedEncoder.getEncoderTicks());
   }



   @Test
   public void testGetPositionFromEncoder()
   {
      double ticksPerPosition = 0.5;
      YoRegistry parentRegistry = new YoRegistry("parent");
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

	@Test
   public void testGetPositionFromEncoderTwo()
   {
      double ticksPerPosition = 10;
      YoRegistry parentRegistry = new YoRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 4.0;

      int expecetedTicks = 40;
      double expectedPosition = expecetedTicks / ticksPerPosition;
      simulatedEncoder.setActualPosition(jointPosition);


      assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder(), 0.01);
   }

	@Test
   public void testGetPositionFromEncoderThree()
   {
      double ticksPerPosition = 100;
      YoRegistry parentRegistry = new YoRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 1.0;

      double expectedPosition = 1.0;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder(), 0.01);
   }

	@Test
   public void testGetPositionFromEncoderFour()
   {
      double ticksPerPosition = 100;
      YoRegistry parentRegistry = new YoRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      double jointPosition = 5.0;

      double expectedPosition = 5.0;
      simulatedEncoder.setActualPosition(jointPosition);

      assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder(), 0.01);
   }

	@Test
   public void testConverTicksToDistance()
   {
      double ticksPerPosition = 100;
      YoRegistry parentRegistry = new YoRegistry("parent");
      SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition, "simEncTest", parentRegistry);

      int ticks = 100;
      assertEquals(1.0, simulatedEncoder.converTicksToDistance(ticks), 0.01);
   }

}
