package us.ihmc.moonwalking.models.SeriesElasticActuator;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.sensorProcessing.encoder.SimulatedEncoder;

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

    @Test
    public void testSimpleTest()
    {
	assertEquals(1, 1);
    }

    @Test
    public void testConstructor()
    {
	double ticksPerPosition = 1.0;
	SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition);
    }

    @Test
    public void testGetTicksOne()
    {
	double ticksPerPosition = 10.0; // units of conversion per count
	SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition);

	double jointPosition = 50.0;

	int expecetedTicks = 500;
	simulatedEncoder.setActualPosition(jointPosition);

	assertEquals(expecetedTicks, simulatedEncoder.getEncoderTicks());
    }

    @Test
    public void testGetTicksTwo()
    {
	double conversion = 0.5; // units of conversion per count
	SimulatedEncoder simulatedEncoder = new SimulatedEncoder(conversion);

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
	double conversion = 0.5; // units of ticks per length
	SimulatedEncoder simulatedEncoder = new SimulatedEncoder(conversion);

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
	double ticksPerPosition = 10.0; // units of conversion per count
	SimulatedEncoder simulatedEncoder = new SimulatedEncoder(ticksPerPosition);

	double jointPosition = 0.5;

	int expecetedTicks = 5;
	simulatedEncoder.setActualPosition(jointPosition);

	assertEquals(expecetedTicks, simulatedEncoder.getEncoderTicks());
    }

    
    
    public void testGetPositionFromEncoder()
    {
	double conversion = 0.5; // units of ticks per length
	SimulatedEncoder simulatedEncoder = new SimulatedEncoder(conversion);

	double jointPosition = 0.9;

	int expecetedTicks = 0;
	double expectedPosition = expecetedTicks /conversion;
	simulatedEncoder.setActualPosition(jointPosition);

	assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder());

	jointPosition = 1.1;

	expecetedTicks = 1;
	expectedPosition = expecetedTicks /conversion;
	simulatedEncoder.setActualPosition(jointPosition);

	assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder());
    }
    
    @Test
    public void testGetPositionFromEncoderTwo()
    {
	double conversion = 10; // units of ticks per length
	SimulatedEncoder simulatedEncoder = new SimulatedEncoder(conversion);

	double jointPosition = 4.0;

	int expecetedTicks = 40;
	double expectedPosition = expecetedTicks /conversion;
	simulatedEncoder.setActualPosition(jointPosition);
	
	
	assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder());
    }
    
    @Test
    public void testGetPositionFromEncoderThree()
    {
	double conversion = 100; // units of ticks per length
	SimulatedEncoder simulatedEncoder = new SimulatedEncoder(conversion);

	double jointPosition = 1.0;

	double expectedPosition = 1.0;
	simulatedEncoder.setActualPosition(jointPosition);
	
	assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder());
    }
    
    
    @Test
    public void testGetPositionFromEncoderFour()
    {
	double conversion = 100; // units of ticks per length
	SimulatedEncoder simulatedEncoder = new SimulatedEncoder(conversion);

	double jointPosition = 5.0;

	double expectedPosition = 5.0;
	simulatedEncoder.setActualPosition(jointPosition);
	
	assertEquals(expectedPosition, simulatedEncoder.getPositionFromEncoder());
    }

    @Test
    public void testConverTicksToDistance() 
    {
	double conversion = 100; // units of ticks per length
	SimulatedEncoder simulatedEncoder = new SimulatedEncoder(conversion);
	
	int ticks = 100;
	assertEquals(1.0, simulatedEncoder.converTicksToDistance(ticks));
    }

}
