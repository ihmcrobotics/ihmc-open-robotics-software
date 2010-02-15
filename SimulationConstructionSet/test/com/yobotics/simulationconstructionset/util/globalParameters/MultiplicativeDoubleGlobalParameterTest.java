package com.yobotics.simulationconstructionset.util.globalParameters;

import static org.junit.Assert.*;

import javax.management.RuntimeErrorException;

import org.junit.Test;

public class MultiplicativeDoubleGlobalParameterTest
{
    private static int counter = 0;
    
    @Test
    public void testSetThrowsException()
    {
	SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();

	
	double valueA = 4.67;
	double valueB = -765.7654;
	
	DoubleGlobalParameter doubleGlobalParameterA = new DoubleGlobalParameter("testParameter" + counter++, "test description", valueA, systemOutGlobalParameterChangedListener);
	DoubleGlobalParameter doubleGlobalParameterB = new DoubleGlobalParameter("testParameter" + counter++, "test description", valueB, systemOutGlobalParameterChangedListener);

	MultiplicativeDoubleGlobalParameter multiplicativeDoubleGlobalParameter = new MultiplicativeDoubleGlobalParameter("testMulti" + counter++,
		"multiplicative parameter", new DoubleGlobalParameter[]{doubleGlobalParameterA, doubleGlobalParameterB}, systemOutGlobalParameterChangedListener);
	
	try
	{
	    multiplicativeDoubleGlobalParameter.set(10.0);
	    fail();
	}
	catch (Exception e)
	{
//	    throw new RuntimeException("it was supposed to throw an exception");
	}
    }

    @Test
    public void testMultiplicativeDoubleGlobalParameter()
    {
	SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();

	double valueA = 4.67;
	double valueB = -765.7654;
	
	DoubleGlobalParameter doubleGlobalParameterA = new DoubleGlobalParameter("testParameter" + counter++, "test description", valueA, systemOutGlobalParameterChangedListener);
	DoubleGlobalParameter doubleGlobalParameterB = new DoubleGlobalParameter("testParameter" + counter++, "test description", valueB, systemOutGlobalParameterChangedListener);

	MultiplicativeDoubleGlobalParameter multiplicativeDoubleGlobalParameter = new MultiplicativeDoubleGlobalParameter("testMulti" + counter++,
		"multiplicative parameter", new DoubleGlobalParameter[]{doubleGlobalParameterA, doubleGlobalParameterB}, systemOutGlobalParameterChangedListener);
	
	
	assertEquals(valueA * valueB, multiplicativeDoubleGlobalParameter.getValue());
    }
    
    
    @Test
    public void testMultiplicativeDoubleGlobalParameterUpdate()
    {
	SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();

	
	double valueA = 4.67;
	double valueB = -765.7654;
	
	DoubleGlobalParameter doubleGlobalParameterA = new DoubleGlobalParameter("testParameter" + counter++, "test description", valueA, systemOutGlobalParameterChangedListener);
	DoubleGlobalParameter doubleGlobalParameterB = new DoubleGlobalParameter("testParameter" + counter++, "test description", valueB, systemOutGlobalParameterChangedListener);

	MultiplicativeDoubleGlobalParameter multiplicativeDoubleGlobalParameter = new MultiplicativeDoubleGlobalParameter("testMulti" + counter++,
		"multiplicative parameter", new DoubleGlobalParameter[]{doubleGlobalParameterA, doubleGlobalParameterB}, systemOutGlobalParameterChangedListener);
	
	
	valueA = -795.09;
	doubleGlobalParameterA.set(valueA);	
	assertEquals(valueA * valueB, multiplicativeDoubleGlobalParameter.getValue());

	valueB = 0.58674;
	doubleGlobalParameterB.set(valueB);
	assertEquals(valueA * valueB, multiplicativeDoubleGlobalParameter.getValue());

	valueA = 0.0;
	valueB = 345675.866;
	doubleGlobalParameterA.set(valueA);
	doubleGlobalParameterB.set(valueB);
	assertEquals(valueA * valueB, multiplicativeDoubleGlobalParameter.getValue());
    }


}
