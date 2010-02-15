package com.yobotics.simulationconstructionset.util.globalParameters;

import static org.junit.Assert.*;

import org.junit.*;

public class MultiplicativeDoubleGlobalParameterTest
{
    
    @Before
    public void setUp()
    {
       GlobalParameter.clearGlobalRegistry();
    }
    
    @After
    public void tearDown()
    {
       GlobalParameter.clearGlobalRegistry();
    }
    
    @Test
    public void testSetThrowsException()
    {
	SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();

	double valueA = 4.67;
	double valueB = -765.7654;
	
	DoubleGlobalParameter doubleGlobalParameterA = new DoubleGlobalParameter("testParameterA", "test description", valueA, systemOutGlobalParameterChangedListener);
	DoubleGlobalParameter doubleGlobalParameterB = new DoubleGlobalParameter("testParameterB", "test description", valueB, systemOutGlobalParameterChangedListener);

	MultiplicativeDoubleGlobalParameter multiplicativeDoubleGlobalParameter = new MultiplicativeDoubleGlobalParameter("testMulti",
		"multiplicative parameter", new DoubleGlobalParameter[]{doubleGlobalParameterA, doubleGlobalParameterB}, systemOutGlobalParameterChangedListener);
	
	try
	{
	    multiplicativeDoubleGlobalParameter.set(10.0);
	    fail();
	}
	catch (Exception e)
	{
	}
    }

    @Test
    public void testMultiplicativeDoubleGlobalParameter()
    {
	SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();

	double valueA = 4.67;
	double valueB = -765.7654;
	
	DoubleGlobalParameter doubleGlobalParameterA = new DoubleGlobalParameter("testParameterA", "test description", valueA, systemOutGlobalParameterChangedListener);
	DoubleGlobalParameter doubleGlobalParameterB = new DoubleGlobalParameter("testParameterB", "test description", valueB, systemOutGlobalParameterChangedListener);

	MultiplicativeDoubleGlobalParameter multiplicativeDoubleGlobalParameter = new MultiplicativeDoubleGlobalParameter("testMulti",
		"multiplicative parameter", new DoubleGlobalParameter[]{doubleGlobalParameterA, doubleGlobalParameterB}, systemOutGlobalParameterChangedListener);
	
	
	assertEquals(valueA * valueB, multiplicativeDoubleGlobalParameter.getValue());
    }
    
    
    @Test
    public void testMultiplicativeDoubleGlobalParameterUpdate()
    {
	SystemOutGlobalParameterChangedListener systemOutGlobalParameterChangedListener = new SystemOutGlobalParameterChangedListener();

	
	double valueA = 4.67;
	double valueB = -765.7654;
	
	DoubleGlobalParameter doubleGlobalParameterA = new DoubleGlobalParameter("testParameterA", "test description", valueA, systemOutGlobalParameterChangedListener);
	DoubleGlobalParameter doubleGlobalParameterB = new DoubleGlobalParameter("testParameterB", "test description", valueB, systemOutGlobalParameterChangedListener);

	MultiplicativeDoubleGlobalParameter multiplicativeDoubleGlobalParameter = new MultiplicativeDoubleGlobalParameter("testMulti",
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
