package com.yobotics.simulationconstructionset.util.globalParameters;

import static org.junit.Assert.*;
import junit.framework.TestCase;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.yobotics.simulationconstructionset.YoVariableType;

public class DoubleGlobalParameterTest //extends TestCase
{
//    private DoubleGlobalParameter doubleGlobalParameter;
    private final double DEFAULT_VALUE = 11.99;
    private static int counter = 0;
    
    
    @Before
    public void setUp() throws Exception
    {
    }

    @After
    public void tearDown() throws Exception
    {
    }

    
    
//    @Test
//    public void testGetValueInStringFormat()
//    {
//	Double doubleValue = new Double(DEFAULT_VALUE);
//	System.out.println(doubleValue.toString());
//	System.out.println(doubleGlobalParameter.getValueInStringFormat() + "***");
//	assertEquals(doubleValue.toString(), doubleGlobalParameter.getValueInStringFormat());
//    }

   
    @Test
    public void testGetValue()
    {	
	DoubleGlobalParameter doubleGlobalParameter = new DoubleGlobalParameter("testParameter" + counter++, "test description", DEFAULT_VALUE, null);
	assertEquals(DEFAULT_VALUE, doubleGlobalParameter.getValue());
    }
    
    @Test
    public void testSetValue()
    {
	DoubleGlobalParameter doubleGlobalParameter = new DoubleGlobalParameter("testParameter" + counter++, "test description", DEFAULT_VALUE, null);

	double newValue = -0.045;
	doubleGlobalParameter.set(newValue);
	assertEquals(newValue, doubleGlobalParameter.getValue());
	
	newValue = 1100.345;
	doubleGlobalParameter.set(newValue, "setting");
	assertEquals(newValue, doubleGlobalParameter.getValue());
	
	newValue = 1100.345;
	doubleGlobalParameter.setOnlyIfChange(newValue, "setting");
	assertEquals(newValue, doubleGlobalParameter.getValue());
	
	newValue = -906.345;
	doubleGlobalParameter.setOnlyIfChange(newValue, "setting");
	assertEquals(newValue, doubleGlobalParameter.getValue());
    }
    
    @Test
    public void testGetYoVariableType()
    {
	DoubleGlobalParameter doubleGlobalParameter = new DoubleGlobalParameter("testParameter" + counter++, "test description", DEFAULT_VALUE, null);

	assertEquals(YoVariableType.DOUBLE, doubleGlobalParameter.getYoVariableType());
    }
    
    

//    @Test
//    public void testSetOnlyIfChangeDouble()
//    {
//	fail("Not yet implemented");
//    }
//
//  
//    @Test
//    public void testSetDouble()
//    {
//	fail("Not yet implemented");
//    }
//
//    @Test
//    public void testSetDoubleString()
//    {
//	fail("Not yet implemented");
//    }
    
    
//    private class GlobalParameterChangedListener implements GlobalParameterChangedListener
//    {
//	
//    }

}
