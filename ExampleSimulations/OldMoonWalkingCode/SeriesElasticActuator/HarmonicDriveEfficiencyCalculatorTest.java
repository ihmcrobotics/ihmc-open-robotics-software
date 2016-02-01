package us.ihmc.moonwalking.models.SeriesElasticActuator;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class HarmonicDriveEfficiencyCalculatorTest
{

    private static final double RPM_2_RAD_PER_SEC = 2.0 * Math.PI / 60.0;
    
    @Test
    public void testJustSpeed()
    {
	HarmonicDriveEfficiencyCalculator harmonicDriveEfficiencyCalculator = new HarmonicDriveEfficiencyCalculator(null);
	
	double speed = 1000.0 * RPM_2_RAD_PER_SEC;
	double calculatedEfficiency = harmonicDriveEfficiencyCalculator.calculateGearboxEfficiency(speed, 1e12);
	System.out.println("calculatedEfficiency=" + calculatedEfficiency);
	assertEquals(0.7, calculatedEfficiency);
	
	speed = 3500.0 * RPM_2_RAD_PER_SEC;
	calculatedEfficiency = harmonicDriveEfficiencyCalculator.calculateGearboxEfficiency(speed, 1e12);
	System.out.println("calculatedEfficiency=" + calculatedEfficiency);
	assertEquals(0.55, calculatedEfficiency);
	

	speed = -3500.0 * RPM_2_RAD_PER_SEC;
	calculatedEfficiency = harmonicDriveEfficiencyCalculator.calculateGearboxEfficiency(speed, 1e12);
	System.out.println("calculatedEfficiency=" + calculatedEfficiency);
	assertEquals(0.55, calculatedEfficiency);
    }
    
    @Test
    public void testJustTorque()
    {
	
//	       double[] loadTorque = new double[]{0.0, 9.6, 19.2, 38.4, 48.0, 67.0, 96.0};
//	double[] compensationCoeff = new double[]{0.3, 0.3,  0.5,  0.72, 0.8,  0.9,  1.0};
	
	HarmonicDriveEfficiencyCalculator harmonicDriveEfficiencyCalculator = new HarmonicDriveEfficiencyCalculator(null);

	double speed = 0.0;
	double torque = 96.0;
	double calculatedEfficiency = harmonicDriveEfficiencyCalculator.calculateGearboxEfficiency(speed, torque);
	System.out.println("calculatedEfficiency=" + calculatedEfficiency);
	assertEquals(1.0, calculatedEfficiency);

	
	torque = 38.4;
	calculatedEfficiency = harmonicDriveEfficiencyCalculator.calculateGearboxEfficiency(speed, torque);
//	System.out.println("calculatedEfficiency=" + calculatedEfficiency);
	assertEquals(0.72, calculatedEfficiency);
	
	torque = 9.6;
	calculatedEfficiency = harmonicDriveEfficiencyCalculator.calculateGearboxEfficiency(speed, torque);
//	System.out.println("calculatedEfficiency=" + calculatedEfficiency);
	assertEquals(0.3, calculatedEfficiency);
	
	torque = 0.0;
	calculatedEfficiency = harmonicDriveEfficiencyCalculator.calculateGearboxEfficiency(speed, torque);
//	System.out.println("calculatedEfficiency=" + calculatedEfficiency);
	assertEquals(0.3, calculatedEfficiency);
	
	torque = 400.0;
	calculatedEfficiency = harmonicDriveEfficiencyCalculator.calculateGearboxEfficiency(speed, torque);
//	System.out.println("calculatedEfficiency=" + calculatedEfficiency);
	assertEquals(1.0, calculatedEfficiency);
	
	torque = -400.0;
	calculatedEfficiency = harmonicDriveEfficiencyCalculator.calculateGearboxEfficiency(speed, torque);
//	System.out.println("calculatedEfficiency=" + calculatedEfficiency);
	assertEquals(1.0, calculatedEfficiency);
    }
    

  

}
