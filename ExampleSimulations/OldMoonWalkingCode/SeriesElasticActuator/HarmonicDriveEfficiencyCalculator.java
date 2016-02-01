package us.ihmc.moonwalking.models.SeriesElasticActuator;

import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.utilities.math.trajectories.LinearInterpolater;

public class HarmonicDriveEfficiencyCalculator
{
//    private static final double 
    private static int instanceCounter = 0;

    private LinearInterpolater linearInterpolaterViscousEfficiency;
    private LinearInterpolater linearInterpolaterCompensationCoef;
    private final YoVariableRegistry yoVariableRegistry;

    private final YoVariable viscousEfficiency;
    private final YoVariable compensationCoeff;
    
    public HarmonicDriveEfficiencyCalculator(YoVariableRegistry parentRegistry)
    {
	String suffix = "_" + instanceCounter;
	yoVariableRegistry = new YoVariableRegistry("HarmonicDrive" + suffix);
	
	viscousEfficiency = new YoVariable("viscousEfficiency" + suffix, "HD efficiency due to gearbox speed", yoVariableRegistry);
	compensationCoeff = new YoVariable("compensationCoeff" + suffix, "HD efficiency due to Torque", yoVariableRegistry);

	{
	    // These are in rpm
	    double[] speed = new double[]
	    { 0.0, 500.0, 1000.0, 2000.0, 3500.0, 4800.0, 7000.0 };
	    double[] speedInRadPerSec = new double[speed.length];
	    for (int i = 0; i < speed.length; i++)
	    {
		// Convert from rpm to rad/s
		speedInRadPerSec[i] = speed[i] * 2.0 * Math.PI / 60.0;
	    }

	    double[] efficiency = new double[]
	    { 1.0, 0.75, 0.70, 0.65, 0.55, 0.45, 0.3};

	    try
	    {
		linearInterpolaterViscousEfficiency = new LinearInterpolater(speedInRadPerSec, efficiency);
	    } catch (Exception e)
	    {
		// TODO Auto-generated catch block
		e.printStackTrace();
	    }
	}
	
	{
	    double[] loadTorque = new double[]{0.0, 9.6, 19.2, 38.4, 48.0, 67.0, 96.0};
	    double[] compensationCoeff = new double[]
	                         	    {0.3, 0.3, 0.5, 0.72, 0.8, 0.9, 1.0};

	    try
	    {
		linearInterpolaterCompensationCoef = new LinearInterpolater(loadTorque, compensationCoeff);
	    } catch (Exception e)
	    {
		// TODO Auto-generated catch block
		e.printStackTrace();
	    }
	}
	
	if (parentRegistry != null)
		parentRegistry.addChild(yoVariableRegistry);

	instanceCounter++;
	
	
    }
    
    
    public double calculateGearboxEfficiency(double gearboxSpeed, double gearboxTorque)
    {
	gearboxSpeed = Math.abs(gearboxSpeed);
	gearboxTorque = Math.abs(gearboxTorque);
	
	viscousEfficiency.val = linearInterpolaterViscousEfficiency.getPoint(gearboxSpeed);
	compensationCoeff.val = linearInterpolaterCompensationCoef.getPoint(gearboxTorque);
	return viscousEfficiency.val * compensationCoeff.val;
    }
    
    
}

//speed	efficiency
//0	100
//500	75
//1000	70
//2000	65
//3500	55
//4800	45
//7000	30


//Load Torque	Compensation Coeff
//0	0.3
//9.6	0.3
//19.2	0.5
//38.4	0.72
//48	0.8
//67.2	0.9
//96	1

