package us.ihmc.robotics.math.filters;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * 
 * @author 	cwh
 * 
 * 		   	This class is intended to be used with the TransferFunctionDiscretizer, which creates a discrete representation of a continuous
 * 			transfer function using Tustin's method. This class is expected to be a part of a strict timing loop of a known frequency, whose 
 * 			frequency should match what was given to the TransferFunctionDiscretizer. If this isn't correct, the discrete representation will
 * 			not match the expected performance. 
 * 
 * 			An example simulation of this class can be found in the YoFilteredDoubleSimulation.java class and Tests can be found in the 
 * 			YoFilteredDoubleTests.java class. More details can be found in the associated paper: 
 * 			"Software Implementation of Digital Filtering via Tustin's Bilinear Transform" by Connor Herron. 
 *
 */

public class YoFilteredDouble {
	
	private TransferFunctionDiscretizer filter;
	
	private DMatrixRMaj inputCoefficients;
	private DMatrixRMaj outputCoefficients;
	
	private DMatrixRMaj inputHistory;
	private DMatrixRMaj outputHistory;

	private final YoDouble filteredYoDouble;
	private final YoDouble rawYoDouble;
	private boolean firstTick = true;
	private double [] inTemp;
	private double [] outTemp;
	private double newFilteredValue;
	private boolean safeStartup;

	public YoFilteredDouble(String name, YoRegistry registry, TransferFunctionDiscretizer filter) {
		this(name, registry, filter, true);
	}
	
	/**
	 * 
	 * @param name - class name
	 * @param registry - YoRegistry to add YoVariables to.
	 * @param filter - Filter object from trec-simulation-tools.YoVariables.
	 * @param safeStartup -	Recommended to set true. Boolean refers to whether you would like to set the  
	 * 						to avoid input/output histories of zero. Without this enabled, you will start the first ticks with high outputs.
	 */
	public YoFilteredDouble(String name, YoRegistry registry, TransferFunctionDiscretizer filter, boolean safeStartup) {
		this.filter = filter;
		
		inputCoefficients 	= this.filter.getInputCoefficients();
		outputCoefficients 	= this.filter.getOutputCoefficients();
		
		inputHistory 	= new DMatrixRMaj (1, inputCoefficients.numCols);
		outputHistory 	= new DMatrixRMaj (1, outputCoefficients.numCols);
		this.safeStartup = safeStartup;
		
		inTemp = new double[inputCoefficients.numCols];
		outTemp = new double[outputCoefficients.numCols];
		
		filteredYoDouble 	= new YoDouble(name + "_filtered", registry);
		rawYoDouble 	 	= new YoDouble(name + "_raw", registry);
	}
	
	/*
	 *  Use this method in initialization to start the filtered value somewhere other than zero. If you want to set an offset, 
	 *  set skipStartup = false for initialization to avoid initial values at zero. It's placed separately from the constructor because sometimes we want to set the offset to sensor
	 *  values that haven't been measured yet.
	 */
	private void setOffset(double offset) {
		for (int i = 0; i < outputCoefficients.numCols; i++) {
			outTemp[i] = offset;
		}
		
		for (int i = 0; i < inputCoefficients.numCols; i++) {
			inTemp[i] = offset;
		}
		
		// Set output history to the offset value.
		outputHistory.set(1, outTemp.length, false, outTemp);
		
		// Set input history back to zero value.
		inputHistory.set(1, inTemp.length, false, inTemp);
	}
	
	// Returns raw input value.
	public double getRawValue() {
		return rawYoDouble.getDoubleValue();
	}
	
	// Returns filtered output value.
	public double getFilteredValue() {
		return filteredYoDouble.getDoubleValue();
	}
	
	// Computes the new filtered value based on input/output history and Filter. Sets new filtered and unfiltered values.
	public void set(double newRawValue) {
		rawYoDouble.set(newRawValue);

		// If we want a safe startup for initialization, set the input/output histories to the first input.
		if (this.safeStartup && firstTick) {
			setOffset(newRawValue);
			firstTick = false;
		}
		
		// Update input history.
		System.arraycopy(inputHistory.data, 0, inTemp, 1, inputHistory.data.length - 1);
		inTemp[0] = newRawValue;
		inputHistory.set(1, inTemp.length, false, inTemp);
		
		// Solve for new filtered value.
		newFilteredValue = CommonOps_DDRM.dot(inputCoefficients, inputHistory) + CommonOps_DDRM.dot(outputCoefficients, outputHistory);
		
		// Set new filtered value.
		filteredYoDouble.set(newFilteredValue);
		
		// Update output history.
		System.arraycopy(outputHistory.data, 0, outTemp, 1, outputHistory.data.length - 1);
		outTemp[0] = newFilteredValue;
		outputHistory.set(1, outTemp.length, false, outTemp);
	}
}