package us.ihmc.robotics.math.filters;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

/**
 * 
 * @author cwh
 * 
 * 		   	This class is intended to be used with the YoFilteredDouble object to filter a signal with a desired causal transfer function. 
 * 			The TransferFunctionDiscretizer creates a discrete filter from a continuous transfer function representation:
 *  		   
 *  			G(s) = k * (b_0*s^m + b_1*s^{m-1} + ... + b_{m-1}*s + b_m)/(a_0*s^n + a_1*s^{n-1} + ... + a_{n-1}*s + a_n)
 *  
 *  		Where n >= m for the transfer function to be causal. This class does not check whether your desired transfer function is stable.
 * 			An example simulation of this class can be found in the YoFilteredDoubleSimulation.java class. More details can be found in the 
 * 			associated paper: "Software Implementation of Digital Filtering via Tustin's Bilinear Transform" by Connor Herron.
 *
 */
public class TransferFunctionDiscretizer {
	
	// Used for temporary private calculations.
	private DMatrixRMaj outputVec;
	private DMatrixRMaj tempVec;
	private int numCols;
	
	// Desired Transfer function numerator and denominator.
	private DMatrixRMaj numerator;
	private DMatrixRMaj denominator;
	
	// Input/Output Coefficients.
	private DMatrixRMaj inputCoefficients;
	private DMatrixRMaj outputCoefficients;
	
	private double k;
	private double p;
	private double fs;
	private double output;
	private int n;
	private int m;
	private String name;
	
	public TransferFunctionDiscretizer(ContinuousTransferFunction tf, double sampleFrequency) {
		this(tf.getName(), tf, sampleFrequency);
	}
	
	/*
	 * @param name Name of the filter.
	 * @param k Transfer Function Gain
	 * @param numerator Order of continuous polynomial coefficients from largest to smallest order, i.e. [b_0, b_1, ... , b_{m-1}, b_{m}].
	 * @param demoninator Order of continuous polynomial coefficients from largest to smallest order, i.e. [a_0, a_1, ..., a_{n-1}, a_n].
	 * @param sampleFrequency Frequency [Hz] which this variable will be updated. Filter performance will be worse if this is not accurate.
	 */
	public TransferFunctionDiscretizer(String name, ContinuousTransferFunction tf, double sampleFrequency) {
		
		this.name = name;
		
		m = tf.getNumerator().length;
		n = tf.getDenominator().length;
		
		// Handles multiple cases of m and n for matrix sizes.
		if (m > n) {
			throw new IllegalArgumentException("Transfer Function is non-causal!");
		}
		else if (m < n) {
			double []d = new double[n];
			System.arraycopy(tf.getNumerator(), 0, d, n-m, m);
			this.numerator 		= new DMatrixRMaj(1, d.length, false, d);
		}
		else {
			this.numerator 		= new DMatrixRMaj(1, m, false, tf.getNumerator());
		}
		
		this.denominator 	= new DMatrixRMaj(1, n, false, tf.getDenominator());
		
		this.k = tf.getGain();
		this.fs = sampleFrequency;
		this.p = 1/(2*this.fs);
		
		// Apply gain across numerator.
		CommonOps_DDRM.scale(k, this.numerator);
		
		// Solve for the corresponding input/output filter coefficients.
		buildFilter();
	}
	
	private void buildFilter() {
		
		numerator = solveFx_to_Fz(numerator);
		denominator = solveFx_to_Fz(denominator);
		
		// Solve for outputCoefficients.
		inputCoefficients = numerator.copy();
		CommonOps_DDRM.scale(1.0/denominator.get(0), inputCoefficients);
		
		// Solve for inputCoefficients.
		double []d = new double[n-1];
		System.arraycopy(denominator.data, 1, d, 0, denominator.numCols - 1);
		outputCoefficients = new DMatrixRMaj(1, d.length, false, d);
		CommonOps_DDRM.scale(-1.0/denominator.get(0), outputCoefficients);
	}
	
	private double calc;
	
	private DMatrixRMaj solveFx_to_Fz(DMatrixRMaj Fx) {
		numCols = Fx.numCols;
		DMatrixRMaj Fz = new DMatrixRMaj(1,numCols);
		
		/*
		 *  1) 	Dividing by s^n. Multiply out the correct number of gains.
		 *  	Converting Numerator & Denominator to F(x) form, where they
		 *  	are backwards in this form. 
		 */
		calc = 1/p;
		for (int i = 0; i < n; i++) {
			calc *= p;
			Fx.times(i, calc);
		}
		Fx = reverseVector(Fx);
		
		// 2) Apply Synthetic Division. F(x) -> F(x+1).
		Fx = syntheticDivision(Fx, 1.0);
		
		// 3)  Flip Coefficients. F(x+1) -> F(1/x + 1).
		Fx = reverseVector(Fx);
		
		// 4) Convert F(1/x + 1) -> F(2/x + 1).
		calc = 1.0/2.0;
		for (int i = 0; i < numCols; i++) {
			calc *= 2.0;
			Fx.times(i, calc);
		}
		
		// 5) Convert F(2/x + 1) -> F(2/(x-1) + 1).
		Fz = syntheticDivision(Fx, -1.0);
		
		return Fz;
	}
	
	/**
	 *  Apply Synthetic division used for two different steps: 
	 *  
	 *  	1) F(x) -> F(x+1) 
	 *   	2) F(2/x + 1) -> F(2/(x-1) + 1).
	 *   
	 *   This code 
	 * @param inputVec
	 * @return
	 */
	private DMatrixRMaj syntheticDivision(DMatrixRMaj inputVec, double sign) {
		numCols 	= inputVec.numCols;
		tempVec 	= inputVec;
		outputVec 	= inputVec;
		
		// Might need to be NumCols - 1
		for (int i = 0; i < numCols; i++) {
			for (int j = 0; j < (numCols-1-i); j++) {
				tempVec.set(j+1, tempVec.get(j+1) + sign*tempVec.get(j));
			}
			outputVec.set((numCols-1-i), tempVec.get(numCols-1-i));
		}
		
		return outputVec;
	}
	
	// Reverse input Vector from Left to Right.
	private DMatrixRMaj reverseVector(DMatrixRMaj inputVec) {
		numCols = inputVec.numCols;
		outputVec = new DMatrixRMaj(1,numCols);
		for (int i = 0; i < numCols; i++) {
			outputVec.set(i, inputVec.get(numCols-1-i));
		}
		return outputVec;
	}
	
	public DMatrixRMaj getInputCoefficients() {
		return inputCoefficients.copy();
	}
	
	public DMatrixRMaj getOutputCoefficients() {
		return outputCoefficients.copy();
	}
	
	public String getName() {
		return name;
	}

}
