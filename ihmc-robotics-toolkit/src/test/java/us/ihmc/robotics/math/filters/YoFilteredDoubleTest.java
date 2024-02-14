package us.ihmc.robotics.math.filters;

import static org.junit.jupiter.api.Assertions.*;

import org.apache.commons.lang3.ArrayUtils;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;

class YoFilteredDoubleTest {

	@Test
	void testSafeStartup() {
		String name = "tf";
		double k = 5;
		double[] numerator 		= new double[] {1.0, 2.0, 3.0, 4.0, 5.0};
		double[] denominator 	= new double[] {5.0, 4.0, 3.0, 2.0, 1.0};
		
		YoRegistry registry = new YoRegistry("test");
		
		double[] numAndDen = ArrayUtils.addAll(numerator, denominator);
		
		ContinuousTransferFunction tf = new ContinuousTransferFunction(k, numerator, denominator);
		
		TransferFunctionDiscretizer filter = new TransferFunctionDiscretizer(tf, 1000.0);
		
		YoFilteredDouble test_var = new YoFilteredDouble("", registry, filter, true); // 
		
		// Set firstInput to 10.0
		double firstInput = 10.0;
		test_var.set(firstInput);
		
		// Safe startup means that the first output should be very close to 10.0.
		double delta = 0.000000001;
		assertEquals(firstInput, test_var.getFilteredValue(), delta);
	}
	
	@Test
	void testReturnRawValue() {
		String name = "tf";
		double k = 5;
		double[] numerator 		= new double[] {1.0, 2.0, 3.0, 4.0, 5.0};
		double[] denominator 	= new double[] {5.0, 4.0, 3.0, 2.0, 1.0};
		
		YoRegistry registry = new YoRegistry("test");
		
		double[] numAndDen = ArrayUtils.addAll(numerator, denominator);
		
		ContinuousTransferFunction tf = new ContinuousTransferFunction(k, numerator, denominator);
		
		TransferFunctionDiscretizer filter = new TransferFunctionDiscretizer(tf, 1000.0);
		
		YoFilteredDouble test_var = new YoFilteredDouble("", registry, filter, true); // 
		
		// Set firstInput to 10.0
		double[] inputs = new double[] {100.0, 1000.0, 1.0, 10.0, 10.0, 500.0, 234.3};
		double[] returns = new double[inputs.length];
		
		for (int i = 0; i < inputs.length; i++) {
			test_var.set(inputs[i]);
			returns[i] = test_var.getRawValue();
		}
		
		// Safe startup means that the first output should be very close to 10.0.
		double delta = 0.000000001;
		assertArrayEquals(inputs, returns);
	}
	
	@Test
	void testCorrectFilteredValue() {
		
		YoRegistry registry = new YoRegistry("test");
		
		ContinuousTransferFunction tf4 = new ContinuousTransferFunction("Complex Multi-Order Filter", 
				1.0,
				new double[] { 196.919515374308, 21033.790696845190, 427573.897431703983, 18317222.932339027524 },
				new double[] { 1.000000000000, 382.156022138851, 60851.343857079330, 3875784.585037478711 });
		TransferFunctionDiscretizer MultiorderComplex_Filter 	= 	new TransferFunctionDiscretizer(tf4, 1000.0);
		
		YoFilteredDouble test_var = new YoFilteredDouble("", registry, MultiorderComplex_Filter, true); // 
		
		double[] inputs 	= new double[] {100.0, 234.3, 23401.89, 2341.0, 8.0, 183.2341, 151.0};
		double[] returns 	= new double[inputs.length];
		
		for (int i = 0; i < inputs.length; i++) {
			test_var.set(inputs[i]);
			returns[i] = test_var.getFilteredValue();
		}
		
		double[] expectedReturns = new double[] {101.19669652724555, 23201.745404389087, 4001508.5930508487, -685721.8208261877, -1007202.8828843683, -793866.5797882355, -646636.3880505173};
		
		double delta = 0.000001;
		assertArrayEquals(expectedReturns, returns, delta);
	}
	

}
