package us.ihmc.robotics.functionApproximation;
import java.util.ArrayList;

//XXX: consider extend LinearRegression
public class OnlineLinearRegression{
	
	ArrayList<Double> output;
	ArrayList<double[]> input;
	double[] coefficientVector;
	int capacity; 
	
	public OnlineLinearRegression(int capacity, int inputDim) {
		this.capacity=capacity;
		output = new ArrayList<Double>(capacity);
		input = new ArrayList<double[]>(capacity);
		coefficientVector = new double[inputDim];
	}

	public int getNumberOfEntries()
	{
	   return input.size();
	}

	public boolean addEntry(double y, double[] x)
	{
		assert(input.size()==output.size());
		if (input.size()==capacity)
		{
			input.remove(0);
			output.remove(0);
		}
		
		input.add(x);
		output.add(y);
		return true;
	}
	
	public double[] getCoefficient()
	{
		LinearRegression solver = new LinearRegression(input,output);
		solver.solve();
		solver.packCoefficientVector(coefficientVector);
		return coefficientVector;
	}
}
