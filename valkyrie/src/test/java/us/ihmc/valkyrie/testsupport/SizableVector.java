package us.ihmc.valkyrie.testsupport;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class SizableVector {
	List<Double> content = null;
	
	public SizableVector(int size) {
		content = new ArrayList<Double>(size);
		for (int i = 0; i < size; i++) {
			content.add(0.0);
		}
	}
	
	public SizableVector(Double[] values)
	{
		this(Arrays.asList(values));
	}
	
	public SizableVector(List<Double> values)
	{
		content = values;
	}
	
	public int size() {
		return content.size();
	}
	
	public SizableVector(String value) {
		fromText(value);
	}
	
	public void fromText(String value) {
		String[] values = value.split("\\s+");
		System.out.printf("Got %d tokens\n", values.length);
		if (content == null) {
		    content = new ArrayList<Double>();
		} else {
			content.clear();
		}
		for (int i = 0; i<values.length; i++) {
			content.add(Double.valueOf(values[i]));
			System.out.printf("Value at %d is %f\n", i, content.get(i));
		}
		
	}
	
	public String toString() {
		List<String> stringValues = content.stream().map(d -> String.valueOf(d)).collect(Collectors.toList());
		return String.join(" ", stringValues);
	}
	
	public void set(int index, Double value) {
		content.set(index, value);
	}
	
    public Double get(int index) {
    	return content.get(index);
	}
    
    public SizableVector copy() {
    	return new SizableVector(content);
    }
	
	public double length() {
		double total = 0;
		for (int i = 0; i < content.size(); i++) {
			total += content.get(i)*content.get(i);
		}
		return Math.sqrt(total);
	}
	
	public void scale(double scaleFactor) {
		content = content.stream().map(d -> d*scaleFactor).collect(Collectors.toList());
	}
	
	public SizableVector addMultiple(SizableVector vector, double multiplier) {
		if (size() != vector.size()) {
			throw new IllegalArgumentException("Cannot add a SizableVector with a different size");
		}
		SizableVector newVector = new SizableVector(content);
		for (int i = 0; i < content.size(); i++) {
			newVector.set(i, newVector.get(i) + multiplier*vector.get(i));
		}
		return newVector;		
	}
	
	public SizableVector add(SizableVector s) {
		return addMultiple(s, 1);
	}
	
	public SizableVector subtract(SizableVector s) {
		return addMultiple(s, -1);
	}
	
	public void scaleIndex(int index, double scaleFactor) {
		content.set(index, content.get(index)*scaleFactor);
	}

}
