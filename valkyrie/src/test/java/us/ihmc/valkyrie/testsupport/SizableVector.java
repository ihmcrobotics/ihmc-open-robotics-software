package us.ihmc.valkyrie.testsupport;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Class to help with manipulating vectors of various size to/from XML text elements 
 * @author Mark Paterson
 *
 */
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
	
	public SizableVector(String value) {
		fromText(value);
	}
	
	/**
	 * Replace vector content from a space-delimited string
	 * @param value
	 */
	public void fromText(String value) {
		String[] values = value.split("\\s+");
		if (content == null) {
		    content = new ArrayList<Double>();
		} else {
			content.clear();
		}
		for (int i = 0; i<values.length; i++) {
			content.add(Double.valueOf(values[i]));
		}
		
	}
	
	/**
	 * Get number of components in the vector
	 * @return number of components
	 */
	public int size() {
		return content.size();
	}
	
	/**
	 * Return string representation of the vector suitable for use as the text content of an XML element
	 */
	public String toString() {
		List<String> stringValues = content.stream().map(d -> String.valueOf(d)).collect(Collectors.toList());
		return String.join(" ", stringValues);
	}
	
	/**
	 * Set the value of a vector component
	 * @param index -- which component to set
	 * @param value -- value to set
	 */
	public void set(int index, Double value) {
		content.set(index, value);
	}
	
	/**
	 * Get the value of a vector component
	 * @param index -- which component to get
	 * @return value of the component
	 */
    public Double get(int index) {
    	return content.get(index);
	}
    
    /**
     * Create a copy of the current vector
     * @return
     */
    public SizableVector copy() {
    	return new SizableVector(content);
    }
	
    /**
     * Compute the vector magnitude
     * @return magnitude
     */
	public double magnitude() {
		double total = 0;
		for (int i = 0; i < content.size(); i++) {
			total += content.get(i)*content.get(i);
		}
		return Math.sqrt(total);
	}
	
	/**
	 * In-place scaling of the vector
	 * @param scaleFactor
	 */
	public void scale(double scaleFactor) {
		content = content.stream().map(d -> d*scaleFactor).collect(Collectors.toList());
	}
	
	/**
	 * Add a scalar multiple of a vector, returning the result without modifying the current instance.
	 * Vectors must have the same size.
	 * @param vector
	 * @param multiplier
	 * @return result of this + (multiplier * vector)
	 */
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
	
	/**
	 * Convenience function to add a vector and return the result
	 * @param s -- vector to add
	 * @return result of the addition
	 */
	public SizableVector add(SizableVector s) {
		return addMultiple(s, 1);
	}
	
	/**
	 * Convenience function to subtract a vector and return the result
	 * @param s -- vector to substract
	 * @return result of the subtraction
	 */
	public SizableVector subtract(SizableVector s) {
		return addMultiple(s, -1);
	}
	
	/**
	 * In-place scaling of the vector component specified by index
	 * @param index -- vector offset
	 * @param scaleFactor
	 */
	public void scaleIndex(int index, double scaleFactor) {
		content.set(index, content.get(index)*scaleFactor);
	}

}
