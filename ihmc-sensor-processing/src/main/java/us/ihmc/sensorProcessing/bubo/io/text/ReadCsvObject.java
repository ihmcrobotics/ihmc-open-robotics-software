/*
 * Copyright (c) 2013-2014, Peter Abeles. All Rights Reserved.
 *
 * This file is part of Project BUBO.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package us.ihmc.sensorProcessing.bubo.io.text;

import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.sensorProcessing.bubo.io.UtilReflections;

/**
 * <p>
 * Reads a series of classes from a {@link ReadCsv CSV} file.  All classes are encoded on a single line.  Which variables
 * are read is specified in the constructor and each variable must have a setter that follows the naming scheme defined
 * in {@link UtilReflections#createAccessorName(String, boolean, boolean)}.  In addition, if the variable is an array, it
 * <p/>
 * UPDATE THIS TO INCLUDE ARRAYS
 * </p>
 * <p/>
 * <p>
 * See {@link ReadCsv} for a more detailed description of the CSV file definition and about comments within the file.
 * All variables must be a primitive type of byte, char, short, int, long, float, or double.
 * </p>
 *
 * @author Peter Abeles
 */
public class ReadCsvObject<T> extends ReadCsv {

	// the type of object being read in
	private Class<?> objectType;

	// used by primitive variables to pass on the parsed value
	private Method[] setters;

	// used by arrays to get a predeclared array
	private Method[] getters;

	// the type of variable
	private Class<?>[] variableTypes;

	// is the variable an array or not
	private boolean[] arrayVariable;

	/**
	 * @param in         Input stream encoded using CSV
	 * @param objectType The type of objects that will be parsed
	 * @param variables  Variable names that will be parsed in their respective order.
	 */
	public ReadCsvObject(InputStream in, Class<T> objectType, String... variables) {
		super(in);
		this.objectType = objectType;

		setters = new Method[variables.length];
		getters = new Method[variables.length];
		variableTypes = new Class<?>[variables.length];
		arrayVariable = new boolean[variables.length];

		for (int i = 0; i < variables.length; i++) {
			String v = variables[i];
			setters[i] = UtilReflections.findSetter(objectType, v);

			// save the method and extract its input type
			Class<?>[] varType = setters[i].getParameterTypes();

			if (varType.length != 1)
				throw new IllegalArgumentException("Unexpected number of parameters in setter " + setters[i].getName() + " found " + varType.length);

			variableTypes[i] = varType[0];
			arrayVariable[i] = checkValidType(variableTypes[i], setters[i].getName());

			if (arrayVariable[i]) {
				getters[i] = UtilReflections.findGetter(objectType, v);
			}
		}
	}

	/**
	 * Checks to see if the provided type is one of the types that can be parsed  .
	 * <p/>
	 * Returns true if class is an array type or false otherwise.
	 */
	protected static boolean checkValidType(Class<?> type, String setter) {
		if (UtilReflections.isPrimitiveType(type)) {
			return false;
		} else if (UtilReflections.isPrimitiveArrayType(type)) {
			return true;
		} else {
			throw new IllegalArgumentException("The function '" + setter + "' does not have a valid type");
		}
	}

	/**
	 * Parses a single word into a primative type.  This is then passed onto the object using a setter.
	 */
	protected static void parseSingleWord(Object o, Class<?> varType, String word, Method setter)
			throws InvocationTargetException, IllegalAccessException {
		if (varType == byte.class) {
			setter.invoke(o, Byte.parseByte(word));
		} else if (varType == char.class) {
			if (word.length() != 1)
				throw new RuntimeException("Expected a single character, instead found " + word);
			setter.invoke(o, word.charAt(0));
		} else if (varType == short.class) {
			setter.invoke(o, Short.parseShort(word));
		} else if (varType == int.class) {
			setter.invoke(o, Integer.parseInt(word));
		} else if (varType == long.class) {
			setter.invoke(o, Long.parseLong(word));
		} else if (varType == float.class) {
			setter.invoke(o, Float.parseFloat(word));
		} else if (varType == double.class) {
			setter.invoke(o, Double.parseDouble(word));
		} else if (varType == String.class) {
			setter.invoke(o, word);
		} else {
			throw new RuntimeException("Unknown type");
		}
	}

	/**
	 * Uses a getter to access a predeclared array.  It then proceeds to read in as many words as the array is long.
	 *
	 * @param o          Instance of the object with predeclared arrays returned by the getter.
	 * @param varType    Type of array.
	 * @param getter     Returns a predeclared array of the appropriate length
	 * @param words      List of words which are to be parsed.
	 * @param wordsIndex Current index in words list.
	 * @return Number of words read in by the array.
	 */
	protected static int parseArray(Object o, Class<?> varType, Method getter, List<String> words, int wordsIndex)
			throws IOException, InvocationTargetException, IllegalAccessException {

		Object arrayData = getter.invoke(o);
		if (arrayData == null) {
			throw new RuntimeException("In order to parse an array it must be predeclared inside of the object.  " +
					"Otherwise its length is not known. getter = " + getter.getName());
		}

		int arrayLength = Array.getLength(arrayData);
		if (arrayLength > words.size() - wordsIndex)
			throw new IOException("Too few words on this line to read in array");

		if (varType == byte[].class) {
			byte[] a = (byte[]) arrayData;
			for (int i = 0; i < a.length; i++) {
				a[i] = Byte.parseByte(words.get(wordsIndex + i));
			}
		} else if (varType == char[].class) {
			char[] a = (char[]) arrayData;
			for (int i = 0; i < a.length; i++) {
				String word = words.get(wordsIndex + i);
				if (word.length() != 1)
					throw new RuntimeException("Only expecting a single character found this instead: " + word);
				a[i] = word.charAt(0);
			}
		} else if (varType == short[].class) {
			short[] a = (short[]) arrayData;
			for (int i = 0; i < a.length; i++) {
				a[i] = Short.parseShort(words.get(wordsIndex + i));
			}
		} else if (varType == int[].class) {
			int[] a = (int[]) arrayData;
			for (int i = 0; i < a.length; i++) {
				a[i] = Integer.parseInt(words.get(wordsIndex + i));
			}
		} else if (varType == long[].class) {
			long[] a = (long[]) arrayData;
			for (int i = 0; i < a.length; i++) {
				a[i] = Long.parseLong(words.get(wordsIndex + i));
			}
		} else if (varType == float[].class) {
			float[] a = (float[]) arrayData;
			for (int i = 0; i < a.length; i++) {
				a[i] = Float.parseFloat(words.get(wordsIndex + i));
			}
		} else if (varType == double[].class) {
			double[] a = (double[]) arrayData;
			for (int i = 0; i < a.length; i++) {
				a[i] = Double.parseDouble(words.get(wordsIndex + i));
			}
		} else if (varType == String[].class) {
			String[] a = (String[]) arrayData;
			for (int i = 0; i < a.length; i++) {
				a[i] = words.get(wordsIndex + i);
			}
		} else {
			throw new RuntimeException("Unknown array type");
		}

		return arrayLength;
	}

	/**
	 * Reads all objects until the end of the file
	 * @return List of objects read
	 * @throws IOException
	 */
	public List<T> readAll() throws IOException {
		List<T> ret = new ArrayList<T>();

		while( true ) {
			T o = nextObject(null);
			if( o == null )
				break;
			ret.add(o);
		}
		return ret;
	}

	/**
	 * Reads the next object from the InputStream.  If there are no more objects then null is returned.
	 *
	 * @param o If not null then the parsed object will be written to this object, otherwise a new object will be
	 *          created using a no argument constructor.
	 * @return The object which has been read in.
	 * @throws IOException If an error occurs while reading the input stream.
	 */
	@SuppressWarnings({"unchecked"})
	public T nextObject(T o) throws IOException {

		List<String> words = extractWords();

		if (words == null)
			return null;

		// declare a new object if needed
		if (o == null) {
			try {
				o = (T) objectType.newInstance();
			} catch (InstantiationException e) {
				throw new RuntimeException(e);
			} catch (IllegalAccessException e) {
				throw new RuntimeException(e);
			}
		}

		if (words.size() < setters.length)
			throw new IOException("Too few words on this line.  Line = " + getLineNumber());

		// parse the words and call the setter
		int wordIndex = 0;
		for (int i = 0; i < setters.length; i++) {
			Class<?> varType = variableTypes[i];
			try {
				if (arrayVariable[i]) {
					wordIndex += parseArray(o, varType, getters[i], words, wordIndex);
				} else {
					String word = words.get(wordIndex++);
					parseSingleWord(o, varType, word, setters[i]);
				}
			} catch (IllegalAccessException e) {
				throw new RuntimeException(e);
			} catch (InvocationTargetException e) {
				throw new RuntimeException(e);
			}
		}

		if (words.size() != wordIndex)
			throw new IOException("Unexpected number of words at line " + (getLineNumber() - 1));

		return (T) o;
	}

}
