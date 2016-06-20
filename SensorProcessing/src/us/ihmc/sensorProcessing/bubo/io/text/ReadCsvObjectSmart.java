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
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.List;

import us.ihmc.sensorProcessing.bubo.io.UtilReflections;
import us.ihmc.sensorProcessing.bubo.io.serialization.DataDefinition;
import us.ihmc.sensorProcessing.bubo.io.serialization.SerializationDefinitionManager;

/**
 * <p>
 * Reads an arbitrary object from each line in a CSV file.  The object can have other objects for parameters and
 * they will still be correctly parsed.  Each object must be known to the provided {@link SerializationDefinitionManager}
 * or else an exception will be thrown.  Primitive arrays can also be read in but they must be predeclared or else the length
 * will not be known.
 * </p>
 * <p/>
 * <p>
 * ** IMPORTANT ** Make sure that all the data types being read in are known to the passed in SerializationDefinitionManager and
 * that the paramNames for each class are specified in the same order as they have been written to the CSV file.
 * </p>
 *
 * @author Peter Abeles
 */
@SuppressWarnings({"unchecked"})
public class ReadCsvObjectSmart<T> extends ReadCsv {

	// the type of object which is being parsed in this file
	private DataDefinition objectType;

	// which word that was read in is currently being processed
	private int indexWords;

	// if not everything is processed on the line should it ignore it?
	private boolean ignoreUnparsedData = false;

	// contains a description of all the data types being read in
	private SerializationDefinitionManager definitions;

	/**
	 * Creates a reader using a custom SerializationDefinitionManager.
	 *
	 * @param in          Stream containing input CSV data.
	 * @param definitions Contains the definitions of all the data types being read.  Makes sure all data types are known and
	 *                    that the paramName's are in the correct order for this file type.
	 * @param typeName    Name of the type being read in as known to 'definitions'.
	 */
	public ReadCsvObjectSmart(InputStream in, SerializationDefinitionManager definitions, String typeName) {
		super(in);

		this.definitions = definitions;
		this.objectType = definitions.lookup(typeName);
		if (objectType == null) {
			throw new IllegalArgumentException("Please add " + typeName + " to the passed in SerializationDefinitionManager.");
		}
	}

	/**
	 * Reads the next object from the InputStream.  If there are no more objects then null is returned.
	 *
	 * @param o If not null then the parsed object will be written to this object, otherwise a new object will be
	 *          created using a no argument constructor.
	 * @return The object which has been read in. Null if there are no more objects to read.
	 * @throws IOException If an error occurs while reading the input stream.
	 */
	public T nextObject(T o) throws IOException {
		List<String> words = extractWords();
		indexWords = 0;

		if (words == null)
			return null;

		// declare a new object if needed
		if (o == null) {
			try {
				o = (T) objectType.type.newInstance();
			} catch (InstantiationException e) {
				throw new RuntimeException(e);
			} catch (IllegalAccessException e) {
				throw new RuntimeException(e);
			}
		}

		parse(o, objectType, words);

		if (!ignoreUnparsedData && indexWords != words.size())
			throw new IOException("Not enough words have been parsed. Parsed: " + indexWords + " Found: " + words.size());

		return o;
	}

	/**
	 * Extracts the value of 'o' from the provided list of words.
	 *
	 * @param o     An instance of the object being parsed.
	 * @param words List of words that contains the value of 'o'
	 * @throws IOException
	 */
	private void parse(T o, DataDefinition def, List<String> words) throws IOException {

		for (int i = 0; i < def.variableNames.length; i++) {
			String variableName = def.variableNames[i];
			Method getter = def.getters[i];
			Method setter = def.setters[i];

			if (indexWords >= words.size())
				throw new IOException("Not enough words on line.  Line number = " + getLineNumber());

			if (getter == null)
				throw new RuntimeException("A getter could not be found for " + variableName + " in " + o.getClass().getSimpleName());
			else if (setter == null)
				throw new RuntimeException("A setter could not be found for " + variableName + " in " + o.getClass().getSimpleName());

			Class<?> varType = getter.getReturnType();

			try {
				if (UtilReflections.isPrimitiveType(varType)) {
					ReadCsvObject.parseSingleWord(o, varType, words.get(indexWords++), setter);
				} else if (UtilReflections.isPrimitiveArrayType(varType)) {
					indexWords += ReadCsvObject.parseArray(o, varType, getter, words, indexWords);
				} else {
					// see if an instance has already been declared
					T varObj = (T) getter.invoke(o);

					// it has not been declared already so create a new instance
					if (varObj == null)
						varObj = (T) varType.newInstance();

					DataDefinition childDef = definitions.lookup(varType.getSimpleName());
					if (childDef == null)
						throw new RuntimeException("No definition for type " + varType.getSimpleName());

					parse(varObj, childDef, words);
					setter.invoke(o, varObj);
				}
			} catch (InstantiationException e) {
				throw new RuntimeException(e);
			} catch (IllegalAccessException e) {
				throw new RuntimeException(e);
			} catch (InvocationTargetException e) {
				throw new RuntimeException(e);
			}
		}
	}

	/**
	 * If it returns true then an exception will not be thrown if there is additional words that need to be processed at the end.
	 *
	 * @return If unparsed data is ignored.
	 */
	public boolean isIgnoreUnparsedData() {
		return ignoreUnparsedData;
	}

	/**
	 * @param ignoreUnparsedData If unparsed data will throw an exception or not.
	 */
	public void setIgnoreUnparsedData(boolean ignoreUnparsedData) {
		this.ignoreUnparsedData = ignoreUnparsedData;
	}
}
