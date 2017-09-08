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

package us.ihmc.sensorProcessing.bubo.io.serialization;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import us.ihmc.sensorProcessing.bubo.io.UtilReflections;

/**
 * </p>
 * Provides a data structure description for serialization.  Which class variables are serialized,
 * the order they are serialized, and how to set/get these variable from the class instance is provided.
 * </p>
 *
 * @author Peter Abeles
 */
public class DataDefinition {

	/**
	 * the type of class of which a new instance is to be created.
	 */
	public Class<?> type;

	/**
	 * The name of the data which is serialized.
	 */
	public String typeName;

	/**
	 * Names of all the class variables which will be serialized.  The order is the order in which they are serialized.
	 */
	public String[] variableNames;

	/**
	 * The data types of the serialied class variables
	 */
	public Class<?>[] variableTypes;

	/**
	 * All the class variable getters
	 */
	public Method[] getters;

	/**
	 * All the class variable setters
	 */
	public Method[] setters;

	public DataDefinition() {
	}

	/**
	 * Creates a new description from the provided information.  The parameter types and setters/getters
	 * are loaded using java reflections.
	 */
	public DataDefinition(String typeName, Class<?> type, String... variableNames) {
		this.type = type;
		this.typeName = typeName;
		this.variableNames = variableNames;

		this.variableTypes = new Class<?>[variableNames.length];
		this.setters = new Method[variableNames.length];
		this.getters = new Method[variableNames.length];

		for (int i = 0; i < variableNames.length; i++) {
			this.getters[i] = UtilReflections.findGetter(type, variableNames[i]);
			if (getters[i] == null)
				throw new RuntimeException("No getter found for variable '" + variableNames[i] + "'");
			this.variableTypes[i] = this.getters[i].getReturnType();
			this.setters[i] = UtilReflections.findSetter(type, variableNames[i]);
			if (setters[i] == null)
				throw new RuntimeException("No setter found for variable '" + variableNames[i] + "'");
		}
	}

	/**
	 * Creates a new instance of the class whose value is initialized using the provided data and the classes
	 * own accessors.
	 *
	 * @param data Constructor arguments
	 * @return New instance
	 */
	public <T> T createInstance(Object... data) {
		if (data.length != variableNames.length)
			throw new IllegalArgumentException("Unexpected number of input parameters.");

		try {
			T ret = (T) type.newInstance();

			for (int i = 0; i < variableNames.length; i++) {
				setters[i].invoke(ret, data[i]);
			}

			return ret;

		} catch (InstantiationException e) {
			throw new RuntimeException(e);
		} catch (IllegalAccessException e) {
			throw new RuntimeException(e);
		} catch (InvocationTargetException e) {
			throw new RuntimeException(e);
		}
	}

}
