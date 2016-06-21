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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * <p>
 * Loads and saves descriptions of data types for serialization purposes.
 * </p>
 * <p/>
 * <p>
 * There are several different ways to load the class description.  A description can be encoded as a java class that
 * implements {@link SerializationDescription} and is automatically loaded when that name is requested.  A class can be specified
 * by its name, class, and variable list.
 * </p>
 *
 * @author Peter Abeles
 */
@SuppressWarnings({"unchecked"})
public class SerializationDefinitionManager {

   public static List<String> DEFAULT_DEFINITION_PATH;

   static {
           DEFAULT_DEFINITION_PATH = new ArrayList<String>();

           DEFAULT_DEFINITION_PATH.add("bubo.io.data");
   }

   
	// description of different data types which are looked up by their typeName
	private Map<String, DataDefinition> descs = new HashMap<String, DataDefinition>();
	// all the paths that a class description can be searched in
	private List<String> paths = new ArrayList<String>();

	public SerializationDefinitionManager() {
		paths.addAll(DEFAULT_DEFINITION_PATH);
	}

	/**
	 * Clears search class paths.
	 */
	public void clearPath() {
		paths.clear();
	}

	/**
	 * Adds a class path that classes can be loaded from which implement {@link SerializationDescription}.
	 *
	 * @param path A new search path.
	 */
	public void addPath(String path) {
		paths.add(path);
	}

	/**
	 * Loads a description from a class that implements {@link SerializationDescription} and is in the path.
	 * <p/>
	 * If a definition already exists with same type name it will be overloaded with the new one just loaded.
	 *
	 * @param typeName The "simple name" of the class that is to be loaded.
	 * @return Definition of the class or null if it was not found in the path.
	 */
	@SuppressWarnings({"EmptyCatchBlock"})
	public DataDefinition loadDefinition(String typeName) {
		for (String p : paths) {
			try {
				Class<SerializationDescription> c = (Class<SerializationDescription>) Class.forName(p + "." + typeName);

				SerializationDescription t = c.newInstance();

				String[] variableNames = t.getVariables();

				return loadDefinition(typeName, c, variableNames);

			} catch (ClassNotFoundException e) {
			} catch (InstantiationException e) {
				throw new RuntimeException(e);
			} catch (IllegalAccessException e) {
				throw new RuntimeException(e);
			}
		}

		return null;
	}

	/**
	 * Loads a definition of any class.  The type name is assumed to be string provided by Class.getSimpleName().
	 * <p/>
	 * If a definition already exists with same type name it will be overloaded with the new one just loaded.
	 *
	 * @param type          Type of class which is to be serialized.
	 * @param variableNames List of class parameters that are being serialized.
	 * @return Definition of the class.
	 */
	public DataDefinition loadDefinition(Class<?> type, String... variableNames) {
		return loadDefinition(type.getSimpleName(), type, variableNames);
	}

	/**
	 * Loads a definition of any class.
	 * <p/>
	 * If a definition already exists with same type name it will be overloaded with the new one just loaded.
	 *
	 * @param typeName      Definition's typeName
	 * @param type          Type of class which is to be serialized.
	 * @param variableNames List of class parameters that are being serialized.
	 * @return Definition of the class.
	 */
	public DataDefinition loadDefinition(String typeName, Class<?> type, String... variableNames) {
		DataDefinition ret = new DataDefinition(typeName, type, variableNames);

		addDefinition(ret);
		return ret;
	}

	/**
	 * Adds a manually constructed DataDefinition.  This function should be rarely used.
	 *
	 * @param definition Definition that is being added to the manager.
	 */
	public void addDefinition(DataDefinition definition) {
		descs.put(definition.typeName, definition);
	}

	/**
	 * Looks up a previously loaded definition.  If none is found then null is returned.
	 */
	public DataDefinition lookup(String name) {
		return descs.get(name);
	}
}
