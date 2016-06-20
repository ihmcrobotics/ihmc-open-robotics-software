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

package us.ihmc.sensorProcessing.bubo.io;

import java.lang.reflect.Method;
import java.util.List;

/**
 * Variable utility functions related to loading or reading classes using reflections.
 *
 * @author Peter Abeles
 */
public class UtilReflections {

	/**
	 * Searches for functions that follow the standard get/set rules and both get/set
	 * exists for the variable
	 *
	 * @param type The class whose accessors are being examined.
	 * @return List of all the valid getters.
	 */
	public static void findAccessors(Class type,
									 List<Method> setters,
									 List<Method> getters,
									 List<String> variableNames) {

		Method methods[] = type.getMethods();

		for (Method m : methods) {
			if (m.getParameterTypes().length > 0)
				continue;
			if (m.getReturnType() == null)
				continue;

			String getterName = m.getName();

			String target = extractGetterName(getterName);

			if (target == null)
				continue;

			// search for a setter that matches the getter

			for (Method n : methods) {
				if (n.getParameterTypes().length != 1)
					continue;

				if (n.getName().compareTo("set" + target) == 0) {

					// make sure the input parameter is the expected type
					Class paramType = n.getParameterTypes()[0];
					if (paramType == m.getReturnType()) {
						if (getters != null)
							getters.add(m);
						if (setters != null)
							setters.add(n);
						if (variableNames != null)
							variableNames.add(target);
						break;
					}
				}
			}
		}

	}

	/**
	 * Checks to see if the provided String is a valid name for a getter.  If so then
	 * the variable name it is referencing is returned, otherwise null is returned.
	 *
	 * @param functionName Original name of the function.
	 * @return If it is a getter the variable's name is returned, otherwise null.
	 */
	public static String extractGetterName(String functionName) {
		String target = null;

		// see if the current method is a getter
		if (functionName.length() > 3) {
			if (functionName.startsWith("get")) {
				target = functionName.substring(3);
			}
		} else if (functionName.length() > 2) {
			if (functionName.startsWith("is")) {
				target = functionName.substring(2);
			}
		}
		return target;
	}

	public static boolean isPrimitiveType(Class<?> type) {
		return (type == byte.class || type == char.class || type == short.class ||
				type == int.class || type == long.class || type == float.class ||
				type == double.class || type == String.class);

	}

	public static boolean isPrimitiveArrayType(Class<?> type) {
		return (type == byte[].class || type == char[].class || type == short[].class ||
				type == int[].class || type == long[].class || type == float[].class ||
				type == double[].class || type == String[].class);
	}

	/**
	 * Sees if it can find a getter for the provided variable name.  First it tries finding a function
	 * that begins with "get" if that fails then it tries "is".
	 *
	 * @param type         The class containing the method.
	 * @param variableName Name of the variable whose getting is being searched for.
	 * @return If a method was found it is returned, otherwise null is returned.
	 */
	public static Method findGetter(Class type, String variableName) {
		Method m;

		try {
			m = type.getMethod(createAccessorName(variableName, true, false));
		} catch (NoSuchMethodException e) {
			try {
				m = type.getMethod(createAccessorName(variableName, true, true));
			} catch (NoSuchMethodException e1) {
				return null;
			}
		}

		return m;
	}

	/**
	 * Searches for a setter method for the provided variable.
	 *
	 * @param type         Class whose methods are being searched.
	 * @param variableName Name of the variable whose setter is being looked for.
	 * @return The setter or null if none were found.
	 */
	public static Method findSetter(Class type, String variableName) {
		Method[] methods = type.getMethods();

		String funcName = createAccessorName(variableName, false, false);

		for (Method m : methods) {
			if (m.getName().compareTo(funcName) == 0)
				return m;
		}

		return null;
	}

	/**
	 * Generates the name of a setter or getter for the provided variable using the standard format.
	 * Setters begin with "set" and getters with "get".  Next is the variable's name, which has its first
	 * letter capitalized.  An exception to this rule is for boolean variables, which use "is" instead of "get" for
	 * getters.
	 *
	 * @param variable  Name of the variable.
	 * @param isGetter  Is it a getter or setter.
	 * @param isBoolean Is the variable of type boolean?
	 * @return The function's name.
	 */
	public static String createAccessorName(String variable, boolean isGetter, boolean isBoolean) {
		if (variable.length() == 0)
			throw new IllegalArgumentException("The variable's name must be at least one character");

		String pre;

		if (isGetter) {
			if (isBoolean)
				pre = "is";
			else
				pre = "get";
		} else {
			pre = "set";
		}

		char firstLetter = variable.charAt(0);

		return pre + Character.toUpperCase(firstLetter) + variable.substring(1, variable.length());
	}
}
