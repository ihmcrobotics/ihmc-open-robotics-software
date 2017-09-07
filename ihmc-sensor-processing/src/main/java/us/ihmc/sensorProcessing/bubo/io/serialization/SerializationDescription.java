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

/**
 * <p>
 * Description how to a serialized data structure.  Classes which implement this can be automatically loaded with
 * out being explicitly added to the {@link SerializationDefinitionManager} by name.  All variables must have
 * setters and getters and a no argument constructor.  A list of variable names is provided by {@link #getVariables()}
 * since many serialization processes don't specify which variable is next so the order and type are needed.
 * </p>
 *
 * @author Peter Abeles
 */
public interface SerializationDescription {

	/**
	 * Names of variables which are to be serialized.  The order is important and specifies the order in which
	 * it is serialized.
	 *
	 * @return Ordered list of serialized class variables.
	 */
	public String[] getVariables();
}
