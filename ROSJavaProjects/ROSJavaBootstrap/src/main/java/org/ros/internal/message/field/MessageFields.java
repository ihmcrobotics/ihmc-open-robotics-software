/*
 * Copyright (C) 2012 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.internal.message.field;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.context.MessageContext;

import java.util.Collections;
import java.util.List;
import java.util.Map;

/**
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MessageFields {

  private final Map<String, Field> fields;
  private final Map<String, Field> setters;
  private final Map<String, Field> getters;
  private final List<Field> orderedFields;

  public MessageFields(MessageContext messageContext) {
    fields = Maps.newHashMap();
    setters = Maps.newHashMap();
    getters = Maps.newHashMap();
    orderedFields = Lists.newArrayList();
    for (String name : messageContext.getFieldNames()) {
      Field field = messageContext.getFieldFactory(name).create();
      fields.put(name, field);
      getters.put(messageContext.getFieldGetterName(name), field);
      setters.put(messageContext.getFieldSetterName(name), field);
      orderedFields.add(field);
    }
  }

  public Field getField(String name) {
    return fields.get(name);
  }

  public Field getSetterField(String name) {
    return setters.get(name);
  }

  public Field getGetterField(String name) {
    return getters.get(name);
  }

  public List<Field> getFields() {
    return Collections.unmodifiableList(orderedFields);
  }

  public Object getFieldValue(String name) {
    Field field = fields.get(name);
    if (field != null) {
      return field.getValue();
    }
    throw new RosRuntimeException("Uknown field: " + name);
  }

  public void setFieldValue(String name, Object value) {
    Field field = fields.get(name);
    if (field != null) {
      field.setValue(value);
    } else {
      throw new RosRuntimeException("Uknown field: " + name);
    }
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((fields == null) ? 0 : fields.hashCode());
    result = prime * result + ((orderedFields == null) ? 0 : orderedFields.hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (obj == null)
      return false;
    if (getClass() != obj.getClass())
      return false;
    MessageFields other = (MessageFields) obj;
    if (fields == null) {
      if (other.fields != null)
        return false;
    } else if (!fields.equals(other.fields))
      return false;
    if (orderedFields == null) {
      if (other.orderedFields != null)
        return false;
    } else if (!orderedFields.equals(other.orderedFields))
      return false;
    return true;
  }
}
