//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.ros.node.parameter;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import org.ros.namespace.GraphName;

public interface ParameterTree {
//   boolean getBoolean(GraphName var1);

   boolean getBoolean(String var1);

   boolean getBoolean(GraphName var1, boolean var2);

   boolean getBoolean(String var1, boolean var2);

   int getInteger(GraphName var1);

   int getInteger(String var1);

   int getInteger(GraphName var1, int var2);

   int getInteger(String var1, int var2);

   double getDouble(GraphName var1);

   double getDouble(String var1);

   double getDouble(GraphName var1, double var2);

   double getDouble(String var1, double var2);

   String getString(GraphName var1);

   String getString(String var1);

   String getString(GraphName var1, String var2);

   String getString(String var1, String var2);

   List<?> getList(GraphName var1);

   List<?> getList(String var1);

   List<?> getList(GraphName var1, List<?> var2);

   List<?> getList(String var1, List<?> var2);

   Map<?, ?> getMap(GraphName var1);

   Map<?, ?> getMap(String var1);

   Map<?, ?> getMap(GraphName var1, Map<?, ?> var2);

   Map<?, ?> getMap(String var1, Map<?, ?> var2);

   void set(GraphName var1, boolean var2);

   void set(String var1, boolean var2);

   void set(GraphName var1, int var2);

   void set(String var1, int var2);

   void set(GraphName var1, double var2);

   void set(String var1, double var2);

   void set(GraphName var1, String var2);

   void set(String var1, String var2);

   void set(GraphName var1, List<?> var2);

   void set(String var1, List<?> var2);

   void set(GraphName var1, Map<?, ?> var2);

   void set(String var1, Map<?, ?> var2);

   boolean has(GraphName var1);

   boolean has(String var1);

   void delete(GraphName var1);

   void delete(String var1);

   GraphName search(GraphName var1);

   GraphName search(String var1);

   Collection<GraphName> getNames();

   void addParameterListener(GraphName var1, ParameterListener var2);

   void addParameterListener(String var1, ParameterListener var2);
}
