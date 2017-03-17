package us.ihmc.robotics.dataStructures.registry;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.RewoundListener;
import us.ihmc.robotics.dataStructures.listener.YoVariableRegistryChangedListener;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;

public class YoVariableRegistry implements YoVariableHolder
{

   // User defined control variables will be placed in this ArrayList when they are registered:
   private ArrayList<YoVariable<?>> controlVars = new ArrayList<YoVariable<?>>();
   private LinkedHashMap<String, YoVariable<?>> controlVarsHashMap = new LinkedHashMap<String, YoVariable<?>>(); // From name to the variable with that name.
   private final String name;
   private NameSpace nameSpace;
   private ArrayList<YoVariableRegistry> children = new ArrayList<YoVariableRegistry>();
   private YoVariableRegistry parent;

   private ArrayList<RewoundListener> simulationRewoundListeners = null;
   private ArrayList<YoVariableRegistryChangedListener> yoVariableRegistryChangedListeners = null;

   private boolean disallowSending = false;
   private boolean isLogged = false;
   private boolean isSent = false;

   private static final Pattern illegalCharacters = Pattern.compile("[ .*?@#$%/^&()<>,:{}'\"\\\\]");

   protected static void checkForIllegalCharacters(String name)
   {
      // String.matches() only matches the whole string ( as if you put ^$ around it ). Use .find() of the Matcher class instead!

      if (illegalCharacters.matcher(name).find())
      {
         throw new RuntimeException(name
               + " is an invalid name for a YoVariableRegistry. A YoVariableRegistry cannot have crazy characters in them, otherwise namespaces will not work.");
      }
   }

   public YoVariableRegistry(String name)
   {
      this(name, false, false);
   }

   public YoVariableRegistry(String name, boolean isLogged, boolean isSent)
   {
      checkForIllegalCharacters(name);

      this.name = name;

      if ((name == null) || (name.equals("")))
      {
         nameSpace = null;
      }
      else
      {
         nameSpace = new NameSpace(name);
      }

      this.isLogged = isLogged;
      this.isSent = isSent;
   }

   public String getName()
   {
      return name;
   }

   public NameSpace getNameSpace()
   {
      return nameSpace;
   }

   public void attachYoVariableRegistryChangedListener(YoVariableRegistryChangedListener listener)
   {
      if (yoVariableRegistryChangedListeners == null)
      {
         yoVariableRegistryChangedListeners = new ArrayList<YoVariableRegistryChangedListener>();
      }

      yoVariableRegistryChangedListeners.add(listener);

      this.verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners();
   }

   public void registerSimulationRewoundListener(RewoundListener simulationRewoundListener)
   {
      if (simulationRewoundListeners == null)
      {
         simulationRewoundListeners = new ArrayList<RewoundListener>();
      }

      simulationRewoundListeners.add(simulationRewoundListener);
   }

   public ArrayList<RewoundListener> getAllSimulationRewoundListeners()
   {
      ArrayList<RewoundListener> ret = new ArrayList<RewoundListener>();

      getAllSimulationRewoundListenersRecursively(ret);

      return ret;
   }

   private void getAllSimulationRewoundListenersRecursively(ArrayList<RewoundListener> simulationRewoundListenersToPack)
   {
      // Add ours:
      if (simulationRewoundListeners != null)
         simulationRewoundListenersToPack.addAll(simulationRewoundListeners);

      // Add children's recursively:
      for (YoVariableRegistry registry : children)
      {
         registry.getAllSimulationRewoundListenersRecursively(simulationRewoundListenersToPack);
      }
   }

   public void registerVariable(YoVariable<?> variable)
   {
      String variableName = variable.getName();
      variableName = variableName.toLowerCase(); // Make everything case insensitive! Cannot have two YoVariables with same names except case.

      if (controlVarsHashMap.containsKey(variableName))
      {
         System.err.println("Error:  " + variable.getName() + " has already been registered in this registry! YoVariableRegistry nameSpace = " + nameSpace);
         // Sometimes RuntimeExceptions are not displayed, but the error out is still visible.
         throw new RuntimeException("Error:  " + variable.getName() + " has already been registered in this registry! YoVariableRegistry nameSpace = "
               + nameSpace);
      }

      controlVarsHashMap.put(variableName, variable);
      controlVars.add(variable);

      notifyListenersYoVariableWasRegistered(variable);
   }

   public ArrayList<YoVariable<?>> getAllVariablesInThisListOnly()
   {
      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();
      ret.addAll(controlVars);

      return ret;
   }

   public ArrayList<YoVariable<?>> getAllVariablesIncludingDescendants()
   {
      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();
      getAllVariablesIncludingDescendantsRecursively(ret);

      return ret;
   }

   private void getAllVariablesIncludingDescendantsRecursively(ArrayList<YoVariable<?>> variables)
   {
      // Add ours:
      variables.addAll(controlVars);

      // Add children's recursively:
      for (YoVariableRegistry registry : children)
      {
         registry.getAllVariablesIncludingDescendantsRecursively(variables);
      }
   }

   public YoVariable<?>[] getAllVariablesArray()
   {
      ArrayList<YoVariable<?>> variables = getAllVariablesIncludingDescendants();

      YoVariable<?>[] ret = new YoVariable[variables.size()];
      variables.toArray(ret);

      return ret;
   }

   public YoVariableList createVarList()
   {
      YoVariableList ret = new YoVariableList(this.nameSpace.getName());

      ret.addVariables(controlVars);

      return ret;
   }

   /*
    * Returns the first discovered instance of a variable matching the given
    * name. It will first check this registry, then it's children in the order
    * in which they were added. Returns null if no variable is found.
    */
   public YoVariable<?> getVariable(String nameSpace, String name)
   {
      if (name.contains("."))
         throw new RuntimeException(name + " contains a dot. It must not when calling getVariable(String nameSpace, String name)");

      return getVariable(nameSpace + "." + name);
   }

   /*
    * Returns the first discovered instance of a variable matching the given
    * name. It will first check this registry, then it's children in the order
    * in which they were added. Returns null if no variable is found.
    */
   public YoVariable<?> getVariable(String name)
   {
      String matchedName = matchNameSpace(name);

      if (matchedName != null)
      {
         matchedName = matchedName.toLowerCase();

         YoVariable<?> variable = controlVarsHashMap.get(matchedName);
         if (variable != null)
            return variable;
      }

      for (YoVariableRegistry child : children)
      {
         YoVariable<?> variable = child.getVariable(name);
         if (variable != null)
            return variable;
      }

      return null;
   }

   /*
    * Returns all of the variables matching the given name, searching in this
    * YoVariableRegistry and all of its children.
    */
   public ArrayList<YoVariable<?>> getVariables(String nameSpace, String name)
   {
      if (name.contains("."))
         throw new RuntimeException(name + " contains a dot. It must not when calling hasVariable(String nameSpace, String name)");

      return getVariables(nameSpace + "." + name);
   }

   /*
    * Returns all of the variables matching the given name, searching in this
    * YoVariableRegistry and all of its children.
    */
   public ArrayList<YoVariable<?>> getVariables(String name)
   {
      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();

      getVariables(ret, name);

      return ret;
   }

   /*
    * Adds to listToPack all of the variables matching the given name, searching
    * in this YoVariableRegistry and all of its children.
    */
   public void getVariables(ArrayList<YoVariable<?>> listToPack, String name)
   {
      String matchedName = matchNameSpace(name);

      if (matchedName != null)
      {
         matchedName = matchedName.toLowerCase();

         YoVariable<?> variable = controlVarsHashMap.get(matchedName);
         if (variable != null)
            listToPack.add(variable);
      }

      for (YoVariableRegistry child : children)
      {
         child.getVariables(listToPack, name);
      }
   }

   /*
    * Returns true if this YoVariableRegistry, or any of its children, hold a
    * variable matching this name, and no more than one of them. To match the
    * given name, the variable's nameSpace must end with the given nameSpace and
    * the variables name must be the given name.
    */
   public boolean hasUniqueVariable(String nameSpace, String name)
   {
      if (name.contains("."))
         throw new RuntimeException(name + " contains a dot. It must not when calling hasVariable(String nameSpace, String name)");

      return hasUniqueVariable(nameSpace + "." + name);
   }

   /*
    * Returns true if this YoVariableRegistry, or any of its children, hold a
    * variable of this name, and no more than one of them. To match the given
    * name, the variable's nameSpace must end with the given nameSpace,
    * specified by the last part of the given name, and the variables name must
    * be the end part of the given name. For example
    * hasUniqueVariable(b.c.variableName) will be true if this registry is a,
    * and it has a child b, with a child c, which has a variable named
    * variableName.
    */
   public boolean hasUniqueVariable(String name)
   {
      int numberOfInstances = this.getNumberOfInstancesRecursively(name);
      if (numberOfInstances == 1)
         return true;

      return false;
   }

   private int getNumberOfInstancesRecursively(String name)
   {
      int ret = 0;

      String matchedName = matchNameSpace(name);
      if (matchedName != null)
      {
         matchedName = matchedName.toLowerCase();

         YoVariable<?> variable = controlVarsHashMap.get(matchedName);
         if (variable != null)
            ret = ret + 1;
      }

      for (YoVariableRegistry child : children)
      {
         ret = ret + child.getNumberOfInstancesRecursively(name);
      }

      return ret;
   }

   private String matchNameSpace(String name)
   {
      int dotIndex = name.lastIndexOf(".");

      if (dotIndex != -1)
      {
         String nameSpaceString = name.substring(0, dotIndex);
         if (!this.nameSpace.endsWith(nameSpaceString))
            return null;
         name = name.substring(dotIndex + 1);
      }

      return name;
   }

   public void addChild(YoVariableRegistry child)
   {
      addChild(child, true);
   }

   public void addChild(YoVariableRegistry child, boolean notifyListeners)
   {
      // Prepend the parents nameSpace to the child. NameSpace will figure out if it's valid or not.
      // This then requires that the child only has it's portion of the namespace that the parent does not.

      if (child == null)
         return;

      // Make sure no children with this name already:
      for (YoVariableRegistry childToCheck : children)
      {
         String childToCheckShortName = childToCheck.getNameSpace().getShortName();
         String childShortName = child.getNameSpace().getShortName();
         if (childToCheckShortName.equals(childShortName))
         {
            throw new RuntimeException("Adding a child to a YoVariableRegistry that has the same name as a previous one: "
                  + childToCheck.getNameSpace().getName() + ". Parent name space = " + this.getNameSpace().getName());
         }
      }

      NameSpace parentNameSpace = this.getNameSpace();

      child.prependNameSpace(parentNameSpace);

      //    System.err.println("Child: " + child.getNameSpace().getShortName() + " parent:" + this.getNameSpace().getName());

      child.setParent(this);
      children.add(child);

      if (notifyListeners) notifyListenersYoVariableRegistryWasAdded(child);
   }

   private void prependNameSpace(NameSpace parentNameSpace)
   {
      if (this.nameSpace == null)
         throw new RuntimeException("Cannot prepend a namespace. This namespace is null. Only root can have a null namespace");
      if (parentNameSpace == null)
         return;

      // Fix my name
      this.nameSpace = new NameSpace(parentNameSpace.getName() + "." + this.nameSpace.getName());

      // Fix my children
      for (YoVariableRegistry child : children)
      {
         child.prependNameSpace(parentNameSpace);
      }
   }

   public void recursivelyChangeNamespaces(NameSpaceRenamer nameSpaceRenamer)
   {
      NameSpace nameSpace = this.getNameSpace();
      String nameSpaceString = nameSpace.getName();

      nameSpaceString = nameSpaceRenamer.changeNamespaceString(nameSpaceString);
      this.changeNameSpace(nameSpaceString);

      System.out.println(nameSpaceString);

      ArrayList<YoVariableRegistry> children = this.getChildren();

      for (YoVariableRegistry child : children)
      {
         child.recursivelyChangeNamespaces(nameSpaceRenamer);
      }
   }

   public void changeNameSpace(String newNamespace)
   {
      System.err.println("Warning: Changing namespace from " + this.nameSpace + " to " + newNamespace);
      this.nameSpace = new NameSpace(newNamespace);
   }

   public ArrayList<YoVariableRegistry> getChildren()
   {
      return children;
   }

   public YoVariableRegistry getParent()
   {
      return parent;
   }

   private void setParent(YoVariableRegistry parent)
   {
      if (this.parent != null)
         throw new RuntimeException("Parent was already set!! It was " + this.parent + ". this = " + this);

      this.parent = parent;

      this.verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners();
   }

   public ArrayList<YoVariableList> createVarListsIncludingChildren()
   {
      LinkedHashMap<String, YoVariableList> hashMap = new LinkedHashMap<String, YoVariableList>();
      createVarListsIncludingChildren(hashMap);

      ArrayList<YoVariableList> ret = new ArrayList<YoVariableList>(hashMap.values());
      Collections.sort(ret);

      return ret;
   }

   private void createVarListsIncludingChildren(HashMap<String, YoVariableList> allVarLists)
   {
      // Add mine:
      YoVariableList myVarList = this.createVarList();
      if (allVarLists.containsKey(myVarList.getName()))
      {
         YoVariableList varList = allVarLists.get(myVarList.getName());
         varList.addVariables(myVarList);
      }
      else
      {
         allVarLists.put(myVarList.getName(), myVarList);
      }

      // Add all the children recursively:
      for (YoVariableRegistry child : children)
      {
         child.createVarListsIncludingChildren(allVarLists);
      }
   }

   public ArrayList<YoVariableRegistry> getAllRegistriesIncludingChildren()
   {
      ArrayList<YoVariableRegistry> ret = new ArrayList<YoVariableRegistry>();
      getAllRegistrysIncludingDescendants(ret);

      return ret;
   }

   private void getAllRegistrysIncludingDescendants(ArrayList<YoVariableRegistry> yoVariableRegistriesToPack)
   {
      // Add mine:
      yoVariableRegistriesToPack.add(this);

      // Add all the children recursively:
      for (YoVariableRegistry child : children)
      {
         child.getAllRegistrysIncludingDescendants(yoVariableRegistriesToPack);
      }
   }

   public void clear()
   {
      controlVars.clear();
      controlVarsHashMap.clear();
      children.clear();

      notifyListenersYoVariableRegistryWasCleared(this);
   }

   public String toString()
   {
      //    StringBuffer buf = new StringBuffer();
      //    buf.append("YoVariableRegistry " + nameSpace.getName());
      //
      //    for (AbstractYoVariable var : controlVars)
      //    {
      //       buf.append("\n");
      //       buf.append(var.toString());
      //    }
      //
      //    return buf.toString();
      return nameSpace.getName();
   }

   public YoVariableRegistry getOrCreateAndAddRegistry(NameSpace fullNameSpace)
   {
      if ((this.nameSpace == null) && (fullNameSpace == null))
      {
         return this;
      }

      if ((this.nameSpace != null) && this.nameSpace.equals(fullNameSpace))
      {
         return this;
      }

      if ((this.nameSpace == null) || (fullNameSpace.startsWith(this.nameSpace.getName())))
      {
         for (YoVariableRegistry child : children)
         {
            YoVariableRegistry registry = child.getOrCreateAndAddRegistry(fullNameSpace);
            if (registry != null)
               return registry;
         }

         // If, after going through all the children, none of them match, then
         // create it here and return it.
         NameSpace nameSpaceToContinueWith = fullNameSpace.stripOffFromBeginning(this.nameSpace);
         YoVariableRegistry registry = createChainOfRegistries(nameSpaceToContinueWith);
         this.addChild(registry);

         return getToBottomRegistry(registry);
      }

      return null;
   }

   public YoVariableRegistry getRegistry(NameSpace fullNameSpace)
   {
      if ((this.nameSpace == null) && (fullNameSpace == null))
      {
         return this;
      }

      if ((this.nameSpace != null) && this.nameSpace.equals(fullNameSpace))
      {
         return this;
      }

      if ((this.nameSpace == null) || (fullNameSpace.startsWith(this.nameSpace.getName())))
      {
         for (YoVariableRegistry child : children)
         {
            YoVariableRegistry registry = child.getRegistry(fullNameSpace);
            if (registry != null)
               return registry;
         }

         throw new RuntimeException("Registry not found");
      }

      return null;
   }

   public void setLogging(boolean log)
   {
      this.isLogged = log;
   }

   public void setLoggingIncludingDescendants(boolean log)
   {
      setLogging(log);

      for (YoVariableRegistry registry : children)
      {
         registry.setLoggingIncludingDescendants(log);
      }
   }

   public boolean isLogged()
   {
      return isLogged;
   }

   public void setDisallowSending()
   {
      this.disallowSending = true;
      this.isSent = false;
   }

   public void setSending(boolean send)
   {
      if (disallowSending && send)
      {
         String errorMessage = "Trying to set sending in registry " + this.getName() + " even though disallowSending is true.";
         //         throw new RuntimeException(errorMessage);
         System.err.println(errorMessage);
      }
      else
      {
         this.isSent = send;
      }
   }

   public void setSendingIncludingDescendants(boolean send)
   {
      setSending(send);

      for (YoVariableRegistry registry : children)
      {
         registry.setSendingIncludingDescendants(send);
      }
   }

   public boolean isDisallowSendingSet()
   {
      return disallowSending;
   }

   public boolean isSent()
   {
      return isSent;
   }

   public int getNumberOfYoVariables()
   {
      return controlVars.size();
   }

   public YoVariable<?> getYoVariable(int index)
   {
      return controlVars.get(index);
   }

   private YoVariableRegistry createChainOfRegistries(NameSpace fullNameSpace)
   {
      String rootName = fullNameSpace.getRootName();
      YoVariableRegistry rootRegistry = new YoVariableRegistry(rootName);

      String subName = fullNameSpace.getNameWithRootStripped();
      if (subName == null)
         return rootRegistry;

      NameSpace subNameSpace = new NameSpace(subName);
      YoVariableRegistry subRegistry = createChainOfRegistries(subNameSpace);

      rootRegistry.addChild(subRegistry);

      return rootRegistry;
   }

   private static YoVariableRegistry getToBottomRegistry(YoVariableRegistry root)
   {
      if ((root.children == null) || (root.children.size() == 0))
         return root;
      if (root.children.size() > 1)
         throw new RuntimeException("This should only be called with a new chain!!");

      return getToBottomRegistry(root.children.get(0));
   }

   public void printAllVariablesIncludingDescendants(PrintStream out)
   {
      for (YoVariable<?> var : controlVars)
      {
         out.print(var.getFullNameWithNameSpace() + "\n");
      }

      for (YoVariableRegistry child : children)
      {
         child.printAllVariablesIncludingDescendants(out);
      }
   }

   public ArrayList<YoVariable<?>> getAllVariables()
   {
      return this.getAllVariablesIncludingDescendants();
   }

   public ArrayList<YoVariable<?>> getVariables(NameSpace nameSpace)
   {
      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();

      ArrayList<YoVariable<?>> allVariables = getAllVariables();

      for (YoVariable<?> variable : allVariables)
      {
         if (variable.getYoVariableRegistry().getNameSpace().equals(nameSpace))
         {
            ret.add(variable);
         }
      }

      return ret;
   }

   public synchronized ArrayList<YoVariable<?>> getMatchingVariables(String[] names, String[] regularExpressions)
   {
      ArrayList<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();

      if (names != null)
      {
         for (int i = 0; i < names.length; i++)
         {
            if (names[i] != null)
            {
               String name = names[i];
               YoVariable<?> var = getVariable(name);

               if (var != null)
               {
                  ret.add(var);
               }
            }
         }
      }

      recursivelyGetMatchingVariables(ret, regularExpressions);

      return ret;
   }

   private void recursivelyGetMatchingVariables(ArrayList<YoVariable<?>> ret, String[] regularExpressions)
   {
      if (regularExpressions != null)
      {
         for (int i = 0; i < regularExpressions.length; i++)
         {
            Pattern pattern = Pattern.compile(regularExpressions[i]);

            for (int j = 0; j < controlVars.size(); j++)
            {
               YoVariable<?> var = controlVars.get(j);
               Matcher matcher = pattern.matcher(var.getName());

               if (matcher.matches())
               {
                  ret.add(var);
               }
            }
         }
      }

      for (YoVariableRegistry child : children)
      {
         child.recursivelyGetMatchingVariables(ret, regularExpressions);
      }
   }

   public boolean areEqual(YoVariableRegistry registry)
   {
      if (registry == null)
         return false;

      if (!this.getNameSpace().equals(registry.getNameSpace()))
         return false;

      if (this.isLogged != registry.isLogged)
         return false;
      if (this.isSent != registry.isSent)
         return false;
      if (this.disallowSending != registry.disallowSending)
         return false;

      for (YoVariable<?> variable : controlVars)
      {
         if (getVariableWithSameName(registry.controlVars, variable) == null)
         {
            return false;
         }
      }

      if (this.children.size() != registry.children.size())
         return false;

      for (YoVariableRegistry child : this.children)
      {
         YoVariableRegistry matchingChild = getRegistryWithSameNameSpace(registry.children, child);
         if (!child.areEqual(matchingChild))
         {
            return false;
         }
      }

      return true;
   }

   private static YoVariable<?> getVariableWithSameName(ArrayList<YoVariable<?>> variables, YoVariable<?> variableToMatch)
   {
      for (YoVariable<?> variable : variables)
      {
         if (variable.getFullNameWithNameSpace().equals(variableToMatch.getFullNameWithNameSpace()))
            return variable;
      }

      return null;
   }

   private static YoVariableRegistry getRegistryWithSameNameSpace(ArrayList<YoVariableRegistry> registries, YoVariableRegistry registryToMatch)
   {
      for (YoVariableRegistry registry : registries)
      {
         if (registryToMatch.getNameSpace().equals(registry.getNameSpace()))
         {
            return registry;
         }
      }

      return null;
   }

   private void verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners()
   {
      if ((parent != null) && (yoVariableRegistryChangedListeners != null))
      {
         throw new RuntimeException("Only root YoVariableRegistries should have listeners. This registry does! YoVariableRegistry = " + this);
      }
   }

   private void notifyListenersYoVariableRegistryWasAdded(YoVariableRegistry child)
   {
      // Push it up the chain. Only the root will notify it's listeners. Non-roots shouldn't have listeners.
      verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners();

      if (parent != null)
      {
         parent.notifyListenersYoVariableRegistryWasAdded(child);
      }
      else if (yoVariableRegistryChangedListeners != null)
      {
         for (YoVariableRegistryChangedListener listener : yoVariableRegistryChangedListeners)
         {
            listener.yoVariableRegistryWasAdded(child);
         }
      }
   }

   private void notifyListenersYoVariableRegistryWasCleared(YoVariableRegistry registry)
   {
      // Push it up the chain. Only the root will notify it's listeners. Non-roots shouldn't have listeners.
      verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners();

      if (parent != null)
      {
         parent.notifyListenersYoVariableRegistryWasCleared(registry);
      }

      if (yoVariableRegistryChangedListeners != null)
      {
         for (YoVariableRegistryChangedListener listener : yoVariableRegistryChangedListeners)
         {
            listener.yoVariableRegistryWasCleared(registry);
         }
      }
   }

   private void notifyListenersYoVariableWasRegistered(YoVariable<?> variable)
   {
      // Push it up the chain. Only the root will notify it's listeners. Non-roots shouldn't have listeners.
      verifyDoNotHaveBothParentAndYoVariableRegistryChangedListeners();

      if (parent != null)
      {
         parent.notifyListenersYoVariableWasRegistered(variable);
      }

      else if (yoVariableRegistryChangedListeners != null)
      {
         for (YoVariableRegistryChangedListener listener : yoVariableRegistryChangedListeners)
         {
            listener.yoVariableWasRegistered(this, variable);
         }
      }
   }

   public void closeAndDispose()
   {
      if (controlVars != null)
      {
         controlVars.clear();
         controlVars = null;
      }

      if (controlVarsHashMap != null)
      {
         controlVarsHashMap.clear();
         controlVarsHashMap = null;
      }

      if (children != null)
      {
         for (YoVariableRegistry child : children)
         {
            child.closeAndDispose();
         }
         children = null;
      }

      parent = null;

      if (simulationRewoundListeners != null)
      {
         simulationRewoundListeners.clear();
         simulationRewoundListeners = null;
      }

      if (yoVariableRegistryChangedListeners != null)
      {
         yoVariableRegistryChangedListeners.clear();
         yoVariableRegistryChangedListeners = null;
      }
   }

   public static void printSizeRecursively(int minVariablesToPrint, int minChildrenToPrint, YoVariableRegistry root)
   {
      ArrayList<YoVariableRegistry> registriesOfInterest = new ArrayList<>();
      int totalVariables = collectRegistries(minVariablesToPrint, minChildrenToPrint, root, registriesOfInterest);
      Collections.sort(registriesOfInterest, new Comparator<YoVariableRegistry>()
      {
         @Override
         public int compare(YoVariableRegistry o1, YoVariableRegistry o2)
         {
            if (o1.getNumberOfYoVariables() == o2.getNumberOfYoVariables())
               return 0;
            return o1.getNumberOfYoVariables() > o2.getNumberOfYoVariables() ? -1 : 1;
         }
      });

      System.out.println("");
      PrintTools.info("String recursively at " + root.getName() + " registry.");
      System.out.println("Total Number of YoVariables: " + totalVariables);
      System.out.println("Listing Variables with more then " + minVariablesToPrint + " variables or more then " + minChildrenToPrint + " children.");
      System.out.println("Sorting by number of children.\n");

      for (int registryIdx = 0; registryIdx < registriesOfInterest.size(); registryIdx++)
         YoVariableRegistry.printInfo(registriesOfInterest.get(registryIdx));

      System.out.println("");
   }

   private static int collectRegistries(int minVariablesToPrint, int minChildrenToPrint, YoVariableRegistry registry,
         ArrayList<YoVariableRegistry> registriesOfInterest)
   {
      int variables = registry.getNumberOfYoVariables();
      int children = registry.getChildren().size();

      if (variables >= minVariablesToPrint || children >= minChildrenToPrint)
         registriesOfInterest.add(registry);

      int totalNumberOfVariables = variables;
      for (int childIdx = 0; childIdx < children; childIdx++)
      {
         YoVariableRegistry childRegistry = registry.getChildren().get(childIdx);
         totalNumberOfVariables += YoVariableRegistry.collectRegistries(minVariablesToPrint, minChildrenToPrint, childRegistry,
               registriesOfInterest);
      }

      return totalNumberOfVariables;
   }

   private static void printInfo(YoVariableRegistry registry)
   {
      int variables = registry.getNumberOfYoVariables();
      int children = registry.getChildren().size();

      int maxPropertyLength = 17;
      String variableString = trimStringToLength("Variables: " + variables, maxPropertyLength, "...");
      String childrenString = trimStringToLength("Children: " + children, maxPropertyLength, "...");

      int maxNameLength = 60;
      String name = registry.getClass().getSimpleName() + " " + registry.getName();
      name = trimStringToLength(name, maxNameLength, "...");

      System.out.println(name + " " + variableString + " " + childrenString);
   }

   private static String trimStringToLength(String original, int length, String placeholder)
   {
      int chararcters = original.length();
      int placeholderLength = placeholder.length();

      if (chararcters > length)
         return original.substring(0, length - placeholderLength) + placeholder;
      else
         return StringUtils.rightPad(original, length, " ");
   }

}