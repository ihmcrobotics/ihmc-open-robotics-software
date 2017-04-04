package us.ihmc.robotics.dataStructures.registry;

import java.io.Serializable;
import java.util.ArrayList;

public class NameSpace implements Serializable
{
   private static final long serialVersionUID = -2584260031738121095L;
   private final String name;
   private final ArrayList<String> subNames;

   public NameSpace(String name)
   {
      this.name = name;
      subNames = getSubStrings(name);
      checkRepInvariant();
   }
   
   public ArrayList<String> getSubNames()
   {
      return subNames;
   }

   public NameSpace(ArrayList<String> subNamesStartingWithRoot)
   {
      this(recreateNameFromSubNames(subNamesStartingWithRoot));
   }

   private static ArrayList<String> getSubStrings(String name)
   {
      ArrayList<String> ret = new ArrayList<String>();

      String endOfName = name;
      while (true)
      {
         int indexOfDot = endOfName.indexOf(".");
         if (indexOfDot == -1)
         {
            ret.add(endOfName);

            return ret;
         }

         String nextOne = endOfName.substring(0, indexOfDot);
         ret.add(nextOne);
         endOfName = endOfName.substring(indexOfDot + 1);
      }
   }

   private void checkRepInvariant()
   {
      String check = "";
      for (int i = 0; i < subNames.size(); i++)
      {
         String partToAdd = subNames.get(i);
         if (partToAdd.equals(""))
            throw new RuntimeException("Cannot construct NameSpace with double .. or a . at the end or beginning, or empty!");
         String subName = partToAdd;

         check = check + subName;
         if (i < subNames.size() - 1)
            check = check + ".";
      }

      if (!check.equals(name))
         throw new RuntimeException("Rep Invariant doesn't hold!! name = " + name + ", check = " + check);

      // Make sure no duplicate subnames.
      for (int i = 0; i < subNames.size() - 1; i++)
      {
         String subNameI = subNames.get(i);
         for (int j = i + 1; j < subNames.size(); j++)
         {
            String subNameJ = subNames.get(j);

            if (subNameI.equals(subNameJ))
               throw new RuntimeException("Cannot construct NameSpace with duplicate subNameSpaces! nameSpace = " + this);
         }
      }
   }

   public String getName()
   {
      return name;
   }

   public String getShortName()
   {
      return subNames.get(subNames.size() - 1);
   }

   public String getRootName()
   {
      return subNames.get(0);
   }

   public String getNameWithRootStripped()
   {
      if (subNames.size() < 2)
         return null;

      StringBuilder builder = new StringBuilder();
      for (int i = 1; i < subNames.size() - 1; i++)
      {
         builder.append(subNames.get(i));
         builder.append(".");
      }

      builder.append(subNames.get(subNames.size() - 1));

      return builder.toString();
   }


   private static String recreateNameFromSubNames(ArrayList<String> subNames)
   {
      StringBuilder builder = new StringBuilder();
      for (int i = 0; i < subNames.size() - 1; i++)
      {
         builder.append(subNames.get(i));
         builder.append(".");
      }

      builder.append(subNames.get(subNames.size() - 1));

      return builder.toString();
   }


   public String toString()
   {
      return name;
   }

   /**
    * Checks if the given name is in this nameSpace. Must match from the end up to a dot or the start.
    * For example "robot.controller.module" endsWith("module") and endsWith("controller.module") and endsWith("robot.controller.module")
    * but does not endsWith("bot.controller.module") or endsWith("") or anything else.
    * @param name Name to check if this NameSpace ends with.
    * @return boolean Whether this NameSpace ends with the given name.
    */
   public boolean endsWith(String nameToMatch)
   {
      // Only true if it does end with this and, if there are more letters, that the previous one is a "."
      if (!this.name.endsWith(nameToMatch))
         return false;
      if (this.name.length() == nameToMatch.length())
         return true;
      if (this.name.length() < nameToMatch.length())
         return false;    // Defensive test. Really should never get here if the previous didn't pass.

      int index = this.name.length() - nameToMatch.length() - 1;
      char character = this.name.charAt(index);
      if (character == '.')
         return true;

      return false;
   }

   /**
    * Checks if the given name is in this nameSpace. Must match from the start up to a dot or the end.
    * For example "robot.controller.module" startsWith("robot") and startsWith("robot.controller") and startsWith("robot.controller.module")
    * but does not startsWith("robot.controller.mod") or startsWith("") or anything else.
    * @param name Name to check if this NameSpace starts with.
    * @return boolean Whether this NameSpace starts with the given name.
    */
   public boolean startsWith(String nameToMatch)
   {
      // Only true if it does start with this and, if there are more letters, that the next one is a "."
      if (!this.name.startsWith(nameToMatch))
         return false;
      if (this.name.length() == nameToMatch.length())
         return true;
      if (this.name.length() < nameToMatch.length())
         return false;
      if (this.name.charAt(nameToMatch.length()) == '.')
         return true;

      return false;
   }

   /**
    * Checks if the given name is in this nameSpace. Must match from the start or a dot up to the end or a dot.
    * For example "robot.controller.module" contains("robot") and contains("robot.controller") and contains("robot.controller.module")
    * and contains("module") and contains("controller.module") and contains("robot.controller.module") and contains("controller")
    * but does not contains("robot.controller.mod") or contains("") or anything else.
    * @param name Name to check if this NameSpace contains it.
    * @return boolean Whether this NameSpace contains the given name.
    */
   public boolean contains(String nameToMatch)
   {
      ArrayList<String> subNamesToMatch = getSubStrings(nameToMatch);

      if (subNamesToMatch.size() > subNames.size())
         return false;

      // Throw off the ones on the front of this subNames
      String firstSubNameToMatch = subNamesToMatch.get(0);

      int index;
      for (index = 0; index < subNames.size(); index++)
      {
         if (subNames.get(index).equals(firstSubNameToMatch))
            break;
      }

      if (subNames.size() - index < subNamesToMatch.size())
         return false;

      // Now all the rest have to match.
      for (int indexToMatch = 0; indexToMatch < subNamesToMatch.size(); indexToMatch++)
      {
         if (!subNames.get(index).equals(subNamesToMatch.get(indexToMatch)))
            return false;

         index++;
      }

      return true;
   }


   public boolean equals(Object nameSpace)
   {
      if (nameSpace == this)
         return true;
      if (!(nameSpace instanceof NameSpace))
         return false;

      NameSpace nameSpaceToCheck = (NameSpace) nameSpace;

      return nameSpaceToCheck.name.equals(this.name);
   }

   /*
    * Strips the given nameSpace off of the beginning of this nameSpace and returns a new NameSpace.
    * If the two don't match all the way through, returns null.
    * If the two are the same, returns null.
    */
   public NameSpace stripOffFromBeginning(NameSpace nameSpaceToRemove)
   {
      if (nameSpaceToRemove == null)
         return new NameSpace(this.name);

      ArrayList<String> thisSubNames = this.subNames;
      ArrayList<String> stripSubNames = nameSpaceToRemove.subNames;

      if (stripSubNames.size() >= thisSubNames.size())
         return null;

      ArrayList<String> newSubNames = new ArrayList<String>();

      for (int i = 0; i < stripSubNames.size(); i++)
      {
         if (!thisSubNames.get(i).equals(stripSubNames.get(i)))
         {
            return null;
         }
      }

      for (int i = stripSubNames.size(); i < thisSubNames.size(); i++)
      {
         newSubNames.add(thisSubNames.get(i));
      }

      return new NameSpace(newSubNames);
   }

   public static NameSpace createNameSpaceFromAFullVariableName(String fullVariableName)
   {
      int lastIndexOfDot = fullVariableName.lastIndexOf(".");
      if (lastIndexOfDot < 0)
      {
         return new NameSpace("NoNameSpaceRegistry");
      }

      String nameSpaceString = fullVariableName.substring(0, lastIndexOfDot);

      return new NameSpace(nameSpaceString);
   }

   public static String stripOffNameSpaceToGetVariableName(String variableName)
   {
      int lastIndexOfDot = variableName.lastIndexOf(".");
      if (lastIndexOfDot < 0)
      {
         return variableName;
      }

      return variableName.substring(lastIndexOfDot + 1);
   }

}
