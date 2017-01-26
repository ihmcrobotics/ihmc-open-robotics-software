package us.ihmc.simulationconstructionset.scripts;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;

import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;


public class TimeScript implements Script
{
   // Rep invariant: timeScriptEntryList must be always sorted!
   private ArrayList<TimeScriptEntry> sortedTimeScriptEntryList = new ArrayList<TimeScriptEntry>();
   private final IntegerYoVariable nextTimeScriptIndex;

   public TimeScript(YoVariableRegistry registry)
   {
      nextTimeScriptIndex = new IntegerYoVariable("nextTimeScriptIndex", registry);
      nextTimeScriptIndex.set(0);
   }

   @Override
   public void doScript(double t)
   {
      if (nextTimeScriptIndex.getIntegerValue() >= sortedTimeScriptEntryList.size()) return;
      
      TimeScriptEntry timeScriptEntry = sortedTimeScriptEntryList.get(nextTimeScriptIndex.getIntegerValue());
      
      if (t >= timeScriptEntry.getTime())
      {
         timeScriptEntry.setVarsToValues();
         timeScriptEntry.doCommands();
         nextTimeScriptIndex.increment();
      }
   }

   public void addEntry(TimeScriptEntry timeScriptEntry)
   {
      if (timeScriptEntry == null)
         return;
      
      sortedTimeScriptEntryList.add(timeScriptEntry);
      Collections.sort(sortedTimeScriptEntryList);
   }

   public void removeEntry(TimeScriptEntry timeScriptEntry)
   {
      sortedTimeScriptEntryList.remove(timeScriptEntry);
   }

   public void readTimeScript(YoVariableHolder holder, String pathname)
   {
      readTimeScript(holder, new File(pathname));
   }

   public void readTimeScript(YoVariableHolder holder, String parent, String child)
   {
      readTimeScript(holder, new File(parent, child));
   }

   public void readTimeScript(YoVariableHolder holder, URL url)
   {
      try
      {
         InputStream inputStream = url.openStream();
         BufferedReader in = new BufferedReader(new InputStreamReader(inputStream));
         readTimeScript(holder, in);
      }
      catch (IOException exception)
      {
         System.err.println("Error when trying to readTimeScript: " + exception);
      }
   }


   public void readTimeScript(YoVariableHolder holder, File file)
   {
      try
      {
         BufferedReader in = new BufferedReader(new FileReader(file));
         readTimeScript(holder, in);
      }
      catch (FileNotFoundException exception)
      {
         System.err.println("Error when trying to readTimeScript: " + exception);
      }
   }


   public void readTimeScript(YoVariableHolder holder, BufferedReader in)
   {
      StringBuffer buffer = new StringBuffer();

      try
      {
         packBufferWithoutComments(in, buffer);
         in.close();
      }
      catch (FileNotFoundException exception)
      {
         System.err.println("Error when trying to readTimeScript: " + exception);
      }
      catch (IOException exception)
      {
         System.err.println("Error when trying to readTimeScript: " + exception);
      }

      // System.out.println(buffer);

      // Convert to an array of Strings:
      ArrayList<String> strings = extractStrings(buffer);

      // Each string is now an entry in the time Script:
      for (int i = 0; i < strings.size(); i++)
      {
         // System.out.println(strings.get(i));
         this.addEntry(parseTimeScriptEntry(holder, strings.get(i)));
      }

   }

   private TimeScriptEntry parseTimeScriptEntry(YoVariableHolder holder, String line)
   {
      TimeScriptEntry ret;

      // First characters must be t =
      int equalIndex = line.indexOf("=");
      if (equalIndex < 1)
      {
         System.err.println("Bad file format.  Line must start with t= ");
         System.err.println(line);

         return null;
      }

      String t_part = line.substring(0, equalIndex);
      t_part = t_part.trim();

      if (!t_part.equals("t"))
      {
         System.err.println("Bad file format.  Line must start with t= ");
         System.err.println(line);

         // System.err.println(t_part);
         return null;
      }

      // line = line.substring(equalIndex);
      // Next is the value followed by a colon:

      int colonIndex = line.indexOf(":");
      if (colonIndex < 0)
      {
         System.err.println("Bad file format.  No : in line ");
         System.err.println(line);

         return null;
      }

      t_part = line.substring(equalIndex + 1, colonIndex).trim();

      // System.out.println(t_part);
      double time = Double.parseDouble(t_part);

      ret = new TimeScriptEntry(time);

      // Now we have up to the time and the colon.  Shave off that part:
      line = line.substring(colonIndex + 1);

      // System.out.println(line);

      while (line.length() > 0)
      {
         // First characters must be varname =
         equalIndex = line.indexOf("=");

         if (equalIndex < 1)
         {
            System.err.println("Error! Bad file format.  Line must start with varname = ");
            System.err.println(line);

            return null;
         }

         String varname = line.substring(0, equalIndex).trim();
         YoVariable variable = holder.getVariable(varname);
         if (variable == null)
         {
            System.err.println("Warning!  Variable not recognized: " + varname);

            throw new RuntimeException();

            // System.err.println(line);
            // System.err.println(t_part);
            // return null;
         }

         // Next is the value followed by a semicolon:

         int semicolonIndex = line.indexOf(";");
         if (semicolonIndex < 0)
         {
            System.err.println("Error! Bad file format.  No ; in line ");
            System.err.println(line);

            return null;
         }

         t_part = line.substring(equalIndex + 1, semicolonIndex).trim();

         // System.out.println(t_part);
         switch(variable.getYoVariableType())
         {
         case DOUBLE:
         {  
            double value = Double.parseDouble(t_part);
            ret.addVarValue((DoubleYoVariable) variable, value);
            break;
         }
         case BOOLEAN:
         {
            boolean value;
            if (t_part.equals("1.0"))
            {
               value = true;
            }
            else
            {
               value = Boolean.parseBoolean(t_part);
            }
            
            ret.addVarValue((BooleanYoVariable) variable, value);
            break;
         }
         case ENUM:
         {
            @SuppressWarnings("rawtypes")
            EnumYoVariable enumYoVariable = (EnumYoVariable) variable;
            @SuppressWarnings({ "unchecked", "rawtypes" })
            Enum value = Enum.valueOf(enumYoVariable.getEnumType(), t_part);            
            ret.addVarValue(enumYoVariable, value);
            break;
         }
         case INTEGER:
         {
            int value = Integer.parseInt(t_part);
            ret.addVarValue((IntegerYoVariable) variable, value);
            break;
         }
         default:
         {
            throw new RuntimeException("Should not get here!");
         }
         
         }
        
         line = line.substring(semicolonIndex + 1).trim();

      }

      return ret;

   }

   private ArrayList<String> extractStrings(StringBuffer buffer)
   {
      ArrayList<String> ret = new ArrayList<String>();
      boolean done = false;

      int index = 0;
      StringBuffer tempBuffer1, tempBuffer2 = new StringBuffer();
      char c = ' ';

      while (!done)
      {
         tempBuffer1 = new StringBuffer();

         while ((index < buffer.length()) && ((c = buffer.charAt(index)) != ':') && ((c = buffer.charAt(index)) != ';'))
         {
            tempBuffer1.append(c);
            index++;

            // System.out.println(tempBuffer1);
         }

         if (index < buffer.length())
            tempBuffer1.append(c);

         if (c == ':')
         {
            // System.out.println(":::" + tempBuffer2);
            if (tempBuffer2.length() > 0)
            {
               ret.add(tempBuffer2.toString().trim());
               tempBuffer2 = new StringBuffer();
            }
         }

         tempBuffer2.append(tempBuffer1.toString());

         // System.out.println(tempBuffer2);
         // System.out.println();
         // System.out.println("index, buffer: " + index + ", " + buffer.length());

         if (index == buffer.length())
         {
            done = true;

            if (c != ':')
            {
               ret.add(tempBuffer2.toString().trim());
            }
         }

         index++;
      }

      return ret;
   }

   private void packBufferWithoutComments(BufferedReader in, StringBuffer buffer)
   {
      String line;
      inComment = false;

      // Pack it all into a StringBuffer:
      try
      {
         // while((!doneParsing) && (line = in.readLine()) != null)
         while ((line = in.readLine()) != null)
         {
            packBufferWithLine(line, buffer);
         }
      }
      catch (IOException e)
      {
         System.err.println(e);
      }
   }

   private boolean inComment = false;

   private void packBufferWithLine(String line, StringBuffer buffer)
   {
      int startCommentIndex, endCommentIndex, lineCommentIndex;

      // If in a comment, look for the end of it, otherwise throw the whole line away:
      if (inComment)
      {
         endCommentIndex = line.indexOf("*/");

         if (endCommentIndex >= 0)
         {
            inComment = false;
            line = line.substring(endCommentIndex + 2);
         }
         else
            return;
      }

      // System.out.println(line + " " + line.indexOf("//"));

      // Look for the next comments:

      lineCommentIndex = line.indexOf("//");
      startCommentIndex = line.indexOf("/*");

      if (startCommentIndex >= 0)
      {
         // if startComment comes first, remove it, go into comment mode and go from top...
         if ((lineCommentIndex < 0) || (lineCommentIndex > startCommentIndex))
         {
            buffer.append(line.substring(0, startCommentIndex));
            line = line.substring(startCommentIndex);
            inComment = true;
            packBufferWithLine(line, buffer);

            return;
         }
      }

      // Remove single line comments:
      if (lineCommentIndex >= 0)
      {
         buffer.append(line.substring(0, lineCommentIndex));

         return;
      }

      buffer.append(line);
   }

}
