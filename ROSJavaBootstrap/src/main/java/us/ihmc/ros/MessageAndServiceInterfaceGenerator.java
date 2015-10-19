package us.ihmc.ros;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.List;

import org.ros.internal.message.GenerateInterfaces;

import com.google.common.collect.Lists;

public class MessageAndServiceInterfaceGenerator
{


   public static void main(String[] args) throws IOException
   {
      List<String> arguments = Lists.newArrayList(args);
      if (arguments.size() < 2)
      {
         System.err
               .println("insufficient Arguments. Generating the Interfaces from Spoof requires at least two arguments:\n"
                     + "First: the output directory for the interface files\n"
                     + "Second and following: merged together to form the ros package path");
         return;
      }
      File outputDirectory = new File(arguments.remove(0));
      
      Collection<File> packagePath = Lists.newArrayList();
      for (int i=0;i<arguments.size();i++)
      {
         packagePath.add(new File(arguments.remove(0)));
      }

      log("InterfaceFromSpoof: packagePath is "+packagePath);
      GenerateInterfaces generateInterfaces = new GenerateInterfaces();
      generateInterfaces.generate(outputDirectory, arguments, packagePath);
   }

   private static void log(String aMessage)
   {
      System.out.println(aMessage);
   }

}
