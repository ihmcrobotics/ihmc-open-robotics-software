package us.ihmc.utilities.ros.bootstrap;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.List;

import org.ros.internal.message.GenerateInterfaces;

import com.google.common.collect.Lists;

import ihmc_msgs.FootstepDataListRosMessage;
import ihmc_msgs.WholeBodyTrajectoryRosMessage;

/**
 * Regenerates the ROSJava messages from ROS messages ".msg".
 * <p>
 * This is the second step when updating the ROS API. The first step is DRCROSMessageGenerator.
 * </p>
 * <p>
 * The program arguments should be something like : "../ihmc-ros-tools/generated-src
 * ROSMessagesAndServices"
 * </p>
 * <p>
 * For most messages, this will be enough, but for some other like
 * {@link WholeBodyTrajectoryRosMessage} and {@link FootstepDataListRosMessage}, there is a last
 * step which is about updating IHMCROSTranslationRuntimeTools.
 * </p>
 */
public class MessageAndServiceInterfaceGenerator
{

   public static void main(String[] args) throws IOException
   {
      List<String> arguments = Lists.newArrayList(args);
      if (arguments.size() < 2)
      {
         System.err.println("insufficient Arguments. Generating the Interfaces from Spoof requires at least two arguments:\n"
               + "First: the output directory for the interface files\n" + "Second and following: merged together to form the ros package path");
         return;
      }
      File outputDirectory = new File(arguments.remove(0));

      Collection<File> packagePath = Lists.newArrayList();
      for (int i = 0; i < arguments.size(); i++)
      {
         packagePath.add(new File(arguments.remove(0)));
      }

      log("InterfaceFromSpoof: packagePath is " + packagePath);
      GenerateInterfaces generateInterfaces = new GenerateInterfaces();
      generateInterfaces.generate(outputDirectory, arguments, packagePath);
   }

   private static void log(String aMessage)
   {
      System.out.println(aMessage);
   }

}
