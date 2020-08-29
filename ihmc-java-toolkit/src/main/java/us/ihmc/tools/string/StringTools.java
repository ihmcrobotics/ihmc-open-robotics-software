package us.ihmc.tools.string;

import org.apache.logging.log4j.message.ParameterizedMessageFactory;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;

import java.util.function.Supplier;

public class StringTools
{
   private StringTools()
   {
      // disallow construction
   }

   public static String getEveryUppercaseLetter(String string)
   {
      String ret = "";
      int numberOfCharaters = string.length();
      for(int i = 0; i < numberOfCharaters; i++){
         if(Character.isUpperCase(string.charAt(i))){
            char characterToAdd = string.charAt(i);
            ret = ret + characterToAdd;
         }
      }

      return ret;
   }

   public static Supplier<String> format(String message, Object... parameters)
   {
      return () -> ParameterizedMessageFactory.INSTANCE.newMessage(message, parameters).getFormattedMessage();
   }

   public static String zUpPoseString(Pose3DReadOnly pose)
   {
      return "(" + FormattingTools.getFormattedDecimal3D(pose.getX())
            + ", " + FormattingTools.getFormattedDecimal3D(pose.getY())
            + ", " + FormattingTools.getFormattedDecimal3D(pose.getZ())
            + ") yaw: " + FormattingTools.getFormattedDecimal3D(pose.getYaw());
   }
}
