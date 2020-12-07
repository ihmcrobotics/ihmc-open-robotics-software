package us.ihmc.tools.string;

import org.apache.logging.log4j.message.ParameterizedMessageFactory;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

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

   public static Supplier<String> format3D(String message, Object... parameters)
   {
      Object[] formattedParameters = new Object[parameters.length];
      for (int i = 0; i < parameters.length; i++)
      {
         if (parameters[i] instanceof Double)
         {
            formattedParameters[i] = new FormattedDouble((Double) parameters[i]);
         }
         else
         {
            formattedParameters[i] = parameters[i];
         }
      }

      return format(message, formattedParameters);
   }

   public static String zUpPoseString(Pose3DReadOnly pose)
   {
      return "(" + FormattingTools.getFormattedDecimal3D(pose.getX())
            + ", " + FormattingTools.getFormattedDecimal3D(pose.getY())
            + ", " + FormattingTools.getFormattedDecimal3D(pose.getZ())
            + ") yaw: " + FormattingTools.getFormattedDecimal3D(pose.getYaw());
   }

   public static String tupleString(Tuple3DReadOnly tuple)
   {
      return "(" + FormattingTools.getFormattedDecimal3D(tuple.getX())
             + ", " + FormattingTools.getFormattedDecimal3D(tuple.getY())
             + ", " + FormattingTools.getFormattedDecimal3D(tuple.getZ())
             + ")";
   }

   public static class FormattedDouble
   {
      private final Double number;

      public FormattedDouble(Double number)
      {
         this.number = number;
      }

      @Override
      public String toString()
      {
         return FormattingTools.getFormattedDecimal3D(number);
      }
   }
}
