package us.ihmc.communication.ros.generators;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class Point2dRosMessageGenerator implements RosCustomGenerator
{
   private final RosFieldDefinition[] fields;

   public Point2dRosMessageGenerator()
   {
      RosFieldDefinition xFieldDefinition = new RosFieldDefinition()
      {
         @Override
         public String getType()
         {
            return "float64";
         }

         @Override
         public String getFieldName()
         {
            return "x";
         }

         @Override
         public String getDocumentation()
         {
            return "The first coordinate of the point in a 2D plane";
         }

         @Override public boolean isConstant()
         {
            return false;
         }

         @Override public Object getConstantValue()
         {
            return null;
         }
      };

      RosFieldDefinition yFieldDefinition = new RosFieldDefinition()
      {
         @Override
         public String getType()
         {
            return "float64";
         }

         @Override
         public String getFieldName()
         {
            return "y";
         }

         @Override
         public String getDocumentation()
         {
            return "The second coordinate of the point in a 2D plane";
         }

         @Override public boolean isConstant()
         {
            return false;
         }

         @Override public Object getConstantValue()
         {
            return null;
         }
      };

      fields = new RosFieldDefinition[]{xFieldDefinition, yFieldDefinition};
   }

   @Override
   public String getRosTopic()
   {
      return RosMessagePacket.NO_CORRESPONDING_TOPIC_STRING;
   }

   @Override
   public String getRosPackage()
   {
      return "ihmc_msgs";
   }

   @Override
   public RosFieldDefinition[] getFields()
   {
      return fields;
   }

   @Override
   public String getTypeDocumentation()
   {
      return "This message represents a point on a 2d plane. The coordinates are referred to as \"x\" and \"y\"";
   }

   @Override
   public String getMessageName()
   {
      return "Point2dRosMessage";
   }
}
