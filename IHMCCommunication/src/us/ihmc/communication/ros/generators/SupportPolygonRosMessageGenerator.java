package us.ihmc.communication.ros.generators;

/**
 * Created by parallels on 5/4/16.
 */
public class SupportPolygonRosMessageGenerator implements RosCustomGenerator
{
   private final RosFieldDefinition[] fields;

   public SupportPolygonRosMessageGenerator()
   {
      fields = new RosFieldDefinition[3];

      RosFieldDefinition maxVerticesConstant = new RosFieldDefinition()
      {
         @Override public String getType()
         {
            return "int32";
         }

         @Override public String getFieldName()
         {
            return "MAXIMUM_NUMBER_OF_VERTICES";
         }

         @Override public String getDocumentation()
         {
            return "Constant defining max number of possible elements in the array of points";
         }

         @Override public boolean isConstant()
         {
            return true;
         }

         @Override public Object getConstantValue()
         {
            return 8;
         }
      };

      RosFieldDefinition numberOfVertices = new RosFieldDefinition()
      {
         @Override public String getType()
         {
            return "int32";
         }

         @Override public String getFieldName()
         {
            return "number_of_vertices";
         }

         @Override public String getDocumentation()
         {
            return "The number of vertices in the array of points";
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

      RosFieldDefinition verticesFieldDefintion = new RosFieldDefinition()
      {
         @Override public String getType()
         {
            return "ihmc_msgs/Point2dRosMessage[]";
         }

         @Override public String getFieldName()
         {
            return "vertices";
         }

         @Override public String getDocumentation()
         {
            return "The vertices of the support polygon";
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

      fields[0] = maxVerticesConstant;
      fields[1] = numberOfVertices;
      fields[2] = verticesFieldDefintion;
   }

   @Override public String getRosTopic()
   {
      return RosMessagePacket.NO_CORRESPONDING_TOPIC_STRING;
   }

   @Override public String getRosPackage()
   {
      return RosMessagePacket.CORE_IHMC_PACKAGE;
   }

   @Override public RosFieldDefinition[] getFields()
   {
      return fields;
   }

   @Override public String getTypeDocumentation()
   {
      return "This message contains of an array of points that represent the convex hull of a support polygon for a single robot foot";
   }

   @Override public String getMessageName()
   {
      return "SupportPolygonRosMessage";
   }
}
