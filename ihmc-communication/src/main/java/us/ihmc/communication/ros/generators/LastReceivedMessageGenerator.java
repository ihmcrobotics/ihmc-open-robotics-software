package us.ihmc.communication.ros.generators;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class LastReceivedMessageGenerator implements RosCustomGenerator
{
   private final RosFieldDefinition[] fields;

   public LastReceivedMessageGenerator()
   {
      RosFieldDefinition typeField = new RosFieldDefinition()
      {
         @Override
         public String getType()
         {
            return "string";
         }

         @Override
         public String getFieldName()
         {
            return "type";
         }

         @Override
         public String getDocumentation()
         {
            return "The type of the last message received";
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

      RosFieldDefinition idField = new RosFieldDefinition()
      {
         @Override
         public String getType()
         {
            return "int64";
         }

         @Override
         public String getFieldName()
         {
            return "unique_id";
         }

         @Override
         public String getDocumentation()
         {
            return "The Unique ID of the last message received.";
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

      RosFieldDefinition receiveTimestampField = new RosFieldDefinition()
      {
         @Override
         public String getType()
         {
            return "int64";
         }

         @Override
         public String getFieldName()
         {
            return "receive_timestamp";
         }

         @Override
         public String getDocumentation()
         {
            return "The timestamp at which the message was received.";
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

      RosFieldDefinition timeSinceLastReceivedField = new RosFieldDefinition()
      {
         @Override
         public String getType()
         {
            return "float64";
         }

         @Override
         public String getFieldName()
         {
            return "time_since_last_received";
         }

         @Override
         public String getDocumentation()
         {
            return "The time since a message was received";
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

      fields = new RosFieldDefinition[]{typeField, idField, receiveTimestampField, timeSinceLastReceivedField};
   }

   @Override
   public String getRosTopic()
   {
      return "/output/last_received_message";
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
      return "Last Received Message echo's back the ID and type of the last message received by the IHMC ROS API.";
   }

   @Override
   public String getMessageName()
   {
      return "LastReceivedMessage";
   }
}
