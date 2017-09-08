package us.ihmc.utilities.ros.types;

import java.util.ArrayList;
import java.util.List;

import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.internal.message.topic.TopicMessageFactory;
import org.ros.message.MessageDefinitionProvider;

import sensor_msgs.PointField;

public enum PointType
{
   XYZ, XYZI, XYZRGB;

   public static PointType fromFromFieldNames(List<sensor_msgs.PointField> fields)
   {
	   if(fields.size() == 3)
	   {
		   return XYZ;
	   }
	   final String thirdFieldName =fields.get(3).getName();
	   switch (thirdFieldName)
	   {
	   case "luminance" :
	   case "intensity" :
		   return XYZI;
		   
	   case "rgb" :
	   case "rgba" :
		   return XYZRGB;
	   }
	   throw new RuntimeException("unknown PointType: " + thirdFieldName);
   }

   PointType()
   {
   }

   public int getPointStep()
   {
	   switch (this)
	      {
	         case XYZI :
	         case XYZRGB:
	        	 return 16;
	         case XYZ:
	        	 return 12;
	      }
      return 16;
   }

   public List<PointField> getPointField()
   {
      MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
      TopicMessageFactory topicMessageFactory = new TopicMessageFactory(messageDefinitionProvider);

      ArrayList<PointField> pointFields = new ArrayList<>();
      String[] fieldNames = null;
      switch (this)
      {
         case XYZI :
            fieldNames = new String[] {"x", "y", "z", "luminance"};

            break;

         case XYZRGB :
            fieldNames = new String[] {"x", "y", "z", "rgb"};

            break;
         case XYZ :
        	 fieldNames = new String[] {"x", "y", "z"};
      }

      int offset = 0;
      for (int i = 0; i < fieldNames.length; i++)
      {
         PointField pointField = topicMessageFactory.newFromType(PointField._TYPE);
         pointField.setName(fieldNames[i]);
         pointField.setDatatype(PointField.FLOAT32);
         pointField.setCount(1);
         pointField.setOffset(offset);
         pointFields.add(pointField);
         offset += 4;
      }

      return pointFields;
   }
}
