package us.ihmc.communication.ros.generators;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public interface RosFieldDefinition
{
   String getType();
   String getFieldName();
   String getDocumentation();
   boolean isConstant();
   Object getConstantValue();
}
