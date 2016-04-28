package us.ihmc.communication.ros.generators;

import java.lang.annotation.*;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface RosMessagePacket
{
   String NO_CORRESPONDING_TOPIC_STRING = "NONE";

   String documentation();
   String topic() default NO_CORRESPONDING_TOPIC_STRING;
   String rosPackage();
}
