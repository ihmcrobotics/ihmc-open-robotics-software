package us.ihmc.communication.ros.generators;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface RosMessagePacket
{
   String NO_CORRESPONDING_TOPIC_STRING = "NONE";
   String CORE_IHMC_PACKAGE = "ihmc_msgs";

   String documentation();
   String topic() default NO_CORRESPONDING_TOPIC_STRING;
   String rosPackage();
}
