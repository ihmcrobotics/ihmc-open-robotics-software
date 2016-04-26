package us.ihmc.communication.annotations.ros;

import java.lang.annotation.*;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface RosMessagePacket
{
   String documentation();
   String topic() default "NONE";
   String rosPackage();
}
