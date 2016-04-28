package us.ihmc.communication.ros.generators;

import java.lang.annotation.*;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface RosExportedField
{
   String documentation();
}
