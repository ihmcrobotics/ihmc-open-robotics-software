package us.ihmc.communication.annotations.ros;

import java.lang.annotation.*;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface RosExportedField
{
	String documentation();
}
