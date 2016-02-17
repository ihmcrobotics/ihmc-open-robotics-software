package us.ihmc.communication.packetAnnotations;

import java.lang.annotation.*;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface FieldDocumentation
{
	String value();
}
