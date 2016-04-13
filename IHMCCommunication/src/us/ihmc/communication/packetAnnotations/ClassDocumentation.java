package us.ihmc.communication.packetAnnotations;

import java.lang.annotation.*;

@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface ClassDocumentation
{
   String value() default "No Documentation For This Class Is Recorded.";
}
