package us.ihmc.communication.annotations.ros;

import java.lang.annotation.*;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface RosEnumValueDocumentation
{
   String documentation();
}
