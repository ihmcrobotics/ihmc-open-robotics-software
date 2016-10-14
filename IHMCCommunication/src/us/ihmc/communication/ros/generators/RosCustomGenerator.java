package us.ihmc.communication.ros.generators;

import java.lang.annotation.*;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */

public interface RosCustomGenerator
{
   String getRosTopic();

   String getRosPackage();

   RosFieldDefinition[] getFields();

   String getTypeDocumentation();

   String getMessageName();
}
