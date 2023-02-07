package us.ihmc.behaviors.tools.affordanceTemplates;

import us.ihmc.euclid.referenceFrame.FramePose3D;

public interface AffordanceTemplateAction
{
   void loadTemplate(String affordance); // load from JSON

   void framePoseToPack(FramePose3D framePose);

   // save JSON
}
