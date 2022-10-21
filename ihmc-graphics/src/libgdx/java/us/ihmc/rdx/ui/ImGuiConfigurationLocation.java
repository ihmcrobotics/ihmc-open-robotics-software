package us.ihmc.rdx.ui;

import us.ihmc.tools.io.HybridResourceMode;

public enum ImGuiConfigurationLocation
{
   VERSION_CONTROL, USER_HOME;

   public HybridResourceMode toHybridResourceMode()
   {
      return this == VERSION_CONTROL ? HybridResourceMode.WORKSPACE : HybridResourceMode.EXTERNAL;
   }
}
