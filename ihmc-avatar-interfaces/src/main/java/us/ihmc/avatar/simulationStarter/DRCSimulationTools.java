package us.ihmc.avatar.simulationStarter;

import us.ihmc.commons.FormattingTools;

public abstract class DRCSimulationTools
{

   public enum Modules
   {
      SIMULATION,
      SENSOR_MODULE,
      ROS_MODULE,
      ZERO_POSE_PRODUCER,
      REA_MODULE,
      REA_UI,
      KINEMATICS_TOOLBOX,
      KINEMATICS_PLANNING_TOOLBOX,
      FOOTSTEP_PLANNING_TOOLBOX,
      WHOLE_BODY_TRAJECTORY_TOOLBOX,
      FIDUCIAL_DETECTOR,
      OBJECT_DETECTOR,
      DIRECTIONAL_CONTROL_TOOLBOX;

      public String getPropertyNameForEnable()
      {
         return "enable" + FormattingTools.underscoredToCamelCase(toString(), true);
      }

      public String getPropertyNameForSelected()
      {
         return "select" + FormattingTools.underscoredToCamelCase(toString(), true);
      }

      public boolean isAlwaysEnabled()
      {
         if (this == SIMULATION)
            return true;
         else
            return false;
      }

      public boolean getDefaultValueForEnable()
      {
         return true;
      }

      public boolean getDefaultValueForSelected()
      {
         if (this == SIMULATION || this == SENSOR_MODULE)
            return true;
         else
            return false;
      }

      public String getName()
      {
         return FormattingTools.underscoredToCamelCase(toString(), true);
      }
   }
}
