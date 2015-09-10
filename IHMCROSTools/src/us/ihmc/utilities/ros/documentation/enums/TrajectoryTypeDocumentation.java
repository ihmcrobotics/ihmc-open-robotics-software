package us.ihmc.utilities.ros.documentation.enums;

import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.tools.DocumentedEnum;

public class TrajectoryTypeDocumentation implements DocumentedEnum<TrajectoryType>
{

   @Override
   public String getDocumentation(TrajectoryType var)
   {
      return TrajectoryType.BASIC.getDocumentation(var);
   }

   @Override
   public TrajectoryType[] getDocumentedValues()
   {
      return TrajectoryType.BASIC.getDocumentedValues();
   }

}
