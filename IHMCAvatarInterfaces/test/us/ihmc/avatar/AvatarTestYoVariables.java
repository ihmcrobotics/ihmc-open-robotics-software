package us.ihmc.avatar;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public abstract class AvatarTestYoVariables
{
   private final DoubleYoVariable yoTime;

   private final DoubleYoVariable pelvisX;
   private final DoubleYoVariable pelvisY;
   private final DoubleYoVariable pelvisZ;
   private final DoubleYoVariable pelvisYaw;

   public AvatarTestYoVariables(SimulationConstructionSet scs)
   {
      yoTime = (DoubleYoVariable) scs.getVariable("t");

      pelvisX = (DoubleYoVariable) scs.getVariable("q_x");
      pelvisY = (DoubleYoVariable) scs.getVariable("q_y");
      pelvisZ = (DoubleYoVariable) scs.getVariable("q_z");
      pelvisYaw = (DoubleYoVariable) scs.getVariable("q_yaw");
   }

   public DoubleYoVariable getYoTime()
   {
      return yoTime;
   }

   public DoubleYoVariable getPelvisX()
   {
      return pelvisX;
   }

   public DoubleYoVariable getPelvisY()
   {
      return pelvisY;
   }

   public DoubleYoVariable getPelvisZ()
   {
      return pelvisZ;
   }

   public DoubleYoVariable getPelvisYaw()
   {
      return pelvisYaw;
   }
}
