package us.ihmc.simulationconstructionset.dataBuffer;

import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.simulationconstructionset.gui.config.VarGroup;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;

public class DataBufferTools
{
   public static ArrayList<YoVariable<?>> getVarsFromGroup(DataBuffer dataBuffer, String varGroupName, VarGroupList varGroupList)
   {
      if (varGroupName.equals("all"))
      {
         return dataBuffer.getVariables();
      }

      VarGroup varGroup = varGroupList.getVarGroup(varGroupName);
      String[] varNames = varGroup.getVars();
      String[] regularExpressions = varGroup.getRegularExpressions();

      return dataBuffer.getVars(varNames, regularExpressions);
   }
}
