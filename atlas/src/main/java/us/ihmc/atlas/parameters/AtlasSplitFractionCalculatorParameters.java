package us.ihmc.atlas.parameters;

import us.ihmc.footstepPlanning.icp.SplitFractionCalculatorParameterKeys;
import us.ihmc.footstepPlanning.icp.SplitFractionCalculatorParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class AtlasSplitFractionCalculatorParameters extends StoredPropertySet implements SplitFractionCalculatorParametersBasics
{
   public AtlasSplitFractionCalculatorParameters()
   {
      this("ihmc-open-robotics-software", "atlas/src/main/resources");
   }

   public AtlasSplitFractionCalculatorParameters(String projectName, String pathToResources)
   {
      super(SplitFractionCalculatorParameterKeys.keys, AtlasSplitFractionCalculatorParameters.class, projectName, pathToResources);

      setTransferSplitFractionAtFullDepth(0.3);
      setTransferWeightDistributionAtFullDepth(0.75);
      setStepHeightForLargeStepDown(0.1);
      setLargestStepDownHeight(0.3);

      load();
   }

   public static void main(String[] args)
   {
      AtlasSplitFractionCalculatorParameters parameters = new AtlasSplitFractionCalculatorParameters();
      parameters.save();
   }
}
