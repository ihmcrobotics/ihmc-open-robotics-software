package us.ihmc.valkyrie.parameters;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.PlanForToeOffCalculator;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.tools.saveableModule.SaveableModuleState;
import us.ihmc.tools.saveableModule.SaveableModuleStateTools;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class ValkyrieCoPTrajectoryParameters extends CoPTrajectoryParameters
{
   public ValkyrieCoPTrajectoryParameters()
   {
      entryCMPLengthOffsetFactor.set(1.0 / 3.0);
      ballCMPLengthOffsetFactor.set(1.0 / 8.0);
      exitCMPLengthOffsetFactor.set(1.0 / 3.0);

      entryCMPOffset.set(0.0, -0.005);
      ballCMPOffset.set(0.0, 0.01);
      exitCMPOffset.set(0.0, 0.025);

      entryCMPMinX.set(-0.04);
      entryCMPMaxX.set(0.03);

      ballCMPMinX.set(0.0);
      ballCMPMaxX.set(0.055);

      exitCMPMinX.set(0.0);
      exitCMPMaxX.set(0.08);
   }
}
