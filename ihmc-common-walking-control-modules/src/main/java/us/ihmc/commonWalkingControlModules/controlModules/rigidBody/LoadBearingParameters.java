package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.euclid.Axis3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.EnumParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class LoadBearingParameters
{
   private final EnumParameter<LoadBearingControlMode> defaultControlMode;
   private final DoubleParameter normalForceThresholdForLoaded;

   private final BooleanParameter[] angularAxisSelection = new BooleanParameter[3];
   private final BooleanParameter[] linearAxisSelection = new BooleanParameter[3];

   public LoadBearingParameters(String bodyName, YoRegistry registry)
   {
      String prefix = bodyName + "LoadBearing";

      defaultControlMode = new EnumParameter<>(prefix + "DefaultControlMode", registry, LoadBearingControlMode.class, false, LoadBearingControlMode.JOINTSPACE);
      normalForceThresholdForLoaded = new DoubleParameter(prefix + "ForceThreshold", registry, 10.0);

      for (int i = 0; i < 3; i++)
      {
         angularAxisSelection[i] = new BooleanParameter(prefix + "Angular" + Axis3D.values[i].name() + "Enabled", registry, i == 1);
         linearAxisSelection[i] = new BooleanParameter(prefix + "Linear" + Axis3D.values[i].name() + "Enabled", registry, true);
      }
   }

   public LoadBearingControlMode getDefaultControlMode()
   {
      return defaultControlMode.getValue();
   }

   public double getNormalForceThresholdForLoaded()
   {
      return normalForceThresholdForLoaded.getValue();
   }

   public boolean isLinearAxisEnabled(int index)
   {
      return linearAxisSelection[index].getValue();
   }

   public boolean isAngularAxisEnabled(int index)
   {
      return angularAxisSelection[index].getValue();
   }
}
