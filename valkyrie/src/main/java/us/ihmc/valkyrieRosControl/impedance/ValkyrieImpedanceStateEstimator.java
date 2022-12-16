package us.ihmc.valkyrieRosControl.impedance;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.Map;

import static us.ihmc.valkyrieRosControl.impedance.ValkyrieJointList.allJoints;

public class ValkyrieImpedanceStateEstimator
{
   private final RigidBodyBasics rootBody;
   private final Map<String, OneDoFJointBasics> nameToJointMap = new HashMap<>();
   private final Map<String, AlphaFilteredYoVariable> nameToFilteredJointVelocity = new HashMap<>();
   private final Map<String, EffortJointHandle> nameToEffortHandleMap;

   public ValkyrieImpedanceStateEstimator(OneDoFJointBasics[] controlledOneDoFJoints,
                                          Map<String, EffortJointHandle> nameToEffortHandleMap,
                                          double controlDT,
                                          YoRegistry registry)
   {
      this.rootBody = MultiBodySystemTools.getRootBody(controlledOneDoFJoints[0].getSuccessor());
      this.nameToEffortHandleMap = nameToEffortHandleMap;

      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         nameToJointMap.put(controlledOneDoFJoints[i].getName(), controlledOneDoFJoints[i]);

         double alphaVelocity = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(25.0, controlDT);
         AlphaFilteredYoVariable filteredVelocity = new AlphaFilteredYoVariable("qd_filt_" + controlledOneDoFJoints[i].getName(), registry, alphaVelocity);
         nameToFilteredJointVelocity.put(controlledOneDoFJoints[i].getName(), filteredVelocity);
      }
   }

   public void update()
   {
      for (int i = 0; i < allJoints.size(); i++)
      {
         EffortJointHandle effortJointHandle = nameToEffortHandleMap.get(allJoints.get(i));

         OneDoFJointBasics joint = nameToJointMap.get(allJoints.get(i));
         AlphaFilteredYoVariable filteredVelocity = nameToFilteredJointVelocity.get(allJoints.get(i));
         filteredVelocity.update(effortJointHandle.getVelocity());

         joint.setQ(effortJointHandle.getPosition());
         joint.setQd(filteredVelocity.getDoubleValue());
      }

      rootBody.updateFramesRecursively();
   }
}
