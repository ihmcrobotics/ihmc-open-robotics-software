package us.ihmc.systemIdentification.frictionId.simulators;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.systemIdentification.frictionId.frictionModels.AsymmetricCoulombViscousStribeckFrictionModel;
import us.ihmc.systemIdentification.frictionId.frictionModels.JointFrictionModel;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimulatedFrictionController implements Controller
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final HashMap<OneDoFJointReadOnly, YoDouble> viscousDampings = new HashMap<>();
   private final HashMap<OneDoFJointReadOnly, YoDouble> dynamicFrictions = new HashMap<>();
   private final HashMap<OneDoFJointReadOnly, YoDouble> stribeckValues = new HashMap<>();
   private final HashMap<OneDoFJointReadOnly, YoDouble> coulombValues = new HashMap<>();
   private final HashMap<OneDoFJointReadOnly, YoDouble> frictionForces = new HashMap<>();

   private final List<OneDoFJointReadOnly> frictionJoints = new ArrayList<>();

   private final double[] parameters = new double[8];
   private final JointFrictionModel frictionModel = new AsymmetricCoulombViscousStribeckFrictionModel();
   private final ControllerOutput controllerOutput;

   public SimulatedFrictionController(ControllerInput controllerInput, ControllerOutput controllerOutput, CoulombViscousStribeckFrictionParameters parameters)
   {
      this(controllerInput,
           controllerOutput,
           parameters.getViscousDamping(),
           parameters.getDynamicFriction(),
           parameters.getStribeckValue(),
           parameters.getCoulombFriction());
   }

   public SimulatedFrictionController(ControllerInput controllerInput,
                                      ControllerOutput controllerOutput,
                                      double viscousValue,
                                      double dynamicValue,
                                      double stribeckValue,
                                      double coulombValue)
   {
      this.controllerOutput = controllerOutput;
      for (JointReadOnly simulatedJoint : controllerInput.getInput().getJointsToConsider())
      {
         if (!(simulatedJoint instanceof OneDoFJointReadOnly))
            continue;

         OneDoFJointReadOnly simulatedOneDoFJoint = (OneDoFJointReadOnly) simulatedJoint;
         frictionJoints.add(simulatedOneDoFJoint);

         String jointName = simulatedOneDoFJoint.getName();

         YoDouble viscousDamping = new YoDouble(jointName + "_viscousDamping", registry);
         YoDouble dynamicFriction = new YoDouble(jointName + "_dynamicFriction", registry);
         YoDouble stribeckFrictionValue = new YoDouble(jointName + "_stribeckValue", registry);
         YoDouble coulombFriction = new YoDouble(jointName + "_coulombFriction", registry);
         YoDouble frictionForce = new YoDouble(jointName + "_frictionForce", registry);

         viscousDamping.set(viscousValue);
         dynamicFriction.set(dynamicValue);
         stribeckFrictionValue.set(stribeckValue);
         coulombFriction.set(coulombValue);

         viscousDampings.put(simulatedOneDoFJoint, viscousDamping);
         dynamicFrictions.put(simulatedOneDoFJoint, dynamicFriction);
         stribeckValues.put(simulatedOneDoFJoint, stribeckFrictionValue);
         coulombValues.put(simulatedOneDoFJoint, coulombFriction);
         frictionForces.put(simulatedOneDoFJoint, frictionForce);
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void doControl()
   {
      for (int jointIndex = 0; jointIndex < frictionJoints.size(); jointIndex++)
      {
         OneDoFJointReadOnly oneDoFJoint = frictionJoints.get(jointIndex);
         OneDoFJointStateBasics jointOutput = controllerOutput.getOneDoFJointOutput(oneDoFJoint);
         double qd = oneDoFJoint.getQd();

         parameters[0] = viscousDampings.get(oneDoFJoint).getDoubleValue();
         parameters[1] = dynamicFrictions.get(oneDoFJoint).getDoubleValue();
         parameters[2] = stribeckValues.get(oneDoFJoint).getDoubleValue();
         parameters[3] = coulombValues.get(oneDoFJoint).getDoubleValue();
         parameters[4] = viscousDampings.get(oneDoFJoint).getDoubleValue();
         parameters[5] = dynamicFrictions.get(oneDoFJoint).getDoubleValue();
         parameters[6] = stribeckValues.get(oneDoFJoint).getDoubleValue();
         parameters[7] = coulombValues.get(oneDoFJoint).getDoubleValue();

         frictionModel.updateParameters(parameters);
         frictionModel.computeFrictionForce(qd);
         frictionForces.get(oneDoFJoint).set(frictionModel.getFrictionForce());
         jointOutput.addEffort(-frictionForces.get(oneDoFJoint).getDoubleValue());
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }
}
