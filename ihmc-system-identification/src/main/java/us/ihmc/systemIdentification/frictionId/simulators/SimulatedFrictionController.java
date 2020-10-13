package us.ihmc.systemIdentification.frictionId.simulators;

import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.systemIdentification.frictionId.frictionModels.AsymmetricCoulombViscousStribeckFrictionModel;
import us.ihmc.systemIdentification.frictionId.frictionModels.JointFrictionModel;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class SimulatedFrictionController implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final HashMap<OneDegreeOfFreedomJoint, YoDouble> viscousDampings = new HashMap<>();
   private final HashMap<OneDegreeOfFreedomJoint, YoDouble> dynamicFrictions = new HashMap<>();
   private final HashMap<OneDegreeOfFreedomJoint, YoDouble> stribeckValues = new HashMap<>();
   private final HashMap<OneDegreeOfFreedomJoint, YoDouble> coulombValues = new HashMap<>();
   private final HashMap<OneDegreeOfFreedomJoint, YoDouble> frictionForces = new HashMap<>();

   private final List<OneDegreeOfFreedomJoint> frictionJoints = new ArrayList<>();

   private final double[] parameters = new double[8];
   private final JointFrictionModel frictionModel = new AsymmetricCoulombViscousStribeckFrictionModel();

   public SimulatedFrictionController(FloatingRootJointRobot robot, CoulombViscousStribeckFrictionParameters parameters)
   {
      this(robot, parameters.getViscousDamping(), parameters.getDynamicFriction(), parameters.getStribeckValue(), parameters.getCoulombFriction());
   }

   public SimulatedFrictionController(FloatingRootJointRobot robot, double viscousValue, double dynamicValue, double stribeckValue, double coulombValue)
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint simulatedJoint : oneDegreeOfFreedomJoints)
      {
         frictionJoints.add(simulatedJoint);

         String jointName = simulatedJoint.getName();

         YoDouble viscousDamping = new YoDouble(jointName + "_viscousDamping", registry);
         YoDouble dynamicFriction = new YoDouble(jointName + "_dynamicFriction", registry);
         YoDouble stribeckFrictionValue = new YoDouble(jointName + "_stribeckValue", registry);
         YoDouble coulombFriction = new YoDouble(jointName + "_coulombFriction", registry);
         YoDouble frictionForce = new YoDouble(jointName + "_frictionForce", registry);

         viscousDamping.set(viscousValue);
         dynamicFriction.set(dynamicValue);
         stribeckFrictionValue.set(stribeckValue);
         coulombFriction.set(coulombValue);

         viscousDampings.put(simulatedJoint, viscousDamping);
         dynamicFrictions.put(simulatedJoint, dynamicFriction);
         stribeckValues.put(simulatedJoint, stribeckFrictionValue);
         coulombValues.put(simulatedJoint, coulombFriction);
         frictionForces.put(simulatedJoint, frictionForce);
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
         OneDegreeOfFreedomJoint simulatedJoint = frictionJoints.get(jointIndex);
         double torque = simulatedJoint.getTau();
         double qd = simulatedJoint.getQD();

         parameters[0] = viscousDampings.get(simulatedJoint).getDoubleValue();
         parameters[1] = dynamicFrictions.get(simulatedJoint).getDoubleValue();
         parameters[2] = stribeckValues.get(simulatedJoint).getDoubleValue();
         parameters[3] = coulombValues.get(simulatedJoint).getDoubleValue();
         parameters[4] = viscousDampings.get(simulatedJoint).getDoubleValue();
         parameters[5] = dynamicFrictions.get(simulatedJoint).getDoubleValue();
         parameters[6] = stribeckValues.get(simulatedJoint).getDoubleValue();
         parameters[7] = coulombValues.get(simulatedJoint).getDoubleValue();

         frictionModel.updateParameters(parameters);
         frictionModel.computeFrictionForce(qd);
         frictionForces.get(simulatedJoint).set(frictionModel.getFrictionForce());
         simulatedJoint.setTau(torque - frictionForces.get(simulatedJoint).getDoubleValue());
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

   @Override
   public String getDescription()
   {
      return null;
   }
}
