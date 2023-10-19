package us.ihmc.avatar.scs2;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class SCS2RobotRigidBodyMutator implements Controller
{
   private final List<RigidBodyMutator> rigidBodyMutators = new ArrayList<>();
   private final YoRegistry registry;

   SCS2RobotRigidBodyMutator(Robot robot, DoubleProvider time, double dt)
   {
      registry = new YoRegistry(getClass().getSimpleName());

      for (RigidBodyBasics rigidBody : robot.getRootBody().subtreeList())
      {
         if (rigidBody.getInertia() != null)
            rigidBodyMutators.add(new RigidBodyMutator(rigidBody, time, registry, dt));
      }
   }

   @Override
   public void doControl()
   {
      for (RigidBodyMutator mutator : rigidBodyMutators)
         mutator.mutate();
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private static class RigidBodyMutator
   {
      private static final double DEFAULT_MASS_PERCENTAGE_MIN_MAX = 0.1;
      private static final double DEFAULT_COM_OFFSET_MIN_MAX = 0.1;  // 10cm offset
      private static final double DEFAULT_INERTIA_PERCENTAGE_MIN_MAX = 0.1;

      private final RigidBodyBasics rigidBody;

      private final double defaultMass;
      private final double[] defaultInertiaDiagonals;

      /**
       * When mutating mass, the associated {@link #massMutator} is used to generate a percentage plus/minus difference in the default mass of the rigid body.
       * This YoDouble controls the minimum (maximum) percentage.
       */
      private final YoDouble massPercentageMinMax;
      /**
       * When mutating center of mass offset, the default offset is nearly always zero. It is difficult to set bounds on the CoM offset for a rigid body without
       * detailed knowledge of its geometric properties. This YoDouble controls the  minimum (maximum) CoM offset bound in each of x, y, z that the associated
       * {@link #comOffsetMutators} can generate.
       */
      private final YoDouble comOffsetMinMax;
      /**
       * When mutating moment of inertia, only the diagonal entries (Ixx, Iyy, Izz) are modified. The associated {@link #comOffsetMutators} are used to generate
       * a percentage plus/minus difference in the default inertial diagonal entries. This YoDouble controls the minimum (maximum) percentage.
       */
      private final YoDouble momentOfInertiaPercentageMinMax;

      private final YoFunctionGenerator massMutator;
      private final YoFunctionGenerator[] comOffsetMutators;
      private final YoFunctionGenerator[] momentOfInertiaMutators;

      RigidBodyMutator(RigidBodyBasics rigidBody, DoubleProvider time, YoRegistry registry, double dt)
      {
         this.rigidBody = rigidBody;

         defaultMass = rigidBody.getInertia().getMass();
         massPercentageMinMax = new YoDouble(rigidBody.getName() + "_MassPercentageMinMax", registry);
         massPercentageMinMax.set(DEFAULT_MASS_PERCENTAGE_MIN_MAX);
         massMutator = new YoFunctionGenerator(rigidBody.getName() + "_MassMutator", time, registry, false, dt);
         massMutator.setOffset(rigidBody.getInertia().getMass());

         comOffsetMinMax = new YoDouble(rigidBody.getName() + "_CoMOffsetMinMax", registry);
         comOffsetMinMax.set(DEFAULT_COM_OFFSET_MIN_MAX);
         comOffsetMutators = new YoFunctionGenerator[3];
         comOffsetMutators[0] = new YoFunctionGenerator(YoGeometryNameTools.createXName(rigidBody.getName() + "_CoMOffset", "Mutator"), time, registry, false, dt);
         comOffsetMutators[1] = new YoFunctionGenerator(YoGeometryNameTools.createYName(rigidBody.getName() + "_CoMOffset", "Mutator"), time, registry, false, dt);
         comOffsetMutators[2] = new YoFunctionGenerator(YoGeometryNameTools.createZName(rigidBody.getName() + "_CoMOffset", "Mutator"), time, registry, false, dt);

         defaultInertiaDiagonals = new double[3];
         defaultInertiaDiagonals[0] = rigidBody.getInertia().getMomentOfInertia().getM00();
         defaultInertiaDiagonals[1] = rigidBody.getInertia().getMomentOfInertia().getM11();
         defaultInertiaDiagonals[2] = rigidBody.getInertia().getMomentOfInertia().getM22();
         momentOfInertiaPercentageMinMax = new YoDouble(rigidBody.getName() + "_MomentOfInertiaPercentageMinMax", registry);
         momentOfInertiaPercentageMinMax.set(DEFAULT_INERTIA_PERCENTAGE_MIN_MAX);
         momentOfInertiaMutators = new YoFunctionGenerator[3];
         momentOfInertiaMutators[0] = new YoFunctionGenerator(rigidBody.getName() + "_MomentOfInertiaIxxMutator", time, registry, false, dt);
         momentOfInertiaMutators[0].setOffset(rigidBody.getInertia().getMomentOfInertia().getM00());
         momentOfInertiaMutators[1] = new YoFunctionGenerator(rigidBody.getName() + "_MomentOfInertiaIyyMutator", time, registry, false, dt);
         momentOfInertiaMutators[1].setOffset(rigidBody.getInertia().getMomentOfInertia().getM11());
         momentOfInertiaMutators[2] = new YoFunctionGenerator(rigidBody.getName() + "_MomentOfInertiaIzzMutator", time, registry, false, dt);
         momentOfInertiaMutators[2].setOffset(rigidBody.getInertia().getMomentOfInertia().getM22());

      }

      void mutate()
      {
         massMutator.setAmplitude(defaultMass * massPercentageMinMax.getDoubleValue());
         rigidBody.getInertia().setMass(massMutator.getValue());

         for (YoFunctionGenerator mutator : comOffsetMutators)
            mutator.setAmplitude(comOffsetMinMax.getDoubleValue());
         rigidBody.getInertia().setCenterOfMassOffset(comOffsetMutators[0].getValue(),
                                                      comOffsetMutators[1].getValue(),
                                                      comOffsetMutators[2].getValue());

         for (int i = 0; i < 3; ++i)
            momentOfInertiaMutators[i].setAmplitude(defaultInertiaDiagonals[i] * momentOfInertiaPercentageMinMax.getDoubleValue());
         // Only mutating the diagonals of the moment of inertia: Ixx, Iyy, Izz
         rigidBody.getInertia().setMomentOfInertia(momentOfInertiaMutators[0].getValue(),
                                                   momentOfInertiaMutators[1].getValue(),
                                                   momentOfInertiaMutators[2].getValue());
      }
   }
}
