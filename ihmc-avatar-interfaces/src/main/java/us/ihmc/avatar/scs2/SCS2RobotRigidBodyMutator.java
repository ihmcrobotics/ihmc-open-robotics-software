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

/**
 * This class is used to mutate the mass, center of mass offset, and moment of inertia of a robot's rigid bodies in SCS2, simulating the effects of time-varying
 * inertial parameters.
 * <p>
 * The mutators are themselves thin wrappers around {@link YoFunctionGenerator}s, which provide a simple way to mutate the inertial parameters during a simulation.
 * </p>
 *
 * @author James Foster
 */
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

      private final YoFunctionGenerator massMutator;
      private final YoFunctionGenerator[] comOffsetMutators;
      private final YoFunctionGenerator[] momentOfInertiaMutators;

      RigidBodyMutator(RigidBodyBasics rigidBody, DoubleProvider time, YoRegistry registry, double dt)
      {
         this.rigidBody = rigidBody;

         massMutator = new YoFunctionGenerator(rigidBody.getName() + "_MassMutator", time, registry, false, dt);
         double defaultMass = rigidBody.getInertia().getMass();
         massMutator.setOffset(defaultMass);
         massMutator.setAmplitude(defaultMass * DEFAULT_MASS_PERCENTAGE_MIN_MAX);

         comOffsetMutators = new YoFunctionGenerator[3];
         comOffsetMutators[0] = new YoFunctionGenerator(YoGeometryNameTools.createXName(rigidBody.getName() + "_CoMOffset", "Mutator"), time, registry, false, dt);
         comOffsetMutators[1] = new YoFunctionGenerator(YoGeometryNameTools.createYName(rigidBody.getName() + "_CoMOffset", "Mutator"), time, registry, false, dt);
         comOffsetMutators[2] = new YoFunctionGenerator(YoGeometryNameTools.createZName(rigidBody.getName() + "_CoMOffset", "Mutator"), time, registry, false, dt);
         for (YoFunctionGenerator mutator : comOffsetMutators)
         {
            mutator.setOffset(0.0);  // CoM offsets are nearly always default zero
            mutator.setAmplitude(DEFAULT_COM_OFFSET_MIN_MAX);
         }

         momentOfInertiaMutators = new YoFunctionGenerator[3];
         momentOfInertiaMutators[0] = new YoFunctionGenerator(rigidBody.getName() + "_MomentOfInertiaIxxMutator", time, registry, false, dt);
         momentOfInertiaMutators[0].setOffset(rigidBody.getInertia().getMomentOfInertia().getM00());
         momentOfInertiaMutators[1] = new YoFunctionGenerator(rigidBody.getName() + "_MomentOfInertiaIyyMutator", time, registry, false, dt);
         momentOfInertiaMutators[1].setOffset(rigidBody.getInertia().getMomentOfInertia().getM11());
         momentOfInertiaMutators[2] = new YoFunctionGenerator(rigidBody.getName() + "_MomentOfInertiaIzzMutator", time, registry, false, dt);
         momentOfInertiaMutators[2].setOffset(rigidBody.getInertia().getMomentOfInertia().getM22());
         double[] defaultInertiaDiagonals = new double[3];
         defaultInertiaDiagonals[0] = rigidBody.getInertia().getMomentOfInertia().getM00();
         defaultInertiaDiagonals[1] = rigidBody.getInertia().getMomentOfInertia().getM11();
         defaultInertiaDiagonals[2] = rigidBody.getInertia().getMomentOfInertia().getM22();
         for (int i = 0; i < 3; ++i)
         {
            momentOfInertiaMutators[i].setOffset(defaultInertiaDiagonals[i]);
            momentOfInertiaMutators[i].setAmplitude(DEFAULT_INERTIA_PERCENTAGE_MIN_MAX);
         }
      }

      void mutate()
      {
         rigidBody.getInertia().setMass(massMutator.getValue());

         rigidBody.getInertia().setCenterOfMassOffset(comOffsetMutators[0].getValue(),
                                                      comOffsetMutators[1].getValue(),
                                                      comOffsetMutators[2].getValue());

         // Only mutating the diagonals of the moment of inertia: Ixx, Iyy, Izz
         rigidBody.getInertia().setMomentOfInertia(momentOfInertiaMutators[0].getValue(),
                                                   momentOfInertiaMutators[1].getValue(),
                                                   momentOfInertiaMutators[2].getValue());
      }
   }
}
