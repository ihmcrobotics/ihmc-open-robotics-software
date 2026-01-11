package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

import java.util.Random;

class MultiPendulumRobotDefinition extends RobotDefinition
{
   private static final double smallInertia = 1e-4;
   private final Random random = new Random(3902);

   private final int N;
   private double[] linkLength;
   private double[] comOffset;
   private double[] inertia;
   private double[] mass;
   private Axis3D[] axes;

   private final RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
   private final RevoluteJointDefinition[] joints;

   MultiPendulumRobotDefinition(String name, int N)
   {
      super(name);
      this.N = N;

      setRootBodyDefinition(elevator);

      linkLength = new double[N];
      comOffset = new double[N];
      inertia = new double[N];
      mass = new double[N];
      joints = new RevoluteJointDefinition[N];
      axes = new Axis3D[N];

      for (int i = 0; i < N; i++)
      {
         // set robot properties
         linkLength[i] = EuclidCoreRandomTools.nextDouble(random, 0.2, 0.6);
         comOffset[i] = EuclidCoreRandomTools.nextDouble(random, 0.2 * linkLength[i], 0.8 * linkLength[i]);
         mass[i] = EuclidCoreRandomTools.nextDouble(random, 0.5, 1.5);
         inertia[i] = EuclidCoreRandomTools.nextDouble(random, 0.25, 0.75);
         axes[i] = Axis3D.values[i % 3];

         // setup id robot
         RigidBodyDefinition predecessor = i == 0 ? elevator : joints[i - 1].getSuccessor();
         Vector3D jointOffset = i == 0 ? new Vector3D() : new Vector3D(0.0, 0.0, -linkLength[i - 1]);
         Vector3D comOffset = new Vector3D(0.0, 0.0, -this.comOffset[i]);
         joints[i] = new RevoluteJointDefinition("joint" + i, jointOffset, axes[i]);
         predecessor.addChildJoint(joints[i]);
         RigidBodyDefinition successor = new RigidBodyDefinition("link" + i);
         successor.setMass(mass[i]);
         successor.getMomentOfInertia().setPrincipalComponents(inertia[i], smallInertia, smallInertia);
         successor.setCenterOfMassOffset(comOffset);
         VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
         visualDefinitionFactory.addCylinder(linkLength[i], 0.02, ColorDefinitions.Orange());
         successor.addVisualDefinitions(visualDefinitionFactory.getVisualDefinitions());
         joints[i].setSuccessor(successor);
      }
   }

   public int getN()
   {
      return N;
   }

   public RevoluteJointDefinition[] getJointDefinitions()
   {
      return joints;
   }

   public void setInitialState(double... q)
   {
      for (int i = 0; i < Math.min(N, q.length); i++)
      {
         joints[i].setInitialJointState(q[i]);
      }
   }
}
