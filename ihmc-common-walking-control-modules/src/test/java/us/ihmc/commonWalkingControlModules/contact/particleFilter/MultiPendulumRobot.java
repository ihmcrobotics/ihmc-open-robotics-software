package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

import java.util.Random;

class MultiPendulumRobot extends Robot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double gravity = 9.81;
   private static final double smallInertia = 1e-4;
   private final Random random = new Random(3902);

   private final int N;
   private double[] linkLength;
   private double[] comOffset;
   private double[] inertia;
   private double[] mass;
   private Axis3D[] axes;

   private final RigidBody elevator = new RigidBody("elevator", worldFrame);
   private final OneDoFJoint[] joints;
   private final OneDegreeOfFreedomJoint[] scsJoints;

   MultiPendulumRobot(String name, int N)
   {
      super(name);
      this.N = N;

      linkLength = new double[N];
      comOffset = new double[N];
      inertia = new double[N];
      mass = new double[N];
      joints = new OneDoFJoint[N];
      scsJoints = new OneDegreeOfFreedomJoint[N];
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
         RigidBodyBasics predecessor = i == 0 ? elevator : joints[i - 1].getSuccessor();
         Vector3D jointOffset = i == 0 ? new Vector3D() : new Vector3D(0.0, 0.0, -linkLength[i - 1]);
         Vector3D comOffset = new Vector3D(0.0, 0.0, - this.comOffset[i]);
         joints[i] = new RevoluteJoint("joint" + i, predecessor, jointOffset, axes[i]);
         RigidBodyBasics successor = new RigidBody("link" + i, joints[i], inertia[i], smallInertia, smallInertia, mass[i], comOffset);

         // setup scs robot
         scsJoints[i] = new PinJoint("joint" + i, jointOffset, this, axes[i]);
         Link scsLink = new Link("link" + i);
         scsLink.setMass(mass[i]);
         scsLink.setMomentOfInertia(inertia[i], smallInertia, smallInertia);
         scsLink.setComOffset(comOffset);
         Graphics3DObject graphics3DObject = new Graphics3DObject();
         graphics3DObject.rotate(Math.PI, Axis3D.X);
         graphics3DObject.addCylinder(linkLength[i], 0.02, YoAppearance.randomColor(random));
         scsLink.setLinkGraphics(graphics3DObject);
         scsJoints[i].setLink(scsLink);

         if(i == 0)
            addRootJoint(scsJoints[i]);
         else
            scsJoints[i - 1].addJoint(scsJoints[i]);
      }
   }

   public void updateState()
   {
      for (int i = 0; i < N; i++)
      {
         joints[i].setQ(scsJoints[i].getQ());
         joints[i].setQd(scsJoints[i].getQD());
         joints[i].setTau(scsJoints[i].getTau());
      }

      elevator.updateFramesRecursively();
   }

   public int getN()
   {
      return N;
   }

   public OneDoFJoint[] getJoints()
   {
      return joints;
   }

   public OneDegreeOfFreedomJoint[] getScsJoints()
   {
      return scsJoints;
   }

   public void setInitialState(double... q)
   {
      for (int i = 0; i < Math.min(N, q.length); i++)
      {
         scsJoints[i].setQ(q[i]);
      }
   }
}
