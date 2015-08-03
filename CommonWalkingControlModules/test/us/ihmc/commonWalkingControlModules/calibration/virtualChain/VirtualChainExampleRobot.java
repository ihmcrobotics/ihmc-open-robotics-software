package us.ihmc.commonWalkingControlModules.calibration.virtualChain;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.robotics.Axis;

public abstract class VirtualChainExampleRobot extends Robot implements RobotRandomPositionMover
{
   private static final long serialVersionUID = -4030450330772550919L;

   public static VirtualChainExampleRobot constructExampleOne()
   {
      VirtualChainExampleRobotOne ret = new VirtualChainExampleRobotOne();
      return ret;
   }
   
   public static VirtualChainExampleRobot constructExampleTwo()
   {
      VirtualChainExampleRobotTwo ret = new VirtualChainExampleRobotTwo();
      return ret;
   }
   
   public static VirtualChainExampleRobot constructExampleThree()
   {
      VirtualChainExampleRobotThree ret = new VirtualChainExampleRobotThree();
      return ret;
   }
   
   public static VirtualChainExampleRobot constructExampleFour()
   {
      VirtualChainExampleRobotFour ret = new VirtualChainExampleRobotFour();
      return ret;
   }
   
   
   public static final double
      mass1 = 10.0, mass2 = 5.0, mass3 = 1.0;

// private static final Vector3d offset1 = new Vector3d(), offset2 = new Vector3d(0.2, 0.0, 0.0), offset3 = new Vector3d(0.2, 0.0, 0.0);
   public static final Vector3d
      offset1 = new Vector3d(), offset2 = new Vector3d(0.4, 0.2, -0.1), offset3 = new Vector3d(0.3, 0.2, 0.2);

// private static final Vector3d offset1 = new Vector3d(), offset2 = new Vector3d(0.4, 0.2, -0.1), offset3 = new Vector3d(0.0, 0.0, 0.0);

// private static final Vector3d comOffset1 = new Vector3d(0.1, 0.0, 0.0), comOffset2 = new Vector3d(0.1, 0.0, 0.0), comOffset3 = new Vector3d(0.1, 0.0, 0.0);
   public static final Vector3d
      comOffset1 = new Vector3d(0.1, 0.22, -0.3), comOffset2 = new Vector3d(-0.1, 0.06, 0.15), comOffset3 = new Vector3d(-0.13, 0.7, 0.43);

// private static final Vector3d comOffset1 = new Vector3d(0.1, 0.22, -0.3), comOffset2 = new Vector3d(), comOffset3 = new Vector3d();


   public VirtualChainExampleRobot(String name)
   {
      super(name);
   }

   private static class VirtualChainExampleRobotOne extends VirtualChainExampleRobot
   {
      private static final long serialVersionUID = -387898369605560846L;
      private final PinJoint joint1, joint2, joint3;
      private final Random random = new Random(100L);

      public VirtualChainExampleRobotOne()
      {
         super("testOne");

         joint1 = new PinJoint("joint1", offset1, this, Axis.Y);
         Link link1 = new Link("link1");
         link1.setMassAndRadiiOfGyration(mass1, 0.01, 0.02, 0.03);
         link1.setComOffset(comOffset1);
         joint1.setLink(link1);
         this.addRootJoint(joint1);

         joint2 = new PinJoint("joint2", offset2, this, Axis.X);
         Link link2 = new Link("link2");
         link2.setMassAndRadiiOfGyration(mass2, 0.01, 0.02, 0.03);
         link2.setComOffset(comOffset2);
         joint2.setLink(link2);
         joint1.addJoint(joint2);

         joint3 = new PinJoint("joint3", offset3, this, Axis.Z);
         Link link3 = new Link("link3");
         link3.setMassAndRadiiOfGyration(mass3, 0.01, 0.02, 0.03);
         link3.setComOffset(comOffset3);
         joint3.setLink(link3);
         joint2.addJoint(joint3);
      }

      public void moveToRandomPosition()
      {
         joint1.setInitialState(-Math.PI + 2.0 * random.nextDouble() * Math.PI, 0.0);
         joint2.setInitialState(-Math.PI + 2.0 * random.nextDouble() * Math.PI, 0.0);
         joint3.setInitialState(-Math.PI + 2.0 * random.nextDouble() * Math.PI, 0.0);
      }
   }



   private static class VirtualChainExampleRobotTwo extends VirtualChainExampleRobot
   {
      private static final long serialVersionUID = -634147542490794495L;
      private final ArrayList<PinJoint> pinJoints = new ArrayList<PinJoint>();
      private final Random random = new Random(100L);

      public VirtualChainExampleRobotTwo()
      {
         super("testTwo");

         int numberOfJoints = 100;

         for (int i = 0; i < numberOfJoints; i++)
         {
            Vector3d offset = new Vector3d();

            if (i != 0)
            {
               offset = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
            }

            Vector3d axis = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
            axis.normalize();

            PinJoint pinJoint = new PinJoint("joint_" + i, offset, this, axis);
            Link link = new Link("link_" + i);
            link.setMassAndRadiiOfGyration(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
            pinJoint.setLink(link);

            if (i == 0)
            {
               this.addRootJoint(pinJoint);
            }

            else
            {
               int jointIndex = (int) (random.nextDouble() * (pinJoints.size() - 1));

               Joint parentJoint = pinJoints.get(jointIndex);
               parentJoint.addJoint(pinJoint);
            }

            pinJoints.add(pinJoint);
         }

      }

      public void moveToRandomPosition()
      {
         for (PinJoint pinJoint : pinJoints)
         {
            pinJoint.setInitialState(-Math.PI + 2.0 * random.nextDouble() * Math.PI, 0.0);
         }

      }
   }


   private static class VirtualChainExampleRobotThree extends VirtualChainExampleRobot
   {
      private static final long serialVersionUID = 4774793234722509693L;
      private final FloatingJoint joint1;
      private final PinJoint joint2, joint3;
      private final Random random = new Random(100L);

      public VirtualChainExampleRobotThree()
      {
         super("test");

         joint1 = new FloatingJoint("joint1", offset1, this);
         Link link1 = new Link("link1");
         link1.setMassAndRadiiOfGyration(mass1, 0.01, 0.02, 0.03);
         link1.setComOffset(comOffset1);
         joint1.setLink(link1);
         this.addRootJoint(joint1);

         joint2 = new PinJoint("joint2", offset2, this, Axis.X);
         Link link2 = new Link("link2");
         link2.setMassAndRadiiOfGyration(mass2, 0.01, 0.02, 0.03);
         link2.setComOffset(comOffset2);
         joint2.setLink(link2);
         joint1.addJoint(joint2);

         joint3 = new PinJoint("joint3", offset3, this, Axis.Z);
         Link link3 = new Link("link3");
         link3.setMassAndRadiiOfGyration(mass3, 0.01, 0.02, 0.03);
         link3.setComOffset(comOffset3);
         joint3.setLink(link3);
         joint2.addJoint(joint3);
      }

      public void moveToRandomPosition()
      {
         joint1.setPosition(random.nextDouble(), random.nextDouble(), random.nextDouble());
         joint1.setYawPitchRoll(-Math.PI + 2.0 * random.nextDouble() * Math.PI, -Math.PI + 2.0 * random.nextDouble() * Math.PI, -Math.PI + 2.0 * random.nextDouble() * Math.PI);
         joint2.setInitialState(-Math.PI + 2.0 * random.nextDouble() * Math.PI, 0.0);
         joint3.setInitialState(-Math.PI + 2.0 * random.nextDouble() * Math.PI, 0.0);
      }
   }


   private static class VirtualChainExampleRobotFour extends VirtualChainExampleRobot
   {
      private static final long serialVersionUID = -6524194700402276350L;
      private final FloatingJoint floatingJoint;
      private final ArrayList<Joint> pinAndSliderJoints = new ArrayList<Joint>();
      private final Random random = new Random(100L);

      public VirtualChainExampleRobotFour()
      {
         super("testFour");

         floatingJoint = new FloatingJoint("floating", new Vector3d(), this);
         Link link = new Link("floatinglink");
         link.setMassAndRadiiOfGyration(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         floatingJoint.setLink(link);
         this.addRootJoint(floatingJoint);

         int numberOfJoints = 100;

         for (int i = 0; i < numberOfJoints; i++)
         {
            Vector3d offset = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
            Vector3d axis = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
            axis.normalize();

            Joint joint;

//          if (random.nextDouble() > 0.5)
            {
               PinJoint pinJoint = new PinJoint("pinjoint_" + i, offset, this, axis);
               joint = pinJoint;
            }

//          else
//          {
//             SliderJoint sliderJoint = new SliderJoint("sliderjoint_" + i, offset, this, axis);
//             joint = sliderJoint;
//          }

            link = new Link("link_" + i);
            link.setMassAndRadiiOfGyration(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
            joint.setLink(link);

            Joint parentJoint = floatingJoint;

            if ((random.nextDouble() > 0.25) && (pinAndSliderJoints.size() > 0))
            {
               int jointIndex = (int) (random.nextDouble() * (pinAndSliderJoints.size() - 1));
               parentJoint = pinAndSliderJoints.get(jointIndex);
            }

            parentJoint.addJoint(joint);

            pinAndSliderJoints.add(joint);
         }

      }

      public void moveToRandomPosition()
      {
         floatingJoint.setPosition(random.nextDouble(), random.nextDouble(), random.nextDouble());
         floatingJoint.setYawPitchRoll(-Math.PI + 2.0 * random.nextDouble() * Math.PI, -Math.PI + 2.0 * random.nextDouble() * Math.PI,
                                       -Math.PI + 2.0 * random.nextDouble() * Math.PI);

         for (Joint joint : pinAndSliderJoints)
         {
            if (joint.getClass() == PinJoint.class)
            {
               ((PinJoint) joint).setInitialState(-Math.PI + 2.0 * random.nextDouble() * Math.PI, 0.0);
            }
            else
            {
               ((SliderJoint) joint).setInitialState(random.nextDouble(), 0.0);
            }
         }

      }
   }
}
