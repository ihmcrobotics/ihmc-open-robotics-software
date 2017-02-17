package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.random.RandomTools;

public class RandomRobotGenerator
{
   public static Robot generateRandomLinearChainRobot(String name, boolean startWithFloatingJoint, int numberOfPinJoints, Random random)
   {
      Robot robot = new Robot(name);
      
      Joint parentJoint = null;

      if (startWithFloatingJoint)
      {
         String jointName = "floatingJoint";
//         Vector3d offset = RandomTools.generateRandomVector(random, random.nextDouble() + 0.1);

         FloatingJoint floatingJoint = new FloatingJoint(jointName, new Vector3D(), robot);
         Link link = generateRandomLink(random, jointName, null);
         floatingJoint.setLink(link);
         
         robot.addRootJoint(floatingJoint);
         
         parentJoint = floatingJoint;
      }
      
      for (int jointNumber = 0; jointNumber < numberOfPinJoints; jointNumber++)
      {
         Vector3D offset = RandomTools.generateRandomVector(random, random.nextDouble() + 0.1);
         Vector3D axis = RandomTools.generateRandomVector(random, 1.0);
         axis.normalize();

         String jointName = "joint" + jointNumber;
         PinJoint pinJoint = new PinJoint(jointName, offset, robot, axis);
         
         Link link = generateRandomLink(random, jointName, axis);
         pinJoint.setLink(link);
         
         if (parentJoint == null)
         {
            robot.addRootJoint(pinJoint);
         }
         else
         {
            Graphics3DObject parentLinkGraphics = parentJoint.getLink().getLinkGraphics();
            
            Vector3D zVector = new Vector3D(0.0, 0.0, 1.0);
            AxisAngle rotationAxisAngle = computeAxisAngleToAlignVectors(zVector, offset);
            
            parentLinkGraphics.identity();
            parentLinkGraphics.rotate(rotationAxisAngle);
            parentLinkGraphics.addCylinder(offset.length(), offset.length()/20.0, YoAppearance.randomColor(random));
       
            parentJoint.addJoint(pinJoint);
         }
         
         parentJoint = pinJoint;
      }
      
      return robot;
   }

   public static void setRandomJointPositions(Robot robot, Random random)
   {
      ArrayList<Joint> rootJoints = robot.getRootJoints();
      
      for (Joint rootJoint : rootJoints)
      {
         setRandomJointPosition(rootJoint, random);
         
         ArrayList<Joint> childrenJoints = new ArrayList<Joint>();
         rootJoint.recursiveGetChildrenJoints(childrenJoints);
         
         for (Joint childJoint : childrenJoints)
         {
            setRandomJointPosition(childJoint, random);
         }
      }
   }
   
   public static void setRandomJointVelocities(Robot robot, Random random)
   {
      ArrayList<Joint> rootJoints = robot.getRootJoints();
      
      for (Joint rootJoint : rootJoints)
      {
         setRandomJointVelocity(rootJoint, random);
         
         ArrayList<Joint> childrenJoints = new ArrayList<Joint>();
         rootJoint.recursiveGetChildrenJoints(childrenJoints);
         
         for (Joint childJoint : childrenJoints)
         {
            setRandomJointVelocity(childJoint, random);
         }
      }
   }
   
   public static void setRandomJointPosition(Joint joint, Random random)
   {
      if (joint instanceof FloatingJoint)
      {
         FloatingJoint floatingJoint = (FloatingJoint) joint;
       
         RigidBodyTransform randomTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         floatingJoint.setRotationAndTranslation(randomTransform);
      }
      else if (joint instanceof FloatingPlanarJoint)
      {
         FloatingPlanarJoint floatingJoint = (FloatingPlanarJoint) joint;
         
         double rotation = RandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         floatingJoint.setRotation(rotation);

         Tuple2DBasics position = RandomTools.generateRandomVector2d(random, 1.0);
         Tuple2DBasics velocity = RandomTools.generateRandomVector2d(random, 0.5);
         floatingJoint.setCartesianPosition(position, velocity);
      }
      else if (joint instanceof PinJoint)
      {
         PinJoint pinJoint = (PinJoint) joint;
         double rotation = RandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         pinJoint.setQ(rotation);
      }
      else if (joint instanceof SliderJoint)
      {
         SliderJoint sliderJoint = (SliderJoint) joint;
         
         double position = RandomTools.generateRandomDouble(random, -0.1, 0.1);         
         sliderJoint.setQ(position);
      }
      else
      {
         throw new RuntimeException("Joint type not supported!");
      }
   }
   
   
   public static void setRandomJointVelocity(Joint joint, Random random)
   {
      if (joint instanceof FloatingJoint)
      {
         FloatingJoint floatingJoint = (FloatingJoint) joint;
       
         Tuple3DBasics velocity = RandomTools.generateRandomVector(random, 0.5);
         floatingJoint.setVelocity(velocity);
         
         Vector3D angularVelocityInBody = RandomTools.generateRandomVector(random, 2.0);
         floatingJoint.setAngularVelocityInBody(angularVelocityInBody);
      }
      else if (joint instanceof FloatingPlanarJoint)
      {
         FloatingPlanarJoint floatingJoint = (FloatingPlanarJoint) joint;
         
         double rotationalVelocity = RandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         floatingJoint.setRotationalVelocity(rotationalVelocity);

         Tuple2DBasics velocity = RandomTools.generateRandomVector2d(random, 0.5);
         floatingJoint.setCartesianVelocity(velocity);
      }
      else if (joint instanceof PinJoint)
      {
         PinJoint pinJoint = (PinJoint) joint;
         double rotationalVelocity = RandomTools.generateRandomDouble(random, -Math.PI, Math.PI);
         pinJoint.setQd(rotationalVelocity);
      }
      else if (joint instanceof SliderJoint)
      {
         SliderJoint sliderJoint = (SliderJoint) joint;
         
         double velocity = RandomTools.generateRandomDouble(random, -0.1, 0.1);
         sliderJoint.setQd(velocity);
      }
      else
      {
         throw new RuntimeException("Joint type not supported!");
      }
   }
   
   public static AxisAngle computeAxisAngleToAlignVectors(Vector3D vectorToRotate, Vector3D vectorToAlignTo)
   {
      Vector3D crossVector = new Vector3D();
      crossVector.cross(vectorToRotate, vectorToAlignTo);
      double crossVectorLength = crossVector.length();
      
      double rotationAngle = Math.asin(crossVectorLength/vectorToRotate.length()/vectorToAlignTo.length());
      
      if (vectorToRotate.dot(vectorToAlignTo) < 0.0)
      {
         rotationAngle = Math.PI - rotationAngle; 
      }
      
      if (Math.abs(crossVectorLength) > 1e-10)
      {
         crossVector.scale(1.0 / crossVectorLength);
         AxisAngle rotationAxisAngle = new AxisAngle(crossVector, rotationAngle);
         return rotationAxisAngle;
      }
      else
      {
         return new AxisAngle();
      }
   }

   public static Link generateRandomLink(Random random, String jointName, Vector3D axis)
   {
      Link link = new Link(jointName);
      Vector3D comOffset = RandomTools.generateRandomVector(random, 0.2);
      link.setComOffset(comOffset);
      double mass = (0.25 + random.nextDouble()) * 4.0;
      double radiusOfGyrationX = 0.04 + 0.1 * random.nextDouble();
      double radiusOfGyrationY = 0.04 + 0.1 * random.nextDouble();
      double radiusOfGyrationZ = 0.04 + 0.1 * random.nextDouble();
      link.setMassAndRadiiOfGyration(mass, radiusOfGyrationX, radiusOfGyrationY, radiusOfGyrationZ);
      
      Graphics3DObject linkGraphics = new Graphics3DObject();
      
      if (axis != null)
      {
         Vector3D zVector = new Vector3D(0.0, 0.0, 1.0);
         AxisAngle rotationAxisAngle = computeAxisAngleToAlignVectors(zVector, axis);
         linkGraphics.rotate(rotationAxisAngle);
         linkGraphics.addArrow(0.2, YoAppearance.Black(), YoAppearance.Blue());
      }
      else
      {
         linkGraphics.addSphere(0.1, YoAppearance.Red()); 
      }
      
      link.setLinkGraphics(linkGraphics);
            
      return link;
   }
   
   public static void main(String[] args)
   {
      Random random = new Random(1233L);
      boolean startWithFloatingJoint = true;
      Robot robot = generateRandomLinearChainRobot("randomRobot", startWithFloatingJoint , 10, random);
      robot.setGravity(new Vector3D(0.0, 0.0, -0.01));
      
//      setRandomJointPositions(robot, random);
      setRandomJointVelocities(robot, random);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.0001, 100);
      scs.setGroundVisible(false);
      scs.startOnAThread();
   }
}
