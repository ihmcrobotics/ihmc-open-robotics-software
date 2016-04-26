package us.ihmc.commonWalkingControlModules.controllerCore;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.tools.testing.JUnitTools;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

public class VirtualModelControllerTestHelper
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   public static final double toFootCenterX = 0.05042;
   public static final double toFootCenterY = 0.082;

   public static final double footWidth = 0.11;
   public static final double footBack = 0.085;
   public static final double footLength = 0.22;
   public static final double toeWidth = 0.085;
   public static final double ankleHeight = 0.0875;

   public static final double POUNDS = 1.0 / 2.2;    // Pound to Kg conversion.
   public static final double INCHES = 0.0254;    // Inch to Meter Conversion.

   public static final double PELVIS_HEIGHT = 1.0;
   public static final double PELVIS_RAD = 0.1;

   public static final double HIP_WIDTH = 0.2;

   public static final double HIP_DIFFERENTIAL_HEIGHT = 0.05;
   public static final double HIP_DIFFERENTIAL_WIDTH = 0.075;

   public static final double THIGH_LENGTH = 23.29 * INCHES;
   public static final double THIGH_RAD = 0.05;
   public static final double THIGH_MASS = 6.7 * POUNDS;

   public static final double SHIN_LENGTH = 9.1 * INCHES;
   public static final double SHIN_RAD = 0.03;
   public static final double SHIN_MASS = 1.0 * POUNDS;

   public static final double ANKLE_DIFFERENTIAL_HEIGHT = 0.025;
   public static final double ANKLE_DIFFERENTIAL_WIDTH = 0.0375;

   public static final double FOOT_LENGTH = 0.08;
   public static final double FOOT_COM_OFFSET = 3.0 * INCHES;
   public static final double FOOT_RAD = 0.05;
   public static final double FOOT_MASS = 3.0 * POUNDS;

   private final Random random = new Random(100L);

   public VirtualModelControllerTestHelper()
   {
   }

   public RobotLeg createRobotLeg(double gravity)
   {
      RobotLeg robotLeg = new RobotLeg("robotLeg");
      robotLeg.setGravity(gravity);
      HashMap<InverseDynamicsJoint, Joint> jointMap = new HashMap<>();

      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(), new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      FloatingJoint floatingJoint = new FloatingJoint("pelvis", new Vector3d(), robotLeg);
      robotLeg.addRootJoint(floatingJoint);
      SixDoFJoint rootJoint = new SixDoFJoint("pelvis", elevator, elevatorFrame);
      jointMap.put(rootJoint, floatingJoint);

      Link pelvisLink = pelvis();
      floatingJoint.setLink(pelvisLink);
      RigidBody pelvisBody = copyLinkAsRigidBody(pelvisLink, rootJoint, "pelvis");

      Vector3d hipYawOffset = new Vector3d(0.0, -HIP_WIDTH, 0.0);
      PinJoint hip_yaw = new PinJoint("hip_yaw", hipYawOffset, robotLeg, Axis.Z);
      hip_yaw.setQ(random.nextDouble());

      Link hip_differential = hip_differential();
      hip_yaw.setLink(hip_differential);
      floatingJoint.addJoint(hip_yaw);

      RevoluteJoint hipYaw = ScrewTools.addRevoluteJoint("hip_yaw", pelvisBody, hipYawOffset, Z);
      hipYaw.setQ(hip_yaw.getQ().getDoubleValue());
      RigidBody hipDifferentialBody = copyLinkAsRigidBody(hip_differential, hipYaw, "hip_differential");
      jointMap.put(hipYaw, hip_yaw);

      Vector3d hipRollOffset = new Vector3d();
      PinJoint hip_roll = new PinJoint("hip_roll", hipRollOffset, robotLeg, Axis.X);
      hip_roll.setQ(random.nextDouble());

      Link hip_differential2 = hip_differential();
      hip_roll.setLink(hip_differential2);
      hip_yaw.addJoint(hip_roll);

      RevoluteJoint hipRoll = ScrewTools.addRevoluteJoint("hip_roll", hipDifferentialBody, hipRollOffset, X);
      hipRoll.setQ(hip_roll.getQ().getDoubleValue());
      RigidBody hipDifferentialBody2 = copyLinkAsRigidBody(hip_differential2, hipRoll, "hip_differential");
      jointMap.put(hipRoll, hip_roll);

      Vector3d hipPitchOffset = new Vector3d();
      PinJoint hip_pitch = new PinJoint("hip_pitch", hipPitchOffset, robotLeg, Axis.Y);
      hip_pitch.setQ(random.nextDouble());

      Link thigh = thigh();
      hip_pitch.setLink(thigh);
      hip_roll.addJoint(hip_pitch);

      RevoluteJoint hipPitch = ScrewTools.addRevoluteJoint("hip_pitch", hipDifferentialBody2, hipPitchOffset, Y);
      hipPitch.setQ(hip_pitch.getQ().getDoubleValue());
      RigidBody thighBody = copyLinkAsRigidBody(thigh, hipPitch, "thigh");
      jointMap.put(hipPitch, hip_pitch);

      Vector3d kneePitchOffset = new Vector3d(0.0, 0.0, -THIGH_LENGTH);
      PinJoint knee_pitch = new PinJoint("knee_pitch", kneePitchOffset, robotLeg, Axis.Y);
      knee_pitch.setQ(random.nextDouble());

      Link shin = shin();
      knee_pitch.setLink(shin);
      hip_pitch.addJoint(knee_pitch);

      RevoluteJoint kneePitch = ScrewTools.addRevoluteJoint("knee_pitch", thighBody, kneePitchOffset, Y);
      kneePitch.setQ(knee_pitch.getQ().getDoubleValue());
      RigidBody shinBody = copyLinkAsRigidBody(shin, kneePitch, "shin");
      jointMap.put(kneePitch, knee_pitch);

      Vector3d anklePitchOffset = new Vector3d(0.0, 0.0, -SHIN_LENGTH);
      PinJoint ankle_pitch = new PinJoint("ankle_pitch", anklePitchOffset, robotLeg, Axis.Y);
      ankle_pitch.setQ(random.nextDouble());

      Link ankle_differential = ankle_differential();
      ankle_pitch.setLink(ankle_differential);
      knee_pitch.addJoint(ankle_pitch);

      RevoluteJoint anklePitch = ScrewTools.addRevoluteJoint("ankle_pitch", shinBody, anklePitchOffset, Y);
      anklePitch.setQ(ankle_pitch.getQ().getDoubleValue());
      RigidBody ankleDifferentialBody = copyLinkAsRigidBody(ankle_differential, anklePitch, "ankle_differential");
      jointMap.put(anklePitch, ankle_pitch);

      Vector3d ankleRollOffset = new Vector3d();
      PinJoint ankle_roll = new PinJoint("ankle_roll", ankleRollOffset, robotLeg, Axis.X);
      ankle_roll.setQ(random.nextDouble());

      Link foot = foot();
      ankle_roll.setLink(foot);
      ankle_pitch.addJoint(ankle_roll);

      RevoluteJoint ankleRoll = ScrewTools.addRevoluteJoint("ankle_roll", ankleDifferentialBody, ankleRollOffset, X);
      ankleRoll.setQ(ankle_roll.getQ().getDoubleValue());
      RigidBody footBody = copyLinkAsRigidBody(foot, ankleRoll, "foot");
      jointMap.put(ankleRoll, ankle_roll);

      RigidBodyTransform soleToAnkleFrame = TransformTools.createTranslationTransform(footLength / 2.0 - footBack + toFootCenterX,
            -toFootCenterY, -ankleHeight);
      ReferenceFrame soleFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("Sole",
            footBody.getBodyFixedFrame(), soleToAnkleFrame);

      robotLeg.setBase(pelvisBody);
      robotLeg.setEndEffector(footBody);
      robotLeg.setRootJoint(rootJoint);
      robotLeg.setJointMap(jointMap);
      robotLeg.setSoleFrame(soleFrame);


      return robotLeg;
   }

   private Link pelvis()
   {
      AppearanceDefinition pelvisAppearance = YoAppearance.Blue();

      Link ret = new Link("pelvis");

      ret.setMass(100.0);
      ret.setMomentOfInertia(1.0, 1.0, 1.0);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCoordinateSystem(1.0);
      linkGraphics.addCylinder(PELVIS_HEIGHT, PELVIS_RAD, pelvisAppearance);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link hip_differential()
   {
      Link ret = new Link("hip_differential");

      ret.setMass(0.1);
      ret.setMomentOfInertia(0.005, 0.005, 0.005);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCube(HIP_DIFFERENTIAL_WIDTH, HIP_DIFFERENTIAL_WIDTH, HIP_DIFFERENTIAL_HEIGHT);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   private Link thigh()
   {
      AppearanceDefinition thighApp = YoAppearance.Green();

      Link ret = new Link("thigh");

      ret.setMass(THIGH_MASS);    // 2.35);
      ret.setMomentOfInertia(0.0437, 0.0437, 0.0054);
      ret.setComOffset(0.0, 0.0, -THIGH_LENGTH / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(THIGH_LENGTH, THIGH_RAD, thighApp);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   private Link shin()
   {
      AppearanceDefinition shinApp = YoAppearance.Red();

      Link ret = new Link("shin");

      ret.setMass(SHIN_MASS);    // 0.864);
      ret.setMomentOfInertia(0.00429, 0.00429, 0.00106);
      ret.setComOffset(0.0, 0.0, -SHIN_LENGTH / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(SHIN_LENGTH, SHIN_RAD, shinApp);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link ankle_differential()
   {
      Link ret = new Link("ankle_differential");

      ret.setMass(0.1);
      ret.setMomentOfInertia(0.005, 0.005, 0.005);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCube(ANKLE_DIFFERENTIAL_WIDTH, ANKLE_DIFFERENTIAL_WIDTH, ANKLE_DIFFERENTIAL_HEIGHT);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link foot()
   {
      AppearanceDefinition footApp = YoAppearance.PlaneMaterial();

      Link ret = new Link("foot");

      ret.setMass(FOOT_MASS);    // 0.207);
      ret.setMomentOfInertia(0.00041, 0.00041, 0.00001689);
      ret.setComOffset(FOOT_COM_OFFSET, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(FOOT_LENGTH, FOOT_RAD, footApp);

      linkGraphics.translate(0.05, 0.0, FOOT_LENGTH);
      linkGraphics.addCube(0.02, 0.1, 0.1, YoAppearance.Black());

      linkGraphics.translate(-0.1, 0.0, 0.0);
      linkGraphics.addCube(0.02, 0.1, 0.1, YoAppearance.Black());

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }
   private RigidBody copyLinkAsRigidBody(Link link, InverseDynamicsJoint currentInverseDynamicsJoint, String bodyName)
   {
      Vector3d comOffset = new Vector3d();
      link.getComOffset(comOffset);
      Matrix3d momentOfInertia = new Matrix3d();
      link.getMomentOfInertia(momentOfInertia);
      ReferenceFrame nextFrame = createOffsetFrame(currentInverseDynamicsJoint, comOffset, bodyName);
      nextFrame.update();
      RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, link.getMass());
      RigidBody rigidBody = new RigidBody(bodyName, inertia, currentInverseDynamicsJoint);

      return rigidBody;
   }

   public static ReferenceFrame createOffsetFrame(InverseDynamicsJoint currentInverseDynamicsJoint, Vector3d offset, String frameName)
   {
      ReferenceFrame parentFrame = currentInverseDynamicsJoint.getFrameAfterJoint();
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setTranslationAndIdentityRotation(offset);
      ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);

      return beforeJointFrame;
   }

   public static void compareWrenches(Wrench inputWrench, Wrench outputWrench, DenseMatrix64F selectionMatrix)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      inputWrench.getExpressedInFrame().checkReferenceFrameMatch(outputWrench.getExpressedInFrame());

      DenseMatrix64F inputWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      DenseMatrix64F outputWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      DenseMatrix64F selectedValues = new DenseMatrix64F(Wrench.SIZE, 1);

      inputWrench.getMatrix(inputWrenchMatrix);
      outputWrench.getMatrix(outputWrenchMatrix);

      double epsilon = 1e-4;
      int taskSize = selectionMatrix.getNumRows();
      int colIndex = 0;
      for (int i = 0; i < taskSize; i++)
         for (int j = colIndex; j < Wrench.SIZE; j++)
            if (selectionMatrix.get(i, j) == 1)
               selectedValues.set(j, 0, 1);

      // only compare the selected values
      for (int i = 0; i < Wrench.SIZE; i++)
      {
         if (selectedValues.get(i, 0) == 1)
            Assert.assertEquals(inputWrenchMatrix.get(i, 0), outputWrenchMatrix.get(i, 0), epsilon);
         else
            Assert.assertNotEquals(inputWrenchMatrix.get(i, 0), outputWrenchMatrix.get(i, 0), epsilon);
      }
   }

   public static void compareWrenches(Wrench inputWrench, Wrench outputWrench)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      inputWrench.getExpressedInFrame().checkReferenceFrameMatch(outputWrench.getExpressedInFrame());

      double epsilon = 1e-4;
      JUnitTools.assertTuple3dEquals(inputWrench.getAngularPartCopy(), outputWrench.getAngularPartCopy(), epsilon);
      JUnitTools.assertTuple3dEquals(inputWrench.getLinearPartCopy(), outputWrench.getLinearPartCopy(), epsilon);
   }

   public class RobotLeg extends Robot
   {
      private RigidBody base;
      private RigidBody endEffector;
      private ReferenceFrame soleFrame;
      private InverseDynamicsJoint rootJoint;
      private HashMap<InverseDynamicsJoint, Joint> jointMap;

      public RobotLeg(String name)
      {
         super(name);
      }

      public void setBase(RigidBody base)
      {
         this.base = base;
      }

      public void setEndEffector(RigidBody endEffector)
      {
         this.endEffector = endEffector;
      }

      public void setSoleFrame(ReferenceFrame soleFrame)
      {
         this.soleFrame = soleFrame;
      }

      public void setRootJoint(InverseDynamicsJoint rootJoint)
      {
         this.rootJoint = rootJoint;
      }

      public void setJointMap(HashMap<InverseDynamicsJoint, Joint> jointMap)
      {
         this.jointMap = jointMap;
      }

      public RigidBody getBase()
      {
         return base;
      }

      public RigidBody getEndEffector()
      {
         return endEffector;
      }

      public ReferenceFrame getSoleFrame()
      {
         return soleFrame;
      }

      public InverseDynamicsJoint getRootJoint()
      {
         return rootJoint;
      }

      public Map<InverseDynamicsJoint, Joint> getJointMap()
      {
         return jointMap;
      }
   }
}
