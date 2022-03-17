package us.ihmc.exampleSimulations.gravityComp;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.AffineTransformDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.GroundContactPointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class SimpleLegRobotDefinition extends RobotDefinition
{
   public static final String rootJointName = "rootJoint";
   public static final String hipRollJointName = "hipRoll";
   public static final String hipPitchJointName = "hipPitch";
   public static final String kneePitchJointName = "kneePitch";
   public static final String anklePitchJointName = "anklePitch";
   public static final String ankleRollJointName = "ankleRoll";

   public static final String rootBodyName = "rootBody";
   public static final String pelvisBodyName = "pelvis";
   public static final String hipRollBodyName = "hipRollBody";
   public static final String thighBodyName = "thigh";
   public static final String shinBodyName = "shin";
   public static final String anklePitchBodyName = "anklePitchBody";
   public static final String footBodyName = "foot";

   public static final double pelvisMass = 10.0;
   public static final double hipRollBodyMass = 0.25;
   public static final double thighMass = 5.0;
   public static final double shinMass = 5.0;
   public static final double anklePitchBodyMass = 0.1;
   public static final double footMass = 0.1;

   public static final Vector3DReadOnly pelvisSize = new Vector3D(0.3, 0.3, 0.3);
   public static final Vector3DReadOnly footSize = new Vector3D(0.4, 0.4, 0.025); // Large foot for stability
   public static final double thighLength = 0.45;
   public static final double thighRadius = 0.075;
   public static final double shinLength = 0.45;
   public static final double shinRadius = 0.06;

   public SimpleLegRobotDefinition()
   {
      super("leggy");

      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(rootJointName, new Vector3D(0, 0, 0.0));
      RevoluteJointDefinition hipRollJoint = new RevoluteJointDefinition(hipRollJointName, new Vector3D(0, 0, -0.5 * pelvisSize.getZ()), Axis3D.X);
      RevoluteJointDefinition hipPitchJoint = new RevoluteJointDefinition(hipPitchJointName, new Vector3D(0, 0, 0), Axis3D.Y);
      RevoluteJointDefinition kneePitchJoint = new RevoluteJointDefinition(kneePitchJointName, new Vector3D(0, 0, -thighLength), Axis3D.Y);
      RevoluteJointDefinition anklePitchJoint = new RevoluteJointDefinition(anklePitchJointName, new Vector3D(0, 0, -shinLength), Axis3D.Y);
      RevoluteJointDefinition ankleRollJoint = new RevoluteJointDefinition(ankleRollJointName, new Vector3D(0, 0, 0), Axis3D.X);

      RigidBodyDefinition rootBody = new RigidBodyDefinition(rootBodyName);

      RigidBodyDefinition pelvisBody = new RigidBodyDefinition(pelvisBodyName);
      pelvisBody.setMass(pelvisMass);
      pelvisBody.setMomentOfInertia(MomentOfInertiaFactory.solidBox(pelvisMass, pelvisSize));
      pelvisBody.getVisualDefinitions().add(new VisualDefinition(new Box3DDefinition(pelvisSize), new MaterialDefinition(ColorDefinitions.BlueViolet())));

      RigidBodyDefinition hipRollBody = new RigidBodyDefinition(hipRollBodyName);
      hipRollBody.setMass(hipRollBodyMass);
      hipRollBody.setMomentOfInertia(MomentOfInertiaFactory.solidSphere(hipRollBodyMass, 0.1));

      RigidBodyDefinition thighBody = new RigidBodyDefinition(thighBodyName);
      thighBody.setMass(thighMass);
      thighBody.setMomentOfInertia(MomentOfInertiaFactory.solidCylinder(thighMass, thighRadius, thighLength, Axis3D.Z));
      thighBody.setCenterOfMassOffset(0.0, 0.0, -0.5 * thighLength);
      thighBody.getVisualDefinitions()
               .add(new VisualDefinition(new Vector3D(0.0, 0.0, -thighLength),
                                         new Cylinder3DDefinition(thighLength, thighRadius),
                                         new MaterialDefinition(ColorDefinitions.DarkOliveGreen())));

      RigidBodyDefinition shinBody = new RigidBodyDefinition(shinBodyName);
      shinBody.setMass(shinMass);
      shinBody.setMomentOfInertia(MomentOfInertiaFactory.solidCylinder(shinMass, shinRadius, shinLength, Axis3D.Z));
      shinBody.setCenterOfMassOffset(0.0, 0.0, -0.5 * shinLength);
      shinBody.getVisualDefinitions()
              .add(new VisualDefinition(new AffineTransformDefinition(new Quaternion(), new Vector3D(0.0, 0.0, -shinLength)),
                                        new Cylinder3DDefinition(shinLength, shinRadius),
                                        new MaterialDefinition(ColorDefinitions.DarkOrchid())));

      RigidBodyDefinition anklePitchBody = new RigidBodyDefinition(anklePitchBodyName);
      anklePitchBody.setMass(anklePitchBodyMass);
      anklePitchBody.setMomentOfInertia(MomentOfInertiaFactory.solidSphere(anklePitchBodyMass, 0.1));

      RigidBodyDefinition footBody = new RigidBodyDefinition(footBodyName);
      footBody.setMass(footMass);
      footBody.setCenterOfMassOffset(0.25*footSize.getX(), 0.0, -0.5 * footSize.getZ());
      footBody.setMomentOfInertia(MomentOfInertiaFactory.solidBox(footMass, footSize));
      footBody.getVisualDefinitions()
              .add(new VisualDefinition(new Vector3D(0.25*footSize.getX(), 0, -0.5 * footSize.getZ()),
                                        new Box3DDefinition(footSize),
                                        new MaterialDefinition(ColorDefinitions.BlueViolet())));

      for (int i = 0; i < 4; i++)
      {
         double x = ((i & 1) == 0 ? -1.0 : 1.0) * 0.5 * footSize.getX();
         double y = ((i & 2) == 0 ? -1.0 : 1.0) * 0.5 * footSize.getY();
         double z = -footSize.getZ();
         Vector3D gcOffset = new Vector3D(x + 0.25*footSize.getX(), y, z);
         anklePitchJoint.addGroundContactPointDefinition(new GroundContactPointDefinition("gc_" + i, gcOffset));
         footBody.getVisualDefinitions().add(new VisualDefinition(gcOffset, new Sphere3DDefinition(0.01), new MaterialDefinition(ColorDefinitions.Orange())));
      }

      setRootBodyDefinition(rootBody);

      rootBody.addChildJoint(rootJoint);
      rootJoint.setSuccessor(pelvisBody);
      pelvisBody.addChildJoint(hipRollJoint);
      hipRollJoint.setSuccessor(hipRollBody);
      hipRollBody.addChildJoint(hipPitchJoint);
      hipPitchJoint.setSuccessor(thighBody);
      thighBody.addChildJoint(kneePitchJoint);
      kneePitchJoint.setSuccessor(shinBody);
      shinBody.addChildJoint(anklePitchJoint);
      anklePitchJoint.setSuccessor(anklePitchBody);
      anklePitchBody.addChildJoint(ankleRollJoint);
      ankleRollJoint.setSuccessor(footBody);
   }
}
