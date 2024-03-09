package us.ihmc.behaviors.simulation;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.*;

public class BoxDefinition extends RobotDefinition
{
   private SixDoFJointState initialSixDoFState;

   public BoxDefinition()
   {
      super("Box");
   }

   public void build(double mass, Vector3D size)
   {
      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition("boxRootJoint");

      initialSixDoFState = new SixDoFJointState(new YawPitchRoll(), new Point3D());
      initialSixDoFState.setVelocity(new Vector3D(), new Vector3D());
      rootJointDefinition.setInitialJointState(initialSixDoFState);

      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("boxRootBody");
      rootBodyDefinition.addChildJoint(rootJointDefinition);

      RigidBodyDefinition boxBody = new RigidBodyDefinition("boxBody");

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      factory.addBox(size.getX(), size.getY(), size.getZ(), new MaterialDefinition(ColorDefinitions.Red()));
      boxBody.addVisualDefinitions(factory.getVisualDefinitions());

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(new Box3DDefinition(size.getX(), size.getY(), size.getZ()));
      boxBody.addCollisionShapeDefinition(collisionShapeDefinition);

      boxBody.setMass(mass);
      double radiusOfGyrationPercent = 0.8;
      boxBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(boxBody.getMass(),
                                                                                   radiusOfGyrationPercent * size.getX(),
                                                                                   radiusOfGyrationPercent * size.getY(),
                                                                                   radiusOfGyrationPercent * size.getZ()));
      Point3D centerOfMassOffset = new Point3D(0.0, 0.0, size.getZ() / 2.0);
      boxBody.getInertiaPose().getTranslation().set(centerOfMassOffset);
      boxBody.getInertiaPose().getRotation().setToZero();

      rootJointDefinition.setSuccessor(boxBody);

      setRootBodyDefinition(rootBodyDefinition);
   }

   public SixDoFJointState getInitialSixDoFState()
   {
      return initialSixDoFState;
   }
}