package us.ihmc.behaviors.simulation;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.VisualDefinition;

/**
 * A can of alphabet soup for picking up.
 */
public class CanOfSoupDefinition extends RobotDefinition
{
   private SixDoFJointState initialSixDoFState;

   public CanOfSoupDefinition(String name)
   {
      super(name);
   }

   public void build()
   {
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition(getName() + "_canOfSoupRootBody");

      SixDoFJointDefinition rootJointDefinition = new SixDoFJointDefinition(getName() + "_canOfSoupRootJoint");
      rootBodyDefinition.addChildJoint(rootJointDefinition);
      initialSixDoFState = new SixDoFJointState(new YawPitchRoll(), new Point3D());
      initialSixDoFState.setVelocity(new Vector3D(), new Vector3D());
      rootJointDefinition.setInitialJointState(initialSixDoFState);

      double length = RigidBodySceneObjectDefinitions.CAN_OF_SOUP_HEIGHT;
      double radius = RigidBodySceneObjectDefinitions.CAN_OF_SOUP_RADIUS;
      RigidBodyDefinition canOfSoupBody = new RigidBodyDefinition(getName() + "_canOfSoupBody");
      VisualDefinition modelVisualDefinition = new VisualDefinition();
      ModelFileGeometryDefinition geometryDefinition = new ModelFileGeometryDefinition("environmentObjects/canOfSoup/CanOfSoup.g3dj");
      modelVisualDefinition.setGeometryDefinition(geometryDefinition);
      modelVisualDefinition.getOriginPose().getTranslation().setZ(length / 2.0);
      canOfSoupBody.addVisualDefinition(modelVisualDefinition);

      CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition(new Cylinder3DDefinition(length, radius));
      collisionShapeDefinition.getOriginPose().getTranslation().setZ(length / 2.0);
      canOfSoupBody.addCollisionShapeDefinition(collisionShapeDefinition);

      canOfSoupBody.setMass(0.2);
      double radiusOfGyrationPercent = 0.8;
      canOfSoupBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(canOfSoupBody.getMass(),
                                                                                         radiusOfGyrationPercent * length,
                                                                                         radiusOfGyrationPercent * length,
                                                                                         radiusOfGyrationPercent * length));
      Point3D centerOfMassOffset = new Point3D(0.0, 0.0, length / 2.0);
      canOfSoupBody.getInertiaPose().getTranslation().set(centerOfMassOffset);
      canOfSoupBody.getInertiaPose().getRotation().setToZero();
      rootJointDefinition.setSuccessor(canOfSoupBody);

      setRootBodyDefinition(rootBodyDefinition);
   }

   public SixDoFJointState getInitialSixDoFState()
   {
      return initialSixDoFState;
   }
}
