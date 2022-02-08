package us.ihmc.gdx.simulation.impulseBased;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.simulation.BoxRobotDefinition;
import us.ihmc.gdx.simulation.SlopeGroundDefinition;
import us.ihmc.gdx.simulation.scs2.GDXSCS2PhysicsSimulator;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.*;
import us.ihmc.scs2.simulation.parameters.ContactParameters;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class GDXSCS2ImpulseBasedPhysicsTumblingBlocksDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private final GDXSCS2PhysicsSimulator physicsSimulator = new GDXSCS2PhysicsSimulator();

   public GDXSCS2ImpulseBasedPhysicsTumblingBlocksDemo()
   {
      ContactParameters contactParameters = new ContactParameters();
      contactParameters.setMinimumPenetration(5.0e-5);
      contactParameters.setCoefficientOfFriction(0.7);
      contactParameters.setCoefficientOfRestitution(0.3);
      contactParameters.setRestitutionThreshold(0.15);
      contactParameters.setErrorReductionParameter(0.01);

      int numberOfBlocks = 6;
      Random random = new Random(1886L);

      List<RobotDefinition> robotDefinitions = new ArrayList<>();

      double boxSizeX = 0.1;
      double boxSizeY = 0.08;
      double boxSizeZ = 0.1;
      double mass = 0.2;
      double x = 0.0;
      double y = 0.0;

      for (int i = 0; i < numberOfBlocks; i++)
      {
         ColorDefinition appearance = ColorDefinition.rgb(random.nextInt());
         RobotDefinition boxRobot = newBoxRobot("Block" + i, boxSizeX, boxSizeY, boxSizeZ, mass, 0.5, appearance);
         robotDefinitions.add(boxRobot);

         boxRobot.getRigidBodyDefinition("Block" + i + "RigidBody")
                 .addCollisionShapeDefinition(new CollisionShapeDefinition(new Box3DDefinition(boxSizeX, boxSizeY, boxSizeZ)));

         x += 0.02;
         y = 0.0;
         double z = boxSizeZ * 1.05 * (i + 1.0);

         double yaw = 0.0;
         double pitch = RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);
         double roll = RandomNumbers.nextDouble(random, -Math.PI / 90.0, Math.PI / 90.0);

         boxRobot.getRootJointDefinitions().get(0).setInitialJointState(new SixDoFJointState(new YawPitchRoll(yaw, pitch, roll), new Point3D(x, y, z)));
      }

      RigidBodyTransform terrainPose = new RigidBodyTransform();
      terrainPose.getTranslation().subZ(0.05);
      GeometryDefinition terrainGeometry = new Box3DDefinition(1000, 1000, 0.1);
      TerrainObjectDefinition terrain = new TerrainObjectDefinition(new VisualDefinition(terrainPose,
                                                                                         terrainGeometry,
                                                                                         new MaterialDefinition(ColorDefinitions.DarkGrey())),
                                                                    new CollisionShapeDefinition(terrainPose, terrainGeometry));


      robotDefinitions.forEach(physicsSimulator::addRobot);
//      physicsSimulator.addTerrainObject(terrain);

      BoxRobotDefinition boxRobot = new BoxRobotDefinition();
      SixDoFJointState initialJointState = new SixDoFJointState();
      initialJointState.setConfiguration(new Pose3D(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
      boxRobot.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);

      SlopeGroundDefinition slopeTerrain = new SlopeGroundDefinition();

//      physicsSimulator.addRobot(boxRobot);
      physicsSimulator.addTerrainObject(slopeTerrain);

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            physicsSimulator.create(baseUI);
            baseUI.getImGuiPanelManager().addPanel(physicsSimulator.getControlPanel());
         }

         @Override
         public void render()
         {
            physicsSimulator.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   RobotDefinition newBoxRobot(String name, double sizeX, double sizeY, double sizeZ, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RobotDefinition robotDefinition = new RobotDefinition(name);

      RigidBodyDefinition rootBody = new RigidBodyDefinition(name + "RootBody");
      SixDoFJointDefinition rootJoint = new SixDoFJointDefinition(name);
      rootBody.addChildJoint(rootJoint);
      rootJoint.setSuccessor(newBoxRigidBody(name + "RigidBody", sizeX, sizeY, sizeZ, mass, radiusOfGyrationPercent, color));
      robotDefinition.setRootBodyDefinition(rootBody);

      return robotDefinition;
   }

   RigidBodyDefinition newBoxRigidBody(String rigidBodyName, double sizeX, double sizeY, double sizeZ, double mass,
                                       double radiusOfGyrationPercent, ColorDefinition color)
   {
      return newBoxRigidBody(rigidBodyName, sizeX, sizeY, sizeZ, mass, radiusOfGyrationPercent, null, color);
   }

   RigidBodyDefinition newBoxRigidBody(String rigidBodyName, double sizeX, double sizeY, double sizeZ, double mass,
                                       double radiusOfGyrationPercent, Vector3DReadOnly offsetFromParentJoint, ColorDefinition color)
   {
      RigidBodyDefinition rigidBody = new RigidBodyDefinition(rigidBodyName);
      rigidBody.setMass(mass);
      rigidBody.setMomentOfInertia(MomentOfInertiaFactory.fromMassAndRadiiOfGyration(mass,
                                                                                     radiusOfGyrationPercent * sizeX,
                                                                                     radiusOfGyrationPercent * sizeY,
                                                                                     radiusOfGyrationPercent * sizeZ));
      if (offsetFromParentJoint != null)
         rigidBody.setCenterOfMassOffset(offsetFromParentJoint);

      VisualDefinitionFactory factory = new VisualDefinitionFactory();
      if (offsetFromParentJoint != null)
         factory.appendTranslation(offsetFromParentJoint);
      factory.addBox(sizeX, sizeY, sizeZ, new MaterialDefinition(color));
      rigidBody.addVisualDefinitions(factory.getVisualDefinitions());
      return rigidBody;
   }

   public static void main(String[] args)
   {
      new GDXSCS2ImpulseBasedPhysicsTumblingBlocksDemo();
   }
}
