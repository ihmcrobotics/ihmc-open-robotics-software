package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerrainObject;
import us.ihmc.simulationConstructionSetTools.util.ground.MeshTerrainObjectParameters;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class MeshTerrainObjectFactory
{
   public static MeshTerrainObject createWalkwayObject()
   {
      return createWalkwayObject(new Vector3D(), new Quaternion(-Math.PI / 2.0, 0.0, 0.0));
   }

   public static MeshTerrainObject createWalkwayObject(Tuple3DReadOnly translation, Orientation3DReadOnly orientation)
   {
      MeshTerrainObjectParameters parameters = new MeshTerrainObjectParameters("models/walkway/walkway.obj");
      RigidBodyTransform configuration = new RigidBodyTransform(orientation, translation);

      return new MeshTerrainObject(parameters, configuration);
   }

   public static MeshTerrainObject createRoughTerrainObject()
   {
      return createRoughTerrainObject(new Vector3D(), new Quaternion(-Math.PI / 2.0, 0.0, 0.0));
   }

   public static MeshTerrainObject createRoughTerrainObject(Tuple3DReadOnly translation, Orientation3DReadOnly orientation)
   {
      RigidBodyTransform configuration = new RigidBodyTransform(orientation, translation);

      MeshTerrainObjectParameters parameters = new MeshTerrainObjectParameters("models/roughTerrain/roughTerrain.obj");
      parameters.setMaxNoOfHulls(1024);
      parameters.setMaxNoOfVertices(256);
      parameters.setVoxelResolution(500000);
      parameters.setShowDecomposedMeshGraphics(false);
      parameters.setShowUndecomposedMeshGraphics(true);

      return new MeshTerrainObject(parameters, configuration);
   }

   public static MeshTerrainObject createWorkPlatformObject()
   {
      return createWorkPlatformObject(new Vector3D(), new Quaternion(-Math.PI / 2.0, 0.0, 0.0));
   }

   public static MeshTerrainObject createWorkPlatformObject(Tuple3DReadOnly translation, Orientation3DReadOnly orientation)
   {
      RigidBodyTransform configuration = new RigidBodyTransform(orientation, translation);

      MeshTerrainObjectParameters parameters = new MeshTerrainObjectParameters("models/workPlatform/workPlatform.obj");
      parameters.setMaxNoOfHulls(100);
      parameters.setMaxNoOfVertices(20);
      parameters.setVoxelResolution(8000000);
      parameters.setShowDecomposedMeshGraphics(false);
      parameters.setShowUndecomposedMeshGraphics(true);

      return new MeshTerrainObject(parameters, configuration);
   }

   public static TerrainObject3D createFlatGround()
   {
      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/ground2.png");
      double sizeXY = 50.0;
      Box3D box = new Box3D(sizeXY, sizeXY, 1.0);
      box.getPosition().setX(0.0);
      box.getPosition().setY(0.0);
      box.getPosition().setZ(-0.5);

      return new RotatableBoxTerrainObject(box, texture);
   }
}
