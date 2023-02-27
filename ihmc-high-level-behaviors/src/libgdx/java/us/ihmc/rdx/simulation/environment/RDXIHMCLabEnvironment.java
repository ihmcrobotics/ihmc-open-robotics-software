package us.ihmc.rdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.tools.EuclidModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.tools.LibGDXTools;

import java.util.ArrayList;

public class RDXIHMCLabEnvironment implements RenderableProvider
{
   private final ArrayList<RenderableProvider> modelInstances = new ArrayList<>();
   private Model labFloorModel;
   private Model smallCinderBlockModel;
   private Model mediumCinderBlockModel;
   private Model largeCinderBlockModel;
   private Model doorModel;
   private Model doorFrameModel;

   public void create()
   {
      labFloorModel = RDXModelLoader.load("labFloor/LabFloor.g3dj");
      smallCinderBlockModel = RDXModelLoader.load("SmallCinderBlockRough.g3dj");
      mediumCinderBlockModel = RDXModelLoader.load("MediumCinderBlockRough.g3dj");
      largeCinderBlockModel = RDXModelLoader.load("LargeCinderBlockRough.g3dj");
      doorModel = RDXModelLoader.load("DoorOnly.g3dj");
      doorFrameModel = RDXModelLoader.load("DoorFrame.g3dj");

      Pose3D pose = new Pose3D();
      RigidBodyTransform transform = new RigidBodyTransform();

      ModelInstance floorInstance = new ModelInstance(labFloorModel);
      modelInstances.add(floorInstance);
      pose.set(new Point3D(0.0f, 0.0f, 0.0f), new YawPitchRoll(0.0, 0.0, Math.toRadians(90.0)));
      pose.get(transform);
      LibGDXTools.toLibGDX(transform, floorInstance.transform);

//      ModelInstance smallCinderBlockInstance = new ModelInstance(smallCinderBlockModel);
//      modelInstances.add(smallCinderBlockInstance);
//      smallCinderBlockInstance.transform.setTranslation(0.3f, 0.3f, 0.19f);

//      ModelInstance mediumCinderBlockRoughInstance = new ModelInstance(mediumCinderBlockModel);
//      modelInstances.add(mediumCinderBlockRoughInstance);
//
//      ModelInstance largeCinderBlockInstance = new ModelInstance(largeCinderBlockModel);
//      modelInstances.add(largeCinderBlockInstance);

//      ModelInstance doorInstance = new ModelInstance(doorModel);
//      modelInstances.add(doorInstance);
//
//      ModelInstance doorFrameInstance = new ModelInstance(doorFrameModel);
//      modelInstances.add(doorFrameInstance);


//      pose.set(new Point3D(0.9f, 0.9f, 0.0f), new YawPitchRoll(0.0, 0.0, Math.toRadians(90.0)));
//      pose.get(transform);
//      LibGDXTools.toLibGDX(transform, mediumCinderBlockRoughInstance.transform);
//
//      largeCinderBlockInstance.transform.setTranslation(0.6f, 0.6f, 0.19f);

//      pose.set(new Point3D(2.0, 0.46, 0.0), new YawPitchRoll(Math.toRadians(-45.0), 0.0, Math.toRadians(90.0)));
//      pose.get(transform);
//      LibGDXTools.toLibGDX(transform, doorInstance.transform);
//
//      pose.set(new Point3D(2.0, 0.0, 0.0), new YawPitchRoll(0.0, 0.0, Math.toRadians(90.0)));
//      pose.get(transform);
//      LibGDXTools.toLibGDX(transform, doorFrameInstance.transform);
   }

   public EuclidModelInstance addLargeCinderBlock()
   {
      Pose3D offset = new Pose3D(0.20, 0.0, 0.0955, 0.0, Math.toRadians(90.0), 0.0);
      EuclidModelInstance modelInstance = new EuclidModelInstance(largeCinderBlockModel, offset);
      modelInstances.add(modelInstance);
      return modelInstance;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RenderableProvider modelInstance : modelInstances)
      {
         modelInstance.getRenderables(renderables, pool);
      }
   }
}
