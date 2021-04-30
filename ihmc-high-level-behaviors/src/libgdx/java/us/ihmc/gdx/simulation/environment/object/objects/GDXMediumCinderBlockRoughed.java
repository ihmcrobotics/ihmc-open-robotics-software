package us.ihmc.gdx.simulation.environment.object.objects;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXModelPrimitives;

import java.util.concurrent.atomic.AtomicInteger;

public class GDXMediumCinderBlockRoughed extends GDXEnvironmentObject
{
   private static final AtomicInteger INDEX = new AtomicInteger();

   public GDXMediumCinderBlockRoughed()
   {
      Model realisticModel = GDXModelLoader.loadG3DModel("MediumCinderBlockRough.g3dj");

      double height = 0.141535;
      double width = 0.188522;
      double length = 0.393001;
      Box3D collisionBox = new Box3D(length, width, height);

      Sphere3D boundingSphere = new Sphere3D(collisionBox.getSize().length() / 2.0);

      Model collisionModel = GDXModelPrimitives.buildModel(meshBuilder ->
      {
         meshBuilder.addBox((float) length, (float) width, (float) height, Color.RED);
         meshBuilder.addBox((float) width, (float) height, (float) length, Color.GREEN);
         meshBuilder.addBox((float) height, (float) length, (float) width, Color.BLUE);
         meshBuilder.addSphere((float) width, Color.CORAL);
      }, "collisionModel" + INDEX.getAndIncrement());

      create(realisticModel, boundingSphere, collisionBox, collisionBox::isPointInside, collisionModel);
   }

   @Override
   public GDXMediumCinderBlockRoughed duplicate()
   {
      return new GDXMediumCinderBlockRoughed();
   }
}
