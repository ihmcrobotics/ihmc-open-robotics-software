package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class GDXColorTexturesDemo
{
   public GDXColorTexturesDemo()
   {
      GDX3DSceneManager sceneManager = new GDX3DSceneManager();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);

            for (int i = 0; i < 100; i++)
            {
               for (int j = 0; j < 100; j++)
               {
                  MutableInt mutableI = new MutableInt(i);
                  MutableInt mutableJ = new MutableInt(j);
                  ModelInstance coloredSpheres = GDXModelPrimitives.buildModelInstance(meshBuilder ->
                  {
                     Color color = new Color();
                     float hue = mutableI.getValue() / 100.0f * 360.0f; // hue is all that matters to a ColorMeshBuilder
                     color.fromHsv(hue, 1.0f, 1.0f);
                     meshBuilder.addSphere(0.01, new Point3D(mutableI.getValue() / 50.0, mutableJ.getValue() / 50.0, 0.0), color);
                  }, "ColoredSpheres");
                  sceneManager.addModelInstance(coloredSpheres);

                  coloredSpheres.materials.get(0).set(new BlendingAttribute(true, mutableJ.getValue() / 100.0f));
               }
            }

            ModelInstance coloredSphere = GDXModelPrimitives.buildModelInstance(meshBuilder ->
            {
               meshBuilder.addSphere(0.5, new Point3D(4.0, 4.0, 0.0), GDXTools.toGDX(YoAppearance.LightSkyBlue()));
            }, "ColoredSphere");
            sceneManager.addModelInstance(coloredSphere);

            sceneManager.addModelInstance(new BoxesDemoModel().newInstance());
         }

         @Override
         public void render()
         {
            sceneManager.setViewportBoundsToWindow();
            sceneManager.render();
         }
      }, "GDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDXColorTexturesDemo();
   }
}