package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXColorTexturesDemo
{
   public RDXColorTexturesDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            for (int i = 0; i < 100; i++)
            {
               for (int j = 0; j < 100; j++)
               {
                  MutableInt mutableI = new MutableInt(i);
                  MutableInt mutableJ = new MutableInt(j);
                  ModelInstance coloredSpheres = RDXModelBuilder.buildModelInstance(meshBuilder ->
                  {
                     Color color = new Color();
                     float hue = mutableI.getValue() / 100.0f * 360.0f; // hue is all that matters to a ColorMeshBuilder
                     color.fromHsv(hue, 1.0f, 1.0f);
                     meshBuilder.addSphere(0.01, new Point3D(mutableI.getValue() / 50.0, mutableJ.getValue() / 50.0, 0.0), color);
                  }, "ColoredSpheres");
                  baseUI.getPrimaryScene().addModelInstance(coloredSpheres);

                  coloredSpheres.materials.get(0).set(new BlendingAttribute(true, mutableJ.getValue() / 100.0f));
               }
            }

            ModelInstance coloredSphere = RDXModelBuilder.buildModelInstance(meshBuilder ->
            {
               meshBuilder.addSphere(0.5, new Point3D(4.0, 4.0, 0.0), LibGDXTools.toLibGDX(YoAppearance.LightSkyBlue()));
            }, "ColoredSphere");
            baseUI.getPrimaryScene().addModelInstance(coloredSphere);

            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());
         }

         @Override
         public void render()
         {
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

   public static void main(String[] args)
   {
      new RDXColorTexturesDemo();
   }
}