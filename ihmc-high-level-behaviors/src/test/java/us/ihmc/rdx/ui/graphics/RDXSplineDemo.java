package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXSplineGraphic;

public class RDXSplineDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXSplineGraphic spline;
   private double startTime;

   public RDXSplineDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));

            spline = new RDXSplineGraphic();
            spline.createStart(new Point3D(0.1, 0.1, 0.1), Color.BLUE);

            baseUI.getPrimaryScene().addRenderableProvider(spline);
            startTime = System.currentTimeMillis();
         }

         @Override
         public void render()
         {
            double time = System.currentTimeMillis();
            if ((time - startTime) / 1000 < 5)
               spline.createAdditionalPoint(new Point3D(0.1 + Math.sin(5 * (time - startTime) / 1000),
                                                        0.1 + (time - startTime) / 10000,
                                                        0.1 + (time - startTime) / 10000), Color.YELLOW);
            else if ((time - startTime) / 1000 < 6)
               spline.createEnd(Color.BLUE);
            else if ((time - startTime) / 1000 < 6.5)
            {
               spline.clear();
               spline.createStart(new Point3D(0.1, 0.1, 0.1), Color.BLUE);
               startTime = System.currentTimeMillis();
            }

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
      new RDXSplineDemo();
   }
}
