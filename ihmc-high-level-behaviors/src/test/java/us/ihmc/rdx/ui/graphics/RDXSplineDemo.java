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
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/test/resources");
   private RDXSplineGraphic spline;

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
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();

            spline.createStart(new Point3D(0.5,0.5,0.5), Color.BLUE);
            spline.createAdditionalPoint(new Point3D(0.7,0.6,1), Color.YELLOW);
            spline.createAdditionalPoint(new Point3D(1,1,1), Color.YELLOW);
            spline.createAdditionalPoint(new Point3D(1.1,1.1,1.1), Color.YELLOW);
            spline.createEnd(Color.BLUE);
            baseUI.getPrimaryScene().addRenderableProvider(spline);
            spline.clear();
            spline.createStart(new Point3D(-0.5,-0.5,-0.5), Color.BLUE);
            spline.createAdditionalPoint(new Point3D(-0.7,-0.6,-1), Color.YELLOW);
            spline.createAdditionalPoint(new Point3D(-1,-1,-1), Color.YELLOW);
            spline.createAdditionalPoint(new Point3D(-1.1,-1.1,-1.1), Color.YELLOW);
            spline.createEnd(Color.BLUE);

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
