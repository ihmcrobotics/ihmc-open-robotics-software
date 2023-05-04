package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.visualizers.RDXEdgeDefinedShapeGraphic;

public class RDXEdgeDefinedPolygonDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXEdgeDefinedShapeGraphic shape;

   public RDXEdgeDefinedPolygonDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));

            Point3D[] spline1 = new Point3D[] {new Point3D(0.1, 0.6, 0.1), new Point3D(0.2, 0.5, 0.1), new Point3D(0.3, 0.5, 0.2)};
            Point3D[] spline2 = new Point3D[] {new Point3D(0.1, 0.1, 0.1), new Point3D(0.3, 0.3, 0.2), new Point3D(0.5, 0.3, 0.3)};
            Point3D[] spline3 = new Point3D[] {new Point3D(0.1, 0, 0.15), new Point3D(0.2, -0.35, 0.2), new Point3D(0.5, -0.5, 0.3)};
            Point3D[] spline4 = new Point3D[] {new Point3D(0.1, -0.1, 0.4), new Point3D(0.2, -0.35, 0.45), new Point3D(0.5, -0.5, 0.5)};
            Point3D[] spline5 = new Point3D[] {new Point3D(0.1, 0, 0.4), new Point3D(0.3, 0.35, 0.45), new Point3D(0.5, 0.5, 0.5)};
            Point3D[] spline6 = new Point3D[] {new Point3D(0.1, 0.1, 0.4), new Point3D(0.3, 0.45, 0.45), new Point3D(0.5, 0.6, 0.5)};

            Point3D[][] edges = new Point3D[][] {spline1, spline2, spline3, spline4, spline5, spline6};
            shape = new RDXEdgeDefinedShapeGraphic(edges, Color.GREEN, Color.FOREST, 0.5f);
            shape.createMainShape();
            shape.update();
            shape.closeShape();
            shape.generateMesh();

            baseUI.getPrimaryScene().addRenderableProvider(shape);
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
      new RDXEdgeDefinedPolygonDemo();
   }
}
