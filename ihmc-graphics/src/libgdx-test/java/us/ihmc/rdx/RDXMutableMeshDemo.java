package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.mesh.RDXMutableMultiLineModel;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.ArrayList;

public class RDXMutableMeshDemo
{
   private RDXMutableMultiLineModel mutableMultiLineModel;
   private final ArrayList<Point3D> points = new ArrayList<>();
   private double amount = 0.0;

   public RDXMutableMeshDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            mutableMultiLineModel = new RDXMutableMultiLineModel();

            baseUI.getPrimaryScene().addRenderableProvider(mutableMultiLineModel::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Panel", () ->
            {
               if (ImGui.button("Add Point"))
               {
                  points.add(new Point3D(amount, amount, amount++));
               }
               if (ImGui.button("Clear Points"))
               {
                  points.clear();
                  amount = 0.0;
               }
            });
         }

         @Override
         public void render()
         {
            mutableMultiLineModel.update(points, 0.1, Color.WHITE);

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
      new RDXMutableMeshDemo();
   }
}
