package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.mesh.MeshDataGeneratorMissing;
import us.ihmc.rdx.mesh.RDXMeshDataInterpreter;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.EuclidCoreMissingTools;

public class RDXSphericalImageProjectionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXIconTexture imageTexture;
   private ModelInstance modelInstance;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDouble sphereRadius = new ImDouble(20.0);
   private final ImInt sphereLatitudeVertices = new ImInt(100);
   private final ImInt sphereLongitudeVertices = new ImInt(100);
   private final ImBoolean syncProjectionScales = new ImBoolean(true);
   private final ImDouble focalLengthX = new ImDouble(0.45);
   private final ImDouble focalLengthY = new ImDouble(0.45);
   private final ImDouble principlePointX = new ImDouble(0.0);
   private final ImDouble principlePointY = new ImDouble(0.0);
   private Model model;
   private final Vector3D vertexRay = new Vector3D();

   public RDXSphericalImageProjectionDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            imageTexture = new RDXIconTexture("/images/blackflytest.jpg");

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Projection", () ->
            {
               ImGuiTools.volatileInputDouble(labels.get("Sphere radius"), sphereRadius);
               ImGuiTools.volatileInputInt(labels.get("Sphere latitude vertices"), sphereLatitudeVertices);
               ImGuiTools.volatileInputInt(labels.get("Sphere longitude vertices"), sphereLongitudeVertices);
               ImGui.checkbox(labels.get("Sync projection scales"), syncProjectionScales);
               ImGuiTools.sliderDouble(labels.get("Projection scale X"), focalLengthX, 0.01, 2.0);
               if (syncProjectionScales.get())
               {
                  ImGui.beginDisabled();
                  focalLengthY.set(focalLengthX.get());
               }
               ImGuiTools.sliderDouble(labels.get("Projection scale Y"), focalLengthY, 0.01, 2.0);
               if (syncProjectionScales.get())
               {
                  ImGui.endDisabled();
               }
               ImGuiTools.volatileInputDouble(labels.get("Priciple point X (Cx)"), principlePointX);
               ImGuiTools.volatileInputDouble(labels.get("Priciple point Y (Cy)"), principlePointY);
            });
         }

         @Override
         public void render()
         {
            MeshDataHolder sphereMeshDataHolder = MeshDataGeneratorMissing.InvertedSphere(sphereRadius.get(),
                                                                                          sphereLatitudeVertices.get(),
                                                                                          sphereLongitudeVertices.get());

            for (int i = 0; i < sphereMeshDataHolder.getVertices().length; i++)
            {
               Point3D32 vertex = sphereMeshDataHolder.getVertices()[i];
               TexCoord2f texturePoint = sphereMeshDataHolder.getTexturePoints()[i];

               // TODO: Magical function
               vertexRay.set(vertex);

               double angleOfIncidence = EuclidCoreMissingTools.angleFromFirstToSecondVector3D(Axis3D.X, vertexRay);
               double azimuthalAngle = Math.atan2(-vertex.getZ(), -vertex.getY());

               double imageX = principlePointX.get() + focalLengthX.get() * angleOfIncidence * Math.cos(azimuthalAngle);
               double imageY = principlePointY.get() + focalLengthY.get() * angleOfIncidence * Math.sin(azimuthalAngle);

               texturePoint.setX(imageX + 0.5);
               texturePoint.setY(imageY + 0.5);
            }

            ModelBuilder modelBuilder = new ModelBuilder();

            modelBuilder.begin();
            Mesh mesh = RDXMeshDataInterpreter.interpretMeshData(sphereMeshDataHolder);


            MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
            Material material = new Material();
            material.set(TextureAttribute.createDiffuse(imageTexture.getTexture()));
            material.set(ColorAttribute.createDiffuse(Color.WHITE));
            modelBuilder.part(meshPart, material);

            if (model != null)
               model.dispose();

            model = modelBuilder.end();

            modelInstance = new ModelInstance(model);

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            modelInstance.getRenderables(renderables, pool);
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
      new RDXSphericalImageProjectionDemo();
   }
}
