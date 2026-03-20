package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXStabilityRegionGraphic implements RenderableProvider
{
   public static final Color NOMINAL_POLYGON_GRAPHIC_COLOR = Color.valueOf("CC1818");

   private final ConvexPolygon2D supportRegion = new ConvexPolygon2D();

   private final ModelBuilder modelBuilder = new ModelBuilder();
   private final RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
   private final MidFrameZUpFrame midFeetZUpFrame;
   private final RigidBodyTransform transform = new RigidBodyTransform();

   private ModelInstance modelInstance;
   private Model lastModel;

   public RDXStabilityRegionGraphic(FullHumanoidRobotModel ghostFullRobotModel)
   {
      this.midFeetZUpFrame = new MidFrameZUpFrame("midFeetZUpGhost",
                                                  ReferenceFrame.getWorldFrame(),
                                                  ghostFullRobotModel.getSoleFrame(RobotSide.LEFT),
                                                  ghostFullRobotModel.getSoleFrame(RobotSide.RIGHT));
   }

   public void update(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      if (capturabilityBasedStatus == null)
         return;

      meshBuilder.clear();

      // Update frames and compute mid-feet height
      midFeetZUpFrame.update();

      FramePoint3D midFoot = new FramePoint3D(midFeetZUpFrame);
      midFoot.changeFrame(ReferenceFrame.getWorldFrame());
      double footZ = midFoot.getZ();

      // Update region graphic
      Object<Point3D> supportRegion = capturabilityBasedStatus.getMultiContactRegion();
      if (supportRegion.size() >= 3)
      {
         this.supportRegion.clear();

         for (int i = 0; i < supportRegion.size(); i++)
         {
            this.supportRegion.addVertex(supportRegion.get(i));
         }

         this.supportRegion.update();

         if (this.supportRegion.getNumberOfVertices() < 3)
         {
            modelInstance = null;
            lastModel = null;
            return;
         }

         transform.setTranslationAndIdentityRotation(0.0, 0.0, footZ);

         for (int i = 0; i < this.supportRegion.getNumberOfVertices(); i++)
         {
            Point2DReadOnly v0 = this.supportRegion.getVertex(i);
            Point2DReadOnly v1 = this.supportRegion.getNextVertex(i);
            meshBuilder.addLine(v0.getX(), v0.getY(), footZ, v1.getX(), v1.getY(), footZ, 0.01f, NOMINAL_POLYGON_GRAPHIC_COLOR);
         }

         meshBuilder.addPolygon(transform, this.supportRegion, NOMINAL_POLYGON_GRAPHIC_COLOR);
      }

      generateModel();
   }

   private void generateModel()
   {
      modelBuilder.begin();
      Mesh mesh = meshBuilder.generateMesh();

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
      Material material = new Material();
      Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
      material.set(PBRTextureAttribute.createBaseColorTexture(paletteTexture));

      modelBuilder.part(meshPart, material);

      if (lastModel != null)
         lastModel.dispose();

      lastModel = modelBuilder.end();
      modelInstance = new ModelInstance(lastModel);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (supportRegion.isEmpty())
      {
         return;
      }

      modelInstance.getRenderables(renderables, pool);
   }
}
