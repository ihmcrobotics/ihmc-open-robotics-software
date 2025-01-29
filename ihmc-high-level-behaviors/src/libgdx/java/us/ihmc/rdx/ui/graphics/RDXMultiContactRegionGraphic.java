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
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import org.lwjgl.opengl.GL41;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXMultiContactRegionGraphic implements RenderableProvider
{
   private static final Color POLYGON_GRAPHIC_COLOR = Color.valueOf("DEE933");
   private static final double STABILITY_GRAPHIC_HEIGHT = 2.0;

   private final ConvexPolygon2D multiContactSupportRegion = new ConvexPolygon2D();

   private final FramePoint3D comCurrent = new FramePoint3D();
   private final FramePoint3D comXYAtFootHeight = new FramePoint3D();
   private final ConvexPolygon2D closestProximityEdge = new ConvexPolygon2D();

   private int minimumEdgeIndex;
   private double minimumEdgeDistance;

   private final ModelBuilder modelBuilder = new ModelBuilder();
   private final RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
   private final FullHumanoidRobotModel ghostFullRobotModel;
   private final CenterOfMassReferenceFrame centerOfMassFrame;
   private final MidFrameZUpFrame midFeetZUpFrame;
   private final RigidBodyTransform transform = new RigidBodyTransform();

   private ModelInstance modelInstance;
   private Model lastModel;

   public RDXMultiContactRegionGraphic(FullHumanoidRobotModel ghostFullRobotModel)
   {
      this.ghostFullRobotModel = ghostFullRobotModel;
      this.centerOfMassFrame = new CenterOfMassReferenceFrame("ghostCoMFrame", ReferenceFrame.getWorldFrame(), ghostFullRobotModel.getRootBody());
      this.midFeetZUpFrame = new MidFrameZUpFrame("midFeedZUpWhost",
                                                  ReferenceFrame.getWorldFrame(),
                                                  ghostFullRobotModel.getSoleFrame(RobotSide.LEFT),
                                                  ghostFullRobotModel.getSoleFrame(RobotSide.RIGHT));
   }

   public void update(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus)
   {
      multiContactSupportRegion.clear();
      Object<Point3D> supportRegion = kinematicsToolboxOutputStatus.getSupportRegion();

      for (int i = 0; i < supportRegion.size(); i++)
      {
         multiContactSupportRegion.addVertex(supportRegion.get(i));
      }

      multiContactSupportRegion.update();

      if (multiContactSupportRegion.getNumberOfVertices() < 3)
      {
         modelInstance = null;
         lastModel = null;
         return;
      }

      meshBuilder.clear();

      centerOfMassFrame.update();
      midFeetZUpFrame.update();
      updateMinimumEdge();

      FramePoint3D midFoot = new FramePoint3D(midFeetZUpFrame);
      midFoot.changeFrame(ReferenceFrame.getWorldFrame());
      double footZ = midFoot.getZ();

      transform.setTranslationAndIdentityRotation(0.0, 0.0, footZ);

      for (int i = 0; i < multiContactSupportRegion.getNumberOfVertices(); i++)
      {
         Point2DReadOnly v0 = multiContactSupportRegion.getVertex(i);
         Point2DReadOnly v1 = multiContactSupportRegion.getNextVertex(i);

         Color color = i == minimumEdgeIndex ? Color.RED : POLYGON_GRAPHIC_COLOR;
         meshBuilder.addLine(v0.getX(), v0.getY(), footZ, v1.getX(), v1.getY(), footZ, 0.01f, color);
      }

      meshBuilder.addPolygon(transform, multiContactSupportRegion, POLYGON_GRAPHIC_COLOR);

      comCurrent.setToZero(centerOfMassFrame);
      comCurrent.changeFrame(ReferenceFrame.getWorldFrame());

      meshBuilder.addSphere(0.03f, comCurrent, Color.BLACK);

      comXYAtFootHeight.setIncludingFrame(comCurrent);
      comXYAtFootHeight.setZ(footZ);
      meshBuilder.addSphere(0.03f, comXYAtFootHeight, Color.BLACK);

      FramePoint3D comXYElevated = new FramePoint3D(comXYAtFootHeight);
      comXYElevated.addZ(STABILITY_GRAPHIC_HEIGHT);
      meshBuilder.addLine(comXYAtFootHeight, comXYElevated, 0.005f, Color.BLACK);

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

   private void updateMinimumEdge()
   {
      minimumEdgeDistance = Double.POSITIVE_INFINITY;

      for (int i = 0; i < multiContactSupportRegion.getNumberOfVertices(); i++)
      {
         Point2DReadOnly v0 = multiContactSupportRegion.getVertex(i);
         Point2DReadOnly v1 = multiContactSupportRegion.getNextVertex(i);

         double margin = EuclidGeometryTools.distanceFromPoint2DToLine2D(comXYAtFootHeight.getX(), comXYAtFootHeight.getY(), v0, v1);
         if (margin < minimumEdgeDistance)
         {
            minimumEdgeDistance = margin;
            minimumEdgeIndex = i;
         }
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (multiContactSupportRegion.isEmpty())
      {
         return;
      }

      modelInstance.getRenderables(renderables, pool);
   }
}
