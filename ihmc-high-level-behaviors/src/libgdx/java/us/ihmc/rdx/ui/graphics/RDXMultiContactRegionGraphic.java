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
import toolbox_msgs.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXMultiContactRegionGraphic implements RenderableProvider
{
   public static final Color NOMINAL_POLYGON_GRAPHIC_COLOR = Color.valueOf("DEE933");
   private static final double STABILITY_GRAPHIC_HEIGHT = 2.0;

   private final ConvexPolygon2D supportRegion = new ConvexPolygon2D();

   private final FramePoint3D comCurrent = new FramePoint3D();
   private final FramePoint3D comDesired = new FramePoint3D();
   private final FramePoint3D comXYAtFootHeight = new FramePoint3D();
   private final FramePoint3D desiredCoMXYAtFootHeight = new FramePoint3D();

   private int minimumEdgeIndex;
   private double stabilityMargin;

   private final FullHumanoidRobotModel ghostFullRobotModel;
   private final ModelBuilder modelBuilder = new ModelBuilder();
   private final RDXMultiColorMeshBuilder meshBuilder = new RDXMultiColorMeshBuilder();
   private final CenterOfMassReferenceFrame centerOfMassFrame;
   private final MidFrameZUpFrame midFeetZUpFrame;
   private final RigidBodyTransform transform = new RigidBodyTransform();

   private ModelInstance modelInstance;
   private Model lastModel;

   public RDXMultiContactRegionGraphic(FullHumanoidRobotModel ghostFullRobotModel)
   {
      this.ghostFullRobotModel = ghostFullRobotModel;
      this.centerOfMassFrame = new CenterOfMassReferenceFrame("ghostCoMFrame", ReferenceFrame.getWorldFrame(), ghostFullRobotModel.getRootBody());
      this.midFeetZUpFrame = new MidFrameZUpFrame("midFeetZUpGhost",
                                                  ReferenceFrame.getWorldFrame(),
                                                  ghostFullRobotModel.getSoleFrame(RobotSide.LEFT),
                                                  ghostFullRobotModel.getSoleFrame(RobotSide.RIGHT));
   }

   public void update(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus, FramePoint3D desiredCoMPosition)
   {
      meshBuilder.clear();

      // Update frames and compute mid-feet height
      centerOfMassFrame.update();
      midFeetZUpFrame.update();

      FramePoint3D midFoot = new FramePoint3D(midFeetZUpFrame);
      midFoot.changeFrame(ReferenceFrame.getWorldFrame());
      double footZ = midFoot.getZ();

      // Update desired and achieved CoM graphic
      {
         comCurrent.setToZero(centerOfMassFrame);
         comCurrent.changeFrame(ReferenceFrame.getWorldFrame());
         meshBuilder.addSphere(0.03f, comCurrent, Color.BLACK);

         comXYAtFootHeight.setIncludingFrame(comCurrent);
         comXYAtFootHeight.setZ(footZ);
         meshBuilder.addSphere(0.03f, comXYAtFootHeight, Color.BLACK);

         FramePoint3D comXYElevated = new FramePoint3D(comXYAtFootHeight);
         comXYElevated.addZ(STABILITY_GRAPHIC_HEIGHT);
         meshBuilder.addLine(comXYAtFootHeight, comXYElevated, 0.005f, Color.BLACK);

         if (desiredCoMPosition != null)
         {
            comDesired.setMatchingFrame(desiredCoMPosition);
            meshBuilder.addSphere(0.03f, comDesired, Color.GREEN);

            desiredCoMXYAtFootHeight.setIncludingFrame(comDesired);
            desiredCoMXYAtFootHeight.setZ(footZ);
            meshBuilder.addSphere(0.03f, desiredCoMXYAtFootHeight, Color.GREEN);

            FramePoint3D desiredComXYElevated = new FramePoint3D(desiredCoMXYAtFootHeight);
            desiredComXYElevated.addZ(STABILITY_GRAPHIC_HEIGHT);
            meshBuilder.addLine(desiredCoMXYAtFootHeight, desiredComXYElevated, 0.005f, Color.GREEN);
         }
      }

      // Update region graphic
      var ikSupportRegion = kinematicsToolboxOutputStatus.getSupportRegion();
      if (ikSupportRegion.size() >= 3)
      {
         this.supportRegion.clear();

         for (int i = 0; i < ikSupportRegion.size(); i++)
         {
            this.supportRegion.addVertex(ikSupportRegion.get(i).getPoint());
         }

         this.supportRegion.update();

         if (this.supportRegion.getNumberOfVertices() < 3)
         {
            modelInstance = null;
            lastModel = null;
            return;
         }

         updateMinimumEdge();

         transform.setTranslationAndIdentityRotation(0.0, 0.0, footZ);

         for (int i = 0; i < this.supportRegion.getNumberOfVertices(); i++)
         {
            Point2DReadOnly v0 = this.supportRegion.getVertex(i);
            Point2DReadOnly v1 = this.supportRegion.getNextVertex(i);

            Color color = i == minimumEdgeIndex ? Color.RED : NOMINAL_POLYGON_GRAPHIC_COLOR;
            meshBuilder.addLine(v0.getX(), v0.getY(), footZ, v1.getX(), v1.getY(), footZ, 0.01f, color);
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

   private void updateMinimumEdge()
   {
      stabilityMargin = Double.POSITIVE_INFINITY;

      for (int i = 0; i < supportRegion.getNumberOfVertices(); i++)
      {
         Point2DReadOnly v0 = supportRegion.getVertex(i);
         Point2DReadOnly v1 = supportRegion.getNextVertex(i);

         double margin = EuclidGeometryTools.distanceFromPoint2DToLine2D(comXYAtFootHeight.getX(), comXYAtFootHeight.getY(), v0, v1);
         if (margin < stabilityMargin)
         {
            stabilityMargin = margin;
            minimumEdgeIndex = i;
         }
      }
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
