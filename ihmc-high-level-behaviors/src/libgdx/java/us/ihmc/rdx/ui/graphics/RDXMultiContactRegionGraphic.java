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
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXMultiContactRegionGraphic implements RenderableProvider
{
   public static final Color POLYGON_GRAPHIC_COLOR = new Color(0.867f, 0.91f, 0.203f, 1.0f);
   private static final boolean SHOW_EDGE_PROXIMITY_POLYGON = false;
   private static final double STABILITY_GRAPHIC_HEIGHT = 2.0;

   private final ConvexPolygon2D newMultiContactSupportRegion = new ConvexPolygon2D();
   private final ConvexPolygon2D multiContactSupportRegion = new ConvexPolygon2D();

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
      this.midFeetZUpFrame = new MidFrameZUpFrame("midFeedZUpWhost", ReferenceFrame.getWorldFrame(), ghostFullRobotModel.getSoleFrame(RobotSide.LEFT), ghostFullRobotModel.getSoleFrame(RobotSide.RIGHT));
   }

   public void update(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus)
   {
      newMultiContactSupportRegion.clear();
      Object<Point3D> supportRegion = kinematicsToolboxOutputStatus.getMultiContactFeasibleComRegion();

      for (int i = 0; i < supportRegion.size(); i++)
      {
         newMultiContactSupportRegion.addVertex(supportRegion.get(i));
      }

      newMultiContactSupportRegion.update();

      if (newMultiContactSupportRegion.epsilonEquals(multiContactSupportRegion, 1.0e-3))
      {
         return;
      }

      multiContactSupportRegion.set(newMultiContactSupportRegion);

      if (newMultiContactSupportRegion.getNumberOfVertices() < 3)
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

      comXYAtFootHeight.setToZero(centerOfMassFrame);
      comXYAtFootHeight.changeFrame(ReferenceFrame.getWorldFrame());
      comXYAtFootHeight.setZ(footZ);
      meshBuilder.addSphere(0.03f, comXYAtFootHeight, Color.BLACK);

      FramePoint3D comXYElevated = new FramePoint3D(comXYAtFootHeight);
      comXYElevated.addZ(STABILITY_GRAPHIC_HEIGHT);
      meshBuilder.addLine(comXYAtFootHeight, comXYElevated, 0.005f, Color.BLACK);

      if (SHOW_EDGE_PROXIMITY_POLYGON)
      {
         FramePose3D edgeProximityWarningPose = new FramePose3D();
         Point2DReadOnly v0 = multiContactSupportRegion.getVertex(minimumEdgeIndex);
         Point2DReadOnly v1 = multiContactSupportRegion.getNextVertex(minimumEdgeIndex);

         // Set proximity warning position
         FixedFramePoint3DBasics proximityWarningPosition = edgeProximityWarningPose.getPosition();
         proximityWarningPosition.setX(0.5 * (v0.getX() + v1.getX()));
         proximityWarningPosition.setY(0.5 * (v0.getY() + v1.getY()));
         proximityWarningPosition.setZ(footZ + 0.5 * STABILITY_GRAPHIC_HEIGHT);

         // Set proximity warning orientation
         double dx = v1.getX() - v0.getX();
         double dy = v1.getY() - v0.getY();
         Vector3D frameZ = new Vector3D(dy, -dx, 0.0); // orthogonal to region
         frameZ.normalize();
         Vector3D frameY = new Vector3D(Axis3D.Z); // we want y to be up
         Vector3D frameX = new Vector3D();
         frameX.cross(frameY, frameZ);

         RotationMatrix frameRotation = new RotationMatrix();
         frameRotation.setColumns(frameX, frameY, frameZ);
         edgeProximityWarningPose.getOrientation().set(frameRotation);

         edgeProximityWarningPose.get(transform);
         double vertexDistance = v1.distance(v0);
         closestProximityEdge.clear();
         closestProximityEdge.addVertex(-0.5 * vertexDistance, - 0.5 * STABILITY_GRAPHIC_HEIGHT);
         closestProximityEdge.addVertex(0.5 * vertexDistance, - 0.5 * STABILITY_GRAPHIC_HEIGHT);
         closestProximityEdge.addVertex(-0.5 * vertexDistance, 0.5 * STABILITY_GRAPHIC_HEIGHT);
         closestProximityEdge.addVertex(0.5 * vertexDistance, 0.5 * STABILITY_GRAPHIC_HEIGHT);
         closestProximityEdge.update();

         meshBuilder.addPolygon(transform, closestProximityEdge, POLYGON_GRAPHIC_COLOR);
      }

      modelBuilder.begin();
      Mesh mesh = meshBuilder.generateMesh();

      MeshPart meshPart = new MeshPart("xyz", mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
      Material material = new Material();
      Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
      material.set(PBRTextureAttribute.createBaseColorTexture(paletteTexture));
//      material.set(PBRColorAttribute.createBaseColorFactor(new com.badlogic.gdx.graphics.Color(0.867f, 0.91f, 0.203f, 1.0f)));

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
