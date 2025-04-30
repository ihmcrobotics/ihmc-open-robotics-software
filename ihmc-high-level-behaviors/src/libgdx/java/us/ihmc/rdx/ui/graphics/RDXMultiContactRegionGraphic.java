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
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.rdx.mesh.RDXMeshGraphicTools;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;

import java.util.concurrent.atomic.AtomicReference;

public class RDXMultiContactRegionGraphic implements RenderableProvider
{
   public static final Color NOMINAL_POLYGON_GRAPHIC_COLOR = Color.valueOf("DEE933");
   public static final Color OPTIMIZER_POLYGON_GRAPHIC_COLOR = Color.valueOf("EB3D40");
   private static final double STABILITY_GRAPHIC_HEIGHT = 2.0;

   private final ConvexPolygon2D supportRegion = new ConvexPolygon2D();
   private double postureSensitivity = -1.0;

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

   private final AtomicReference<FramePlanarRegionsListMessage> latestPlanarRegionsMessage = new AtomicReference<>();
   private final SideDependentList<Vector3D> bracingRegionPoints = new SideDependentList<>(new Vector3D(), new Vector3D());
   private final SideDependentList<Vector3D> bracingRegionNormals = new SideDependentList<>(new Vector3D(), new Vector3D());
   private final SideDependentList<Quaternion> bracingRegionOrientations = new SideDependentList<>(new Quaternion(), new Quaternion());
   private final SideDependentList<RotationMatrix> bracingRegionRotations = new SideDependentList<>(new RotationMatrix(), new RotationMatrix());
   private final SideDependentList<PlanarRegion> bracingRegions = new SideDependentList<>();

   private ModelInstance modelInstance;
   private Model lastModel;

   public RDXMultiContactRegionGraphic(FullHumanoidRobotModel ghostFullRobotModel, ROS2Node ros2Node)
   {
      this.ghostFullRobotModel = ghostFullRobotModel;
      this.centerOfMassFrame = new CenterOfMassReferenceFrame("ghostCoMFrame", ReferenceFrame.getWorldFrame(), ghostFullRobotModel.getRootBody());
      this.midFeetZUpFrame = new MidFrameZUpFrame("midFeetZUpGhost",
                                                  ReferenceFrame.getWorldFrame(),
                                                  ghostFullRobotModel.getSoleFrame(RobotSide.LEFT),
                                                  ghostFullRobotModel.getSoleFrame(RobotSide.RIGHT));

      ros2Node.createSubscription(PerceptionAPI.PERSPECTIVE_RAPID_REGIONS, s -> latestPlanarRegionsMessage.set(s.takeNextData()));
   }

   public void updateSupportRegionAndGraphics(KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus, FramePoint3D desiredCoMPosition)
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
      Object<Point3D> ikSupportRegion = kinematicsToolboxOutputStatus.getSupportRegion();
      if (ikSupportRegion.size() >= 3)
      {
         this.supportRegion.clear();

         for (int i = 0; i < ikSupportRegion.size(); i++)
         {
            this.supportRegion.addVertex(ikSupportRegion.get(i));
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

         postureSensitivity = kinematicsToolboxOutputStatus.getSupportRegionSensitivity();
         meshBuilder.addPolygon(transform, this.supportRegion, getPolygonColor());
      }

      addRegionGraphic();
      generateModel();
   }

   public void updateBracingRegionAndGraphics()
   {
      midFeetZUpFrame.update();

      // Check for new surface normal
      FramePlanarRegionsListMessage planarRegionsMessage = latestPlanarRegionsMessage.getAndSet(null);
      if (planarRegionsMessage != null)
      {
//         LogTools.info("Received " + planarRegionsMessage.getPlanarRegions().getRegionNormal().size() + " regions.");

         PlanarRegionsList planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsListInWorld(planarRegionsMessage);

         for (RobotSide robotSide : RobotSide.values)
         {
            this.bracingRegions.put(robotSide, null);

            PlanarRegion bracingRegion = null;
            double minDistanceToHandXY = Double.MAX_VALUE;

            for (int i = 0; i < planarRegions.getNumberOfPlanarRegions(); i++)
            {
               PlanarRegion region = planarRegions.getPlanarRegion(i);

               double area = region.getArea();
               double areaThreshold = 0.25;

               if (area < areaThreshold)
                  continue;

               FrameVector3D normal = new FrameVector3D(ReferenceFrame.getWorldFrame(), region.getNormal());
               normal.changeFrame(midFeetZUpFrame);
               double normalZ = normal.getZ();
               double pitch = Math.abs(Math.asin(normalZ));
               double pitchThreshold = Math.toRadians(10.0); // Math.toRadians(9.0);

               // hardware
               //            if (pitch > pitchThreshold)
               //               continue;
               //            if (normal.getX() > -0.2)
               //               continue;

               // back wall
               //            if (pitch > pitchThreshold)
               //               continue;
               //            if (normal.getX() > 0.0)
               //               continue;

               // top wall
//               if (robotSide == RobotSide.RIGHT && (normal.getZ() > -0.5 || normal.getX() > 0.1))
//                  continue;

               // pitched wall
               //            if (Math.abs(normal.getZ()) > 0.5)
               //               continue;
               //            if (normal.getX() > -0.3)
               //               continue;

               // left wall
//               if (robotSide == RobotSide.LEFT && (normal.getY() > -0.5 || normal.getX() > 0.1))
//                  continue;

               // right wall
               if (robotSide == RobotSide.RIGHT && normal.getY() < 0.8)
                  continue;

               Point3D centroid = PlanarRegionTools.getCentroid3DInWorld(region);
               FramePoint3D hand = new FramePoint3D(ghostFullRobotModel.getHandControlFrame(robotSide));
               hand.changeFrame(ReferenceFrame.getWorldFrame());

               normal.changeFrame(ReferenceFrame.getWorldFrame());
               Plane3D regionPlane = new Plane3D(centroid, normal);

               double distanceXY = regionPlane.distance(hand);
               if (distanceXY < minDistanceToHandXY)
               {
                  minDistanceToHandXY = distanceXY;
                  bracingRegion = region;
               }
            }

            double distanceThresholdXY = 1.5;
            if (minDistanceToHandXY < distanceThresholdXY)
            {
               bracingRegionPoints.get(robotSide).set(PlanarRegionTools.getCentroid3DInWorld(bracingRegion));
               bracingRegionNormals.get(robotSide).set(bracingRegion.getNormal());
               bracingRegionRotations.get(robotSide).set(bracingRegion.getTransformToWorldCopy().getRotation());
               bracingRegion.updateConvexHull();
               this.bracingRegions.put(robotSide, bracingRegion);

//               System.out.println(bracingRegionNormal);

               //            FramePose3D bracingRegionPose = new FramePose3D(ReferenceFrame.getWorldFrame(), bracingRegion.getTransformToWorld());
               //            System.out.println(bracingRegionPose.getPosition());
               //            System.out.println(bracingRegionPose.getOrientation());
               //
               //            UnitVector3DReadOnly normal = bracingRegion.getNormal();
               //            System.out.println(normal.getX() + ", " + normal.getY() + ", " + normal.getZ());
               //
               //            ConvexPolygon2D convexHull = bracingRegion.getConvexHull();
               //            System.out.println(convexHull);
            }
         }
      }

      meshBuilder.clear();
      addRegionGraphic();

      generateModel();
   }

   private void addRegionGraphic()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         PlanarRegion bracingRegion = bracingRegions.get(robotSide);
         if (bracingRegion != null)
         {
            Color color = Color.GREEN;
            RigidBodyTransform transformToWorld = bracingRegion.getTransformToWorldCopy();
            meshBuilder.addMultiLine(transformToWorld, bracingRegion.getConcaveHull(), 0.01, color, true);
            for (ConvexPolygon2D convexPolygon : bracingRegion.getConvexPolygons())
            {
               meshBuilder.addPolygon(transformToWorld, convexPolygon, color);
            }

            double length = 0.07;
            double radius = 0.004;
            double cylinderToConeLengthRatio = 0.8;
            double coneDiameterMultiplier = 1.8;
            RDXMeshGraphicTools.drawArrow(meshBuilder, bracingRegionPoints.get(robotSide), bracingRegionRotations.get(robotSide), length, radius, cylinderToConeLengthRatio, coneDiameterMultiplier, Color.RED);
         }
      }
   }

   public boolean hasRegion(RobotSide robotSide)
   {
      return bracingRegions.get(robotSide) != null;
   }

   public PlanarRegion getBracingRegion(RobotSide robotSide)
   {
      return bracingRegions.get(robotSide);
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

   private Color getPolygonColor()
   {
      return (postureSensitivity > 0.035 && stabilityMargin < 0.08) ? OPTIMIZER_POLYGON_GRAPHIC_COLOR : NOMINAL_POLYGON_GRAPHIC_COLOR;
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
      if (!supportRegion.isEmpty() || bracingRegionNormals.get(RobotSide.LEFT).norm() > 0.01 || bracingRegionNormals.get(RobotSide.LEFT).norm() > 0.01)
         modelInstance.getRenderables(renderables, pool);
   }
}
