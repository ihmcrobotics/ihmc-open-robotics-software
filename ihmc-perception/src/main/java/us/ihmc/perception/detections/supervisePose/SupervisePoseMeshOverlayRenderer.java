package us.ihmc.perception.detections.supervisePose;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.sensors.CameraIntrinsics;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public class SupervisePoseMeshOverlayRenderer
{
   /**
    * OBJ vertices in the object coordinate frame.
    */
   private final List<Point3D> meshVertices = new ArrayList<>();

   /**
    * Triangulated OBJ faces.
    */
   private final List<int[]> meshTriangles = new ArrayList<>();

   /**
    * Unique OBJ edges and their adjacent triangles.
    */
   private final List<MeshEdgeInfo> meshEdges = new ArrayList<>();

   /**
    * Scale applied to vertices when the OBJ is loaded.
    *
    * Use:
    * 1.0   when the OBJ is in meters.
    * 0.001 when the OBJ is in millimeters.
    */
   private final double meshScale;

   /**
    * The communicator currently supplies an RGB8 image.
    *
    * RGB:
    * R = 255
    * G = 180
    * B = 0
    */
   private final Scalar frontEdgeColor = new Scalar(255, 180, 0, 255);

   /**
    * Back-facing mesh edges.
    */
   private final Scalar backEdgeColor = new Scalar(0.0, 0.0, 0.0, 255.0);

   private int frontLineThickness = 1;
   private int backLineThickness = 1;

   /**
    * Set this to true if the OBJ has reversed face winding and the front/back
    * edge colors appear swapped.
    */
   private boolean reverseFaceWinding = false;

   public SupervisePoseMeshOverlayRenderer(Path meshFile)
   {
      this(meshFile, 1.0);
   }

   public SupervisePoseMeshOverlayRenderer(Path meshFile,
                                           double meshScale)
   {
      if (meshFile == null)
         throw new IllegalArgumentException("Mesh file path is null");

      if (!Files.isRegularFile(meshFile))
      {
         throw new IllegalArgumentException("Mesh file does not exist: " + meshFile.toAbsolutePath());
      }

      if (!meshFile.getFileName().toString().toLowerCase(Locale.ROOT).endsWith(".obj"))
      {
         throw new IllegalArgumentException("Expected an OBJ mesh file: " + meshFile);
      }

      if (!Double.isFinite(meshScale) || meshScale <= 0.0)
      {
         throw new IllegalArgumentException("Mesh scale must be positive and finite");
      }

      this.meshScale = meshScale;

      loadOBJ(meshFile);
      buildUniqueEdges();

      LogTools.info(String.format(
            "Loaded mesh overlay: file=%s vertices=%d triangles=%d edges=%d scale=%.6f",
            meshFile,
            meshVertices.size(),
            meshTriangles.size(),
            meshEdges.size(),
            meshScale));
   }

   private void loadOBJ(Path meshFile)
   {
      try (BufferedReader reader = Files.newBufferedReader(meshFile))
      {
         String line;
         int lineNumber = 0;

         while ((line = reader.readLine()) != null)
         {
            lineNumber++;

            String trimmedLine = line.trim();

            if (trimmedLine.isEmpty() || trimmedLine.startsWith("#"))
               continue;

            if (trimmedLine.startsWith("v "))
            {
               parseVertex(trimmedLine, lineNumber);
            }
            else if (trimmedLine.startsWith("f "))
            {
               parseFace(trimmedLine, lineNumber);
            }
         }
      }
      catch (IOException exception)
      {
         throw new RuntimeException("Failed to load OBJ mesh: " + meshFile, exception);
      }

      if (meshVertices.isEmpty())
      {
         throw new IllegalArgumentException("OBJ contains no vertices: " + meshFile);
      }

      if (meshTriangles.isEmpty())
      {
         throw new IllegalArgumentException("OBJ contains no valid faces: " + meshFile);
      }
   }

   private void parseVertex(String line,
                            int lineNumber)
   {
      String[] tokens = line.split("\\s+");

      if (tokens.length < 4)
      {
         LogTools.warn("Skipping malformed OBJ vertex at line " + lineNumber + ": " + line);
         return;
      }

      try
      {
         double x = Double.parseDouble(tokens[1]) * meshScale;
         double y = Double.parseDouble(tokens[2]) * meshScale;
         double z = Double.parseDouble(tokens[3]) * meshScale;

         meshVertices.add(new Point3D(x, y, z));
      }
      catch (NumberFormatException exception)
      {
         LogTools.warn("Skipping malformed OBJ vertex at line " + lineNumber + ": " + line);
      }
   }

   private void parseFace(String line, int lineNumber)
   {
      String[] tokens = line.split("\\s+");

      if (tokens.length < 4)
      {
         LogTools.warn("Skipping malformed OBJ face at line " + lineNumber + ": " + line);
         return;
      }

      List<Integer> faceVertexIndices = new ArrayList<>(tokens.length - 1);

      for (int tokenIndex = 1; tokenIndex < tokens.length; tokenIndex++)
      {
         try
         {
            int vertexIndex = parseOBJVertexIndex(tokens[tokenIndex]);

            if (vertexIndex < 0
                || vertexIndex >= meshVertices.size())
            {
               LogTools.warn("Skipping invalid OBJ face at line "+ lineNumber + ": vertex index " + vertexIndex);
               return;
            }

            faceVertexIndices.add(vertexIndex);
         }
         catch (NumberFormatException exception)
         {
            LogTools.warn("Skipping malformed OBJ face at line " + lineNumber + ": " + line);
            return;
         }
      }

      /*
       * Triangle-fan triangulation.
       *
       * A polygon:
       *
       * 0, 1, 2, 3
       *
       * becomes:
       *
       * 0, 1, 2
       * 0, 2, 3
       */
      int firstVertex = faceVertexIndices.get(0);

      for (int index = 1;
           index < faceVertexIndices.size() - 1;
           index++)
      {
         int secondVertex = faceVertexIndices.get(index);
         int thirdVertex = faceVertexIndices.get(index + 1);

         if (firstVertex == secondVertex
             || secondVertex == thirdVertex
             || thirdVertex == firstVertex)
         {
            continue;
         }

         meshTriangles.add(new int[]
                                 {
                                       firstVertex,
                                       secondVertex,
                                       thirdVertex
                                 });
      }
   }

   /**
    * Extracts the vertex part from these OBJ forms:
    *
    * 1
    * 1/2
    * 1//3
    * 1/2/3
    */
   private int parseOBJVertexIndex(String faceToken)
   {
      String vertexIndexToken = faceToken;

      int slashIndex = faceToken.indexOf('/');

      if (slashIndex >= 0)
         vertexIndexToken = faceToken.substring(0, slashIndex);

      int objIndex = Integer.parseInt(vertexIndexToken);

      /*
       * Positive OBJ indices are one-based.
       */
      if (objIndex > 0)
         return objIndex - 1;

      /*
       * Negative OBJ indices are relative to the current end of the vertex
       * list. For example, -1 refers to the most recently declared vertex.
       */
      if (objIndex < 0)
         return meshVertices.size() + objIndex;

      throw new NumberFormatException("OBJ vertex index cannot be zero");
   }

   /**
    * Builds the exact unique edge topology represented by the OBJ faces.
    *
    * No edge-length or stride filtering is performed. Therefore, Java draws
    * every unique edge contained in the sparsified OBJ.
    */
   private void buildUniqueEdges()
   {
      Map<MeshEdge, List<Integer>> edgeTriangleMap = new LinkedHashMap<>();

      for (int triangleIndex = 0; triangleIndex < meshTriangles.size(); triangleIndex++)
      {
         int[] triangle = meshTriangles.get(triangleIndex);

         addTriangleToEdge(edgeTriangleMap, new MeshEdge(triangle[0], triangle[1]), triangleIndex);

         addTriangleToEdge(edgeTriangleMap, new MeshEdge(triangle[1], triangle[2]), triangleIndex);

         addTriangleToEdge(edgeTriangleMap, new MeshEdge(triangle[2], triangle[0]), triangleIndex);
      }

      meshEdges.clear();

      for (Map.Entry<MeshEdge, List<Integer>> entry
            : edgeTriangleMap.entrySet())
      {
         meshEdges.add(new MeshEdgeInfo(entry.getKey(), List.copyOf(entry.getValue())));
      }
   }

   private static void addTriangleToEdge(Map<MeshEdge, List<Integer>> edgeTriangleMap, MeshEdge edge, int triangleIndex)
   {
      edgeTriangleMap.computeIfAbsent(edge, ignored -> new ArrayList<>()).add(triangleIndex);
   }

   public void renderWireframe(Mat image, Pose3DReadOnly objectPoseInCamera, CameraIntrinsics cameraIntrinsics)
   {
      if (image == null || image.isNull())
         return;

      if (objectPoseInCamera == null || cameraIntrinsics == null)
         return;

      if (meshVertices.isEmpty()
          || meshTriangles.isEmpty()
          || meshEdges.isEmpty())
      {
         return;
      }

      Point3D[] verticesInCamera = new Point3D[meshVertices.size()];

      ProjectedVertex[] projectedVertices = new ProjectedVertex[meshVertices.size()];

      /*
       * Transform and project every vertex once.
       */
      for (int vertexIndex = 0; vertexIndex < meshVertices.size(); vertexIndex++)
      {
         Point3D pointInCamera = new Point3D(meshVertices.get(vertexIndex));

         objectPoseInCamera.transform(pointInCamera);

         verticesInCamera[vertexIndex] = pointInCamera;

         projectedVertices[vertexIndex] = projectVertex(pointInCamera, cameraIntrinsics, image.cols(), image.rows());
      }

      /*
       * Determine whether every triangle is facing toward the camera.
       */
      boolean[] triangleFrontFacing = computeTriangleFacing(verticesInCamera);

      /*
       * Draw back-facing edges first. The front-facing edges are drawn
       * afterward so they remain visually dominant.
       */
      drawEdges(image, projectedVertices, triangleFrontFacing, false, backEdgeColor, backLineThickness);

      /*
       * Draw front-facing and silhouette edges in violet-blue.
       */
      drawEdges(image, projectedVertices, triangleFrontFacing, true, frontEdgeColor, frontLineThickness);
   }

   /**
    * Classifies each triangle using its camera-frame normal.
    *
    * In camera coordinates, the camera is located at the origin.
    * For an outward-facing triangle winding, a triangle faces the camera when
    * its normal points generally from the triangle toward the origin.
    */
   private boolean[] computeTriangleFacing(Point3D[] verticesInCamera)
   {
      boolean[] triangleFrontFacing = new boolean[meshTriangles.size()];

      for (int triangleIndex = 0; triangleIndex < meshTriangles.size(); triangleIndex++)
      {
         int[] triangle = meshTriangles.get(triangleIndex);

         Point3D first = verticesInCamera[triangle[0]];

         Point3D second = verticesInCamera[triangle[1]];

         Point3D third = verticesInCamera[triangle[2]];

         Vector3D firstEdge = new Vector3D(second.getX() - first.getX(), second.getY() - first.getY(), second.getZ() - first.getZ());

         Vector3D secondEdge = new Vector3D(third.getX() - first.getX(), third.getY() - first.getY(), third.getZ() - first.getZ());

         Vector3D normal = new Vector3D();
         normal.cross(firstEdge, secondEdge);

         if (normal.normSquared() < 1.0e-16)
         {
            triangleFrontFacing[triangleIndex] = false;
            continue;
         }

         /*
          * Triangle centroid in camera coordinates.
          */
         double centroidX = (first.getX() + second.getX() + third.getX()) / 3.0;

         double centroidY = (first.getY() + second.getY() + third.getY()) / 3.0;

         double centroidZ = (first.getZ() + second.getZ() + third.getZ()) / 3.0;

         /*
          * A front-facing outward normal points toward the camera origin.
          *
          * camera direction from triangle = -centroid
          */
         double facingDot = normal.getX() * (-centroidX) + normal.getY() * (-centroidY) + normal.getZ() * (-centroidZ);

         boolean frontFacing = facingDot > 0.0;

         if (reverseFaceWinding)
            frontFacing = !frontFacing;

         triangleFrontFacing[triangleIndex] = frontFacing;
      }

      return triangleFrontFacing;
   }

   private void drawEdges(Mat image, ProjectedVertex[] projectedVertices, boolean[] triangleFrontFacing, boolean drawFrontEdges, Scalar color, int thickness)
   {
      for (MeshEdgeInfo edgeInfo : meshEdges)
      {
         MeshEdge edge = edgeInfo.edge();

         ProjectedVertex first = projectedVertices[edge.firstVertexIndex()];

         ProjectedVertex second = projectedVertices[edge.secondVertexIndex()];

         if (!first.valid() || !second.valid())
            continue;

         boolean edgeHasFrontFacingTriangle = false;

         for (int triangleIndex
               : edgeInfo.adjacentTriangleIndices())
         {
            if (triangleFrontFacing[triangleIndex])
            {
               edgeHasFrontFacingTriangle = true;
               break;
            }
         }

         /*
          * If at least one neighboring triangle faces the camera, the edge is
          * treated as a front or silhouette edge.
          *
          * If all neighboring triangles face away, the edge is a back edge.
          */
         boolean edgeIsFront = edgeHasFrontFacingTriangle;

         if (edgeIsFront != drawFrontEdges)
            continue;

         Point firstPixel = new Point(first.u(), first.v());

         Point secondPixel = new Point(second.u(), second.v());

         try
         {
            opencv_imgproc.line(image, firstPixel, secondPixel, color, thickness, opencv_imgproc.LINE_AA, 0);
         }
         finally
         {
            firstPixel.close();
            secondPixel.close();
         }
      }
   }

   private ProjectedVertex projectVertex(Point3D pointInCamera, CameraIntrinsics cameraIntrinsics, int imageWidth, int imageHeight)
   {
      double x = pointInCamera.getX();
      double y = pointInCamera.getY();
      double z = pointInCamera.getZ();

      if (!Double.isFinite(x)
          || !Double.isFinite(y)
          || !Double.isFinite(z)
          || z <= 1.0e-6)
      {
         return ProjectedVertex.invalid();
      }

      double uDouble = cameraIntrinsics.getFx() * x / z + cameraIntrinsics.getCx();

      double vDouble = cameraIntrinsics.getFy() * y / z + cameraIntrinsics.getCy();

      if (!Double.isFinite(uDouble) || !Double.isFinite(vDouble))
      {
         return ProjectedVertex.invalid();
      }

      /*
       * Edges are currently drawn only when both endpoints project inside the
       * image. Image-border line clipping can be added later if necessary.
       */
      if (uDouble < 0.0
          || uDouble >= imageWidth
          || vDouble < 0.0
          || vDouble >= imageHeight)
      {
         return ProjectedVertex.invalid();
      }

      int u = (int) Math.round(uDouble);
      int v = (int) Math.round(vDouble);

      return new ProjectedVertex(u, v, true);
   }

   public void setFrontLineThickness(int thickness)
   {
      frontLineThickness = Math.max(1, thickness);
   }

   public void setBackLineThickness(int thickness)
   {
      backLineThickness = Math.max(1, thickness);
   }

   /**
    * Enable this only when front and back colors appear reversed.
    */
   public void setReverseFaceWinding(boolean reverseFaceWinding)
   {
      this.reverseFaceWinding = reverseFaceWinding;
   }

   public int getVertexCount()
   {
      return meshVertices.size();
   }

   public int getTriangleCount()
   {
      return meshTriangles.size();
   }

   public int getEdgeCount()
   {
      return meshEdges.size();
   }

   public void destroy()
   {
      frontEdgeColor.close();
      backEdgeColor.close();
   }

   /**
    * Undirected mesh edge.
    *
    * Sorting the indices ensures that:
    *
    * MeshEdge(1, 2) equals MeshEdge(2, 1).
    */
   private record MeshEdge(int firstVertexIndex, int secondVertexIndex)
   {
      private MeshEdge
      {
         if (firstVertexIndex > secondVertexIndex)
         {
            int temporaryIndex = firstVertexIndex;
            firstVertexIndex = secondVertexIndex;
            secondVertexIndex = temporaryIndex;
         }
      }
   }

   /**
    * Stores an edge and all triangles that share that edge.
    */
   private record MeshEdgeInfo(MeshEdge edge, List<Integer> adjacentTriangleIndices)
   {
   }

   private record ProjectedVertex(int u, int v, boolean valid)
   {
      private static ProjectedVertex invalid()
      {
         return new ProjectedVertex(0, 0, false);
      }
   }
}