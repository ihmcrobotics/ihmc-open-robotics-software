package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.graphics.g3d.utils.MeshPartBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import net.mgsx.gltf.scene3d.attributes.PBRTextureAttribute;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;

import java.util.ArrayList;

/**
 * Class that creates a colored shape that has a set of splines as edges.
 * The shape defined only by the edges is an open shape.
 * It can be closed and transformed into a 3D polygon, or partially closed by adding rectangular patches.
 */
public class RDXEdgeDefinedShapeGraphic implements RenderableProvider
{
   private ModelInstance modelInstance;
   private Model lastModel;
   private final ModelBuilder modelBuilder = new ModelBuilder();
   private volatile Runnable buildMesh = null;
   private final Material material = new Material();
   private final Point3D[][] edges;
   private final Point3D[] startPoints;
   private final Point3D[] endPoints;
   private final int numberOfSplines;
   private final int numberPointsPerSpline;
   private final ArrayList<RDXSplineBody> edgeLines = new ArrayList<>();
   private final Color edgeColor;
   private final float opacity;

   public RDXEdgeDefinedShapeGraphic(Point3D[][] edges, Color color, Color edgeColor, float opacity)
   {
      this.edges = edges;
      numberOfSplines = edges.length;
      numberPointsPerSpline = edges[0].length;

      startPoints =  new Point3D[numberOfSplines];
      endPoints = new Point3D[numberOfSplines];
      for (int i = 0; i < numberOfSplines; i++)
      {
         startPoints[i] = edges[i][0];
         endPoints[i] = edges[i][numberPointsPerSpline - 1];
      }

      this.opacity = opacity;
      this.edgeColor = edgeColor;
      Texture paletteTexture = RDXMultiColorMeshBuilder.loadPaletteTexture();
      material.set(PBRTextureAttribute.createBaseColorTexture(paletteTexture));
      material.set(PBRColorAttribute.createBaseColorFactor(new com.badlogic.gdx.graphics.Color(color.r, color.g, color.b, 1.0f)));
      material.set(new BlendingAttribute(true, GL41.GL_SRC_ALPHA, GL41.GL_ONE_MINUS_SRC_ALPHA, opacity));
   }

   public void update()
   {
      if (buildMesh != null)
      {
         buildMesh.run();
         buildMesh = null;
      }
   }

   public void generateMesh()
   {
      // create the model instance
      lastModel = modelBuilder.end();
      modelInstance = new ModelInstance(lastModel);
   }

   /**
   * Create the main part of the shape by connecting each 2 consecutive points of each adjacent splines with rectangular meshes
    */
   public synchronized void createMainShape()
   {
      buildMesh = () ->
      {
         if (lastModel != null)
         {
            lastModel.dispose();
         }

         modelBuilder.begin();
         MeshPartBuilder meshBuilder = modelBuilder.part("mainShape", GL20.GL_TRIANGLES, VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal, material);
         // create rectangle meshes that connect adjacent splines
         for (int i = 0; i < numberOfSplines; i++)
         {
            // create edge lines to help with visualization of the complex resulting polygon mesh
            edgeLines.add(new RDXSplineBody(0.01f, opacity));
            edgeLines.get(i).setColor(edgeColor);

            // wrap around the index of the next spline to handle cases where adjacent splines do not connect at their endpoints
            int j = (i + 1) % numberOfSplines;
            for (int k = 0; k < numberPointsPerSpline - 1; k++)
            {
               // generate edge lines
               edgeLines.get(i).generateMeshes(edges[i][k], edges[i][k + 1]);

               // get the four points defining a rectangle
               Point3D p1 = edges[i][k];
               Point3D p2 = edges[i][k + 1];
               Point3D p3 = edges[j][k + 1];
               Point3D p4 = edges[j][k];

               addRectangleToMesh(meshBuilder, p1, p2, p3, p4);
            }
         }
      };
   }

   private void addRectangleToMesh(MeshPartBuilder meshBuilder, Point3D p1, Point3D p2, Point3D p3, Point3D p4)
   {
      // calculate the normal of the rectangle
      double nx = (p2.getY() - p1.getY()) * (p4.getZ() - p1.getZ()) - (p2.getZ() - p1.getZ()) * (p4.getY() - p1.getY());
      double ny = (p2.getZ() - p1.getZ()) * (p4.getX() - p1.getX()) - (p2.getX() - p1.getX()) * (p4.getZ() - p1.getZ());
      double nz = (p2.getX() - p1.getX()) * (p4.getY() - p1.getY()) - (p2.getY() - p1.getY()) * (p4.getX() - p1.getX());
      double length = Math.sqrt(nx * nx + ny * ny + nz * nz);
      nx /= length;
      ny /= length;
      nz /= length;

      // add the vertices and indices of the rectangle to the mesh
      meshBuilder.rect((float) p1.getX(),
                       (float) p1.getY(),
                       (float) p1.getZ(),
                       (float) p2.getX(),
                       (float) p2.getY(),
                       (float) p2.getZ(),
                       (float) p3.getX(),
                       (float) p3.getY(),
                       (float) p3.getZ(),
                       (float) p4.getX(),
                       (float) p4.getY(),
                       (float) p4.getZ(),
                       (float) nx,
                       (float) ny,
                       (float) nz);

      // add a second rectangle with the vertices in reverse order to create a face facing the opposite direction
      // this is to avoid having the mesh being visible only from one side
      meshBuilder.rect((float) p4.getX(),
                       (float) p4.getY(),
                       (float) p4.getZ(),
                       (float) p3.getX(),
                       (float) p3.getY(),
                       (float) p3.getZ(),
                       (float) p2.getX(),
                       (float) p2.getY(),
                       (float) p2.getZ(),
                       (float) p1.getX(),
                       (float) p1.getY(),
                       (float) p1.getZ(),
                       (float) -nx,
                       (float) -ny,
                       (float) -nz);
   }

   /**
    * Add a rectangular patch.
    * This can be useful to manually close parts of the shape.
    * In fact, createMainShape() generates an open shape where the start and end points of the edge splines are not fully connected.
    */
   public void addRectangularPatch(Point3D p1, Point3D p2, Point3D p3, Point3D p4)
   {
      MeshPartBuilder meshBuilder = modelBuilder.part("patches", GL20.GL_TRIANGLES, VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal, material);
      addRectangleToMesh(meshBuilder, p1, p2, p3, p4);
   }

   /**
    * Add a polygonal patch to close the shape.
    * Connect all the start and end points of the edge splines.
    * NOTE. This can generate a non-convex shape if the start points or end points are not on the same plane
    */
   public void closeShape()
   {
      // create a polygon mesh connecting the starting points
      if (startPoints.length > 2)
         modelBuilder.part(createPolygonalPatch(startPoints, false), material);
      // create a polygon mesh connecting the end points
      if (endPoints.length > 2)
         modelBuilder.part(createPolygonalPatch(endPoints, true), material);
   }

   private MeshPart createPolygonalPatch(Point3D[] points, boolean reverseVertexOrder)
   {
      float[] vertices = new float[points.length * 3];
      short[] indices = new short[(points.length - 2) * 3];

      // add the vertices of the polygon in a consistent order
      for (int i = 0; i < points.length; i++)
      {
         Point3D p;
         if(reverseVertexOrder)
            p = points[points.length - 1 - i];
         else
            p = points[i];
         vertices[i * 3] = (float) p.getX();
         vertices[i * 3 + 1] = (float) p.getY();
         vertices[i * 3 + 2] = (float) p.getZ();
      }

      // add the indices of the triangles in the polygon
      int numTriangles = points.length - 2;
      for (int i = 0; i < numTriangles; i++)
      {
         indices[i * 3] = 0;
         indices[i * 3 + 1] = (short) (i + 1);
         indices[i * 3 + 2] = (short) (i + 2);
      }

      Mesh mesh = new Mesh(true, vertices.length, indices.length, MeshBuilder.createAttributes(VertexAttributes.Usage.Position));
      mesh.setVertices(vertices);
      mesh.setIndices(indices);

      return new MeshPart(null, mesh, 0, mesh.getNumIndices(), GL41.GL_TRIANGLES);
   }

   public void clear()
   {
      if (lastModel != null)
      {
         lastModel.dispose();
         lastModel = null;
      }
      modelInstance = null;
      for (RDXSplineBody edgeLine : edgeLines)
         edgeLine.clear();
      edgeLines.clear();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
      {
         modelInstance.getRenderables(renderables, pool);
      }
      if (!edgeLines.isEmpty())
      {
         for (RDXSplineBody edgeLine : edgeLines)
            edgeLine.getRenderables(renderables, pool);
      }
   }

   public Point3D[] getStartPoints()
   {
      return startPoints;
   }

   public Point3D[] getEndPoints()
   {
      return endPoints;
   }
}