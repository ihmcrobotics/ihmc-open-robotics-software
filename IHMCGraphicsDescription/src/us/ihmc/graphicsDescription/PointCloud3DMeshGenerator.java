package us.ihmc.graphicsDescription;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Simple and efficient mesh generator for 3D point represented as spheres.
 * <p>
 * It works as the {@link SegmentedLine3DMeshDataGenerator}.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class PointCloud3DMeshGenerator
{
   /**
    * The meshes for this generator. Not that the meshes are recycled.
    */
   private final MeshDataHolder[] meshDataHolders;

   /**
    * The {@code pointTemplate} is used to reset the other meshes without recomputing the actual
    * meshes.
    */
   private final MeshDataHolder pointTemplate;
   private double pointRadius = 1.0;

   /**
    * Create a new mesh generator given the properties necessary to initialize the meshes.
    * <p>
    * The points radius is set to 1 meter but can be changed later via the method
    * {@link #setPointRadius(double)}.
    * </p>
    * 
    * @param numberOfPoints number of spheres to be displayed. Necessary to evaluate the number of
    *           meshes needed for this generator.
    * @param resolution refers to the quality of the rendering for each sphere. A high value will
    *           result in a smooth sphere, while a low value result in a polygonized sphere.
    */
   public PointCloud3DMeshGenerator(int numberOfPoints, int resolution)
   {
      this(numberOfPoints, resolution, 1.0);
   }

   /**
    * Create a new mesh generator given the properties necessary to initialize the meshes.
    * 
    * @param numberOfPoints number of spheres to be displayed. Necessary to evaluate the number of
    *           meshes needed for this generator.
    * @param resolution refers to the quality of the rendering for each sphere. A high value will
    *           result in a smooth sphere, while a low value result in a polygonized sphere.
    * @param pointRadius radius used to create all the spheres.
    * 
    */
   public PointCloud3DMeshGenerator(int numberOfPoints, int resolution, double pointRadius)
   {
      this.pointRadius = pointRadius;
      meshDataHolders = new MeshDataHolder[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
         meshDataHolders[i] = MeshDataGenerator.Sphere(pointRadius, resolution, resolution);
      pointTemplate = MeshDataGenerator.Sphere(pointRadius, resolution, resolution);
   }

   /**
    * Update the meshes of this generator to represent a cloud of sphere located at the given
    * locations.
    * <p>
    * The resulting meshes can be obtained using {@link #getMeshDataHolders()}.
    * </p>
    * 
    * @param pointLocations the positions for each sphere.
    * @throws RuntimeException if {@code pointLocations.length != this.getNumberOfPoints()}.
    */
   public void compute(Point3DReadOnly[] pointLocations)
   {
      if (pointLocations.length != meshDataHolders.length)
         throw new RuntimeException("Unexpected array size. Expected: " + meshDataHolders.length + ", but was: " + pointLocations.length);

      for (int i = 0; i < meshDataHolders.length; i++)
      {
         Point3DReadOnly center = pointLocations[i];
         Point3D32[] pointVertices = meshDataHolders[i].getVertices();
         Point3D32[] templateVertices = pointTemplate.getVertices();

         for (int vertexIndex = 0; vertexIndex < pointVertices.length; vertexIndex++)
         {
            pointVertices[vertexIndex].add(templateVertices[vertexIndex], center);
         }
      }
   }

   /**
    * Changes the radius used for all the spheres.
    * <p>
    * The compute method has to be called before the change is effective on the output
    * meshes.
    * </p>
    * 
    * @param pointRadius the new radius to be used for the spheres.
    */
   public void setPointRadius(double pointRadius)
   {
      double scale = pointRadius / this.pointRadius;

      for (Point3DBasics vertex : pointTemplate.getVertices())
      {
         vertex.scale(scale);
      }
      this.pointRadius = pointRadius;
   }

   /**
    * Gets the number of spheres this generator is currently setup for.
    * 
    * @return the number of points.
    */
   public int getNumberOfPoints()
   {
      return meshDataHolders.length;
   }

   /**
    * Gets the reference to the output meshes of this generator.
    * <p>
    * WARNING: the meshes are part of the internal memory of this generator and are updated when
    * calling the compute method.
    * </p>
    * 
    * @return the reference to the output meshes of this generator.
    */
   public MeshDataHolder[] getMeshDataHolders()
   {
      return meshDataHolders;
   }
}
