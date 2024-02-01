package us.ihmc.rdx.mesh;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;

/**
 * Dashes along an axis.
 */
public class RDXDashedLineMesh
{
   public static final double EPSILON = 1e-4;
   private Color color = null;
   private final Axis3D dashAxis;
   private final double dashLength;
   private RDXModelInstance modelInstance;
   private final Vector3D drawingVector = new Vector3D();
   private final Point3D drawingPointStart = new Point3D();
   private final Point3D drawingPointEnd = new Point3D();
   private double lineWidth = Double.NaN;
   private double length = Double.NaN;
   private final MeshDataBuilder meshDataBuilder = new MeshDataBuilder();

   public RDXDashedLineMesh(Color color, Axis3D dashAxis, double dashLength)
   {
      this.color = color;
      this.dashAxis = dashAxis;
      this.dashLength = dashLength;
   }

   public void update(RigidBodyTransformReadOnly pose, double lineWidthUpdate, double lengthUpdate)
   {
      boolean outOfDate = !EuclidCoreTools.epsilonEquals(lineWidth, lineWidthUpdate, EPSILON);
      outOfDate |= !EuclidCoreTools.epsilonEquals(length, lengthUpdate, EPSILON);

      lineWidth = lineWidthUpdate;
      length = lengthUpdate;

      if (outOfDate)
      {
         meshDataBuilder.clear();

         drawingPointStart.setToZero();

         drawingVector.setAndScale(-length / 2.0, dashAxis);
         drawingPointStart.add(drawingVector);

         int numberOfDashes = (int) Math.ceil(length / (2.0 * dashLength));
         for (int i = 0; i < numberOfDashes; i++)
         {
            drawingVector.setAndScale(dashLength, dashAxis);

            drawingPointEnd.set(drawingPointStart);
            drawingPointEnd.add(drawingVector);

            meshDataBuilder.addLine(drawingPointStart, drawingPointEnd, lineWidth);

            drawingPointStart.add(drawingVector);
            drawingPointStart.add(drawingVector);
         }

         modelInstance = new RDXModelInstance(RDXModelBuilder.buildModelInstance(meshBuilder ->
                                 meshBuilder.addMesh(meshDataBuilder.generateMeshDataHolder(), color)));
      }

      modelInstance.setPoseInWorldFrame(pose);
   }

   public RDXModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);
   }
}