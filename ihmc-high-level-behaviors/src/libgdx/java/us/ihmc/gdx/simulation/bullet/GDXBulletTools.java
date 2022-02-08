package us.ihmc.gdx.simulation.bullet;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.physics.bullet.collision.btConvexHullShape;

import java.nio.FloatBuffer;

public class GDXBulletTools
{
   public static btConvexHullShape createConcaveHullShapeFromMesh(Mesh mesh)
   {
      return createConcaveHullShapeFromMesh(mesh.getVerticesBuffer(), mesh.getNumVertices(), mesh.getVertexSize());
   }

   public static btConvexHullShape createConcaveHullShapeFromMesh(FloatBuffer floatBuffer, int numberOfPoints, int stride)
   {
      floatBuffer.rewind();
      return new btConvexHullShape(floatBuffer, numberOfPoints, stride);
   }
}
