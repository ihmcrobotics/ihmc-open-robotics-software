package us.ihmc.rdx.simulation.bullet;

import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.physics.bullet.Bullet;
import com.badlogic.gdx.physics.bullet.collision.btConvexHullShape;
import com.badlogic.gdx.physics.bullet.linearmath.LinearMath;
import us.ihmc.log.LogTools;

import java.nio.FloatBuffer;

public class RDXBulletTools
{
   private static boolean bulletInitialized = false;

   public static void ensureBulletInitialized()
   {
      if (!bulletInitialized)
      {
         bulletInitialized = true;
         Bullet.init();
         LogTools.info("Loaded Bullet version {}", LinearMath.btGetVersion());
      }
   }

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
