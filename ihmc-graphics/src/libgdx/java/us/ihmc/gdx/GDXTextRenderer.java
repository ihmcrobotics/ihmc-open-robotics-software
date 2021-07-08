package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

import java.util.Comparator;
import java.util.TreeMap;

public class GDXTextRenderer implements RenderableProvider
{
   private final Octree<String> strings = new Octree<>();
   //TODO use a sprite batch probably idk

   public void create() {

   }

   public void update() {

   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   public boolean doesTextExistAtPosition(Point3D position) {
      return strings.containsAt(position);
   }

   public void setTextAtPosition(Point3D position, String text) {
      strings.put(position, text);
   }

   public void clearTextAtPosition(Point3D position) {
      strings.removeAt(position);
   }

   /**
    * Note that range is applied +/- : if position.x is 4 and range.x is 1, an element at either 3 or 5 will be counted.
    */
   public boolean doesTextExistNearPosition(Point3D position, Vector3D range) {
      return strings.containsWithin(position, range);
   }

   /**
    * Note that range is applied +/- : if position.x is 4 and range.x is 1, an element at either 3 or 5 will be counted.
    */
   public void removeTextAroundPosition(Point3D position, Vector3D range) {
      strings.removeWithin(position, range);
   }

   public void clearAllText() {
      strings.clear();
   }

   //THIS IS NOT MEANT FOR USE OTHER THAN BY THIS CLASS - it's not a complete, efficient, or otherwise good implementation of an Octree
   private final class Octree<T>
   {
      private final TreeMap<Point3D, T> X = new TreeMap<>(Comparator.comparing(Point3D::getX));
      private final TreeMap<Point3D, T> Y = new TreeMap<>(Comparator.comparing(Point3D::getY));
      private final TreeMap<Point3D, T> Z = new TreeMap<>(Comparator.comparing(Point3D::getZ));

      public void put(Point3D pos, T value) {
         X.put(pos, value);
         Y.put(pos, value);
         Z.put(pos, value);
      }

      public boolean containsAt(Point3D pos) {
         return containsWithin(pos, new Vector3D(0, 0, 0));
      }

      //range should be (0, 0, 0) if you only want to check the exact point
      public boolean containsWithin(Point3D pos, Vector3D range) {
         Point3D from = new Point3D(pos);
         from.sub(range);
         Point3D to = new Point3D(pos);
         to.add(range);

         if (!X.subMap(from, to).isEmpty())
            return true;
         else if (!Y.subMap(from, to).isEmpty())
            return true;
         else
            return !Z.subMap(from, to).isEmpty();
      }

      public void removeAt(Point3D pos) {
         removeWithin(pos, new Vector3D(0, 0, 0));
      }

      //range should be (0, 0, 0) if you only want to remove the exact point
      public void removeWithin(Point3D pos, Vector3D range) {
         Point3D from = new Point3D(pos);
         from.sub(range);
         Point3D to = new Point3D(pos);
         to.add(range);

         X.subMap(from, to).clear();
         Y.subMap(from, to).clear();
         Z.subMap(from, to).clear();
      }

      public void clear() {
         X.clear();
         Y.clear();
         Z.clear();
      }
   }
}
