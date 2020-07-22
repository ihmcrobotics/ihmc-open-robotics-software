package us.ihmc.gdx;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class GDXDataTypeTools
{
   public static Vector3 toGDX(Tuple3DReadOnly euclidTuple)
   {
      return new Vector3(euclidTuple.getX32(), euclidTuple.getY32(), euclidTuple.getZ32());
   }

   public static Vector2 toGDX(Tuple2DReadOnly euclidTuple)
   {
      return new Vector2(euclidTuple.getX32(), euclidTuple.getY32());
   }
}
