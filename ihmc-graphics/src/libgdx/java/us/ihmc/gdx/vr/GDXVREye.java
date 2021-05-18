package us.ihmc.gdx.vr;

/**
 * Used to select for which eye a specific property should be accessed.
 */
public enum GDXVREye
{
   Left(0), Right(1);

   private final int index;

   GDXVREye(int index)
   {
      this.index = index;
   }

   public int getIndex()
   {
      return index;
   }
}
