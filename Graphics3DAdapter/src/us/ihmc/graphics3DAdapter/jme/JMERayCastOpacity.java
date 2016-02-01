package us.ihmc.graphics3DAdapter.jme;

public enum JMERayCastOpacity
{
   OPAQUE,
   TRANSPARENT;
   public static final String USER_DATA_FIELD="RayCastOpacity";
   public String toString()
   {
      switch(this)
      {
      case OPAQUE:
         return "RayCastOpaque";
      case TRANSPARENT:
         return "RayCastTransparent";
      default:
         return "RayCastTransparent";
      }
   }
}
