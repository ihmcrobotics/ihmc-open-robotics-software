package us.ihmc.robotics.perception;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

public class ProjectionTools
{
   /**
    * For a rectilinear pinhole camera, and an image rendered where the bottom left is x, y (0, 0)
    * with x going right and y going up.
    * Also given a z depth in meters, that is forward positive out to the object.
    * The input is therefore in a left-handed coordinate system.
    * Using the principal offsets and the focal lengths, we can project this depth image pixel
    * to a 3D coordinate in IHMC ZUp frame, where x is forward (depth), y left, and z up.
    * The IHMC ZUp frame is a right-handed coordinate system.
    */
   public static void projectDepthPixelToIHMCZUp3D(Tuple3DBasics inputLeftHandedTupleToPack,
                                                   double principalOffsetX,
                                                   double principalOffsetY,
                                                   double focalLengthX,
                                                   double focalLengthY)
   {
      inputLeftHandedTupleToPack.setX((inputLeftHandedTupleToPack.getX() - principalOffsetX) / focalLengthX * inputLeftHandedTupleToPack.getZ());
      inputLeftHandedTupleToPack.setY((inputLeftHandedTupleToPack.getY() - principalOffsetY) / focalLengthY * inputLeftHandedTupleToPack.getZ());
      transformFromXRightYUpZForwardToIHMCZUp(inputLeftHandedTupleToPack);
   }

   /**
    * For a rectilinear pinhole camera, and an image rendered where the bottom left is x, y (0, 0)
    * with x going right and y going up.
    * Also given a z depth in meters, that is forward positive out to the object.
    * The input is therefore in a left-handed coordinate system.
    * We can transform this depth image pixel
    * to a 3D coordinate in IHMC ZUp frame, where x is forward (depth), y left, and z up.
    * The IHMC ZUp frame is a right-handed coordinate system.
    */
   public static void transformFromXRightYUpZForwardToIHMCZUp(Tuple3DBasics inputLeftHandedTupleToPack)
   {
      double newX = inputLeftHandedTupleToPack.getZ(); // In IHMC ZUp, X is forward, so set it to the depth
      // Y goes left in IHMC ZUp frame, so set it to negative projected X, that goes right in the image
      double newY = -inputLeftHandedTupleToPack.getX();
      // The IHMC ZUp becomes the projected Y in the image.
      double newZ = inputLeftHandedTupleToPack.getY();
      inputLeftHandedTupleToPack.set(newX, newY, newZ);
   }
}
