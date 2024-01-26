package us.ihmc.commonWalkingControlModules.parameterEstimation;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicEllipsoid3DDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class InertiaVisualizationTools
{
   public static ArrayList<YoInertiaEllipsoid> createYoInertiaEllipsoids(RigidBodyReadOnly rootBody, YoRegistry registry)
   {
      ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids = new ArrayList<>();
      // This rootBody is expected to be the pelvis or first physical body, not the elevator.
      List<? extends RigidBodyReadOnly> bodies = rootBody.subtreeList();
      if (rootBody.getName().equals("elevator"))
      {
         bodies.remove(0);
      }
      for (int i = 0; i < bodies.size(); i++)
      {
         if (bodies.get(i).isRootBody())
            continue;

         YoInertiaEllipsoid yoInertiaEllipsoid = new YoInertiaEllipsoid(bodies.get(i), registry);
         yoInertiaEllipsoids.add(yoInertiaEllipsoid);
      }
      return yoInertiaEllipsoids;
   }

   public static YoGraphicDefinition getInertiaEllipsoidGroup(RigidBodyReadOnly rootBody, ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids)
   {
      YoGraphicGroupDefinition ellipsoidGroup = new YoGraphicGroupDefinition("Inertia Ellipsoids");
      // This rootBody is expected to be the pelvis or first physical body, not the elevator.
      List<? extends RigidBodyReadOnly> bodies = rootBody.subtreeList();
      if (rootBody.getName().equals("elevator"))
      {
         bodies.remove(0);
      }

      for (int i = 0; i < bodies.size(); i++)
      {
         RigidBodyReadOnly body = bodies.get(i);
         if (body.isRootBody())
            continue;

         YoGraphicEllipsoid3DDefinition ellipsoid = yoInertiaEllipsoids.get(i).getEllipsoidDefinition();
         if (ellipsoid == null)
            continue;

         ellipsoidGroup.addChild(ellipsoid);
      }

      return ellipsoidGroup;
   }

   /**
    * Takes a scale between 0 and 1 and returns the RGB values for the Coolwarm colormap at that scale.
    * <p>
    * NOTE: This method uses coefficients from a polynomial fit of the RGB values of the Coolwarm colormap from Python. The coefficients were computed
    * externally in a Python script. For a higher or lower order fit, the coefficients would need to be regenerated.
    * </p>
    *
    * @param scale Number between 0 and 1, determines the location on the colormap.
    * @return RGB values for the colormap at the given scale.
    */
   public static int[] getRGBForCoolwarmColorMap(double scale)
   {
      double[] redCoefficients = {0.22799943, 1.17622677, -0.05769346, 3.08415998, -6.81520504, 3.1203027};
      double[] greenCoefficients = {0.30822069, 1.26972463, 3.09837548, -10.38670071, 7.89978329, -2.06379977};
      double[] blueCoefficients = {0.7571628, 1.33148599, 0.18877437, -9.27125131, 10.77877482, -3.61308895};

      if (scale < 0 || scale > 1)
      {
         throw new IllegalArgumentException("Value must be between 0 and 1");
      }

      int red = evaluateRGBPolynomial(redCoefficients, scale);
      int green = evaluateRGBPolynomial(greenCoefficients, scale);
      int blue = evaluateRGBPolynomial(blueCoefficients, scale);

      return new int[] {red, green, blue};
   }

   /**
    * Takes a scale between 0 and 1 and returns the RGB values for the PiYG colormap at that scale.
    * <p>
    * NOTE: This method uses coefficients from a polynomial fit of the RGB values of the PiYG colormap from Python. The coefficients were computed externally
    * in a Python script. For a higher or lower order fit, the coefficients would need to be regenerated.
    * </p>
    *
    * @param scale Number between 0 and 1, determines the location on the colormap.
    * @return RGB values for the colormap at the given scale.
    */
   private static int[] getRGBForPiYGColorMap(double scale)
   {
      double[] redCoefficients = {0.55542857, 2.55793281, -7.64547912, 17.59694419, -23.94099841, 11.07570777};
      double[] greenCoefficients = {-4.52665027e-03, 2.96292190e-01, 1.57697343e+01, -4.01222209e+01, 3.60874809e+01, -1.16116188e+01};
      double[] blueCoefficients = {0.33713237, 0.8968716, 6.34398907, -12.82119389, 0.03361242, 5.35789659};

      if (scale < 0 || scale > 1)
      {
         throw new IllegalArgumentException("Value must be between 0 and 1");
      }

      int red = evaluateRGBPolynomial(redCoefficients, scale);
      int green = evaluateRGBPolynomial(greenCoefficients, scale);
      int blue = evaluateRGBPolynomial(blueCoefficients, scale);

      return new int[] {red, green, blue};
   }

   /**
    * Takes a scale between 0 and 1 and returns the RGB values for the Viridis colormap at that scale.
    * <p>
    * NOTE: This method uses coefficients from a polynomial fit of the RGB values of the Viridis colormap from Python. The coefficients were computed externally
    * in a Python script. For a higher or lower order fit, the coefficients would need to be regenerated.
    * </p>
    *
    * @param scale Number between 0 and 1, determines the location on the colormap.
    * @return RGB values for the colormap at the given scale.
    */
   private static int[] getRGBForViridisColorMap(double scale)
   {
      double[] redCoefficients = {0.2774879, -0.11255063, 2.10607102, -14.14404302, 23.72334937, -10.90841729};
      double[] greenCoefficients = {0.00247317, 1.49535097, -1.31431592, 1.28943779, -0.223256, -0.35398068};
      double[] blueCoefficients = {0.31371651, 2.2855777, -9.80527933, 22.74666121, -25.84324129, 10.38676565};

      if (scale < 0 || scale > 1)
      {
         throw new IllegalArgumentException("Value must be between 0 and 1");
      }

      int red = evaluateRGBPolynomial(redCoefficients, scale);
      int green = evaluateRGBPolynomial(greenCoefficients, scale);
      int blue = evaluateRGBPolynomial(blueCoefficients, scale);

      return new int[] {red, green, blue};
   }

   /**
    * Helper method for {@link #getRGBForViridisColorMap(double)}. Evaluates a polynomial for one color (R, G, or B) with the given
    * coefficients at the given scale.
    *
    * @param colorCoefficients Coefficients of the polynomial, in order of increasing degree.
    * @param scale             Scale at which to evaluate the polynomial.
    * @return Value of the polynomial at the given scale.
    */
   private static int evaluateRGBPolynomial(double[] colorCoefficients, double scale)
   {
      double rgbValue = 0.0;
      for (int i = 0; i < colorCoefficients.length; i++)
      {
         rgbValue += colorCoefficients[i] * Math.pow(scale, i);
      }
      return (int) Math.round(rgbValue * 255);  // Scale to 0-255 range
   }
}
