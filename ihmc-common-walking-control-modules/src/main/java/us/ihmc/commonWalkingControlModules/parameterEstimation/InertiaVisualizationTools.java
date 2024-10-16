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
   public static final double[] COOLWARM_RED_COEFFICIENTS = new double[] {0.22799943, 1.17622677, -0.05769346, 3.08415998, -6.81520504, 3.1203027};
   public static final double[] COOLWARM_GREEN_COEFFICIENTS = new double[] {0.30822069, 1.26972463, 3.09837548, -10.38670071, 7.89978329, -2.06379977};
   public static final double[] COOLWARM_BLUE_COEFFICIENTS = new double[] {0.7571628, 1.33148599, 0.18877437, -9.27125131, 10.77877482, -3.61308895};

   private static final double[] PIYG_RED_COEFFICIENTS = new double[] {0.55542857, 2.55793281, -7.64547912, 17.59694419, -23.94099841, 11.07570777};
   private static final double[] PIYG_GREEN_COEFFICIENTS = new double[] {-4.52665027e-03,
                                                                         2.96292190e-01,
                                                                         1.57697343e+01,
                                                                         -4.01222209e+01,
                                                                         3.60874809e+01,
                                                                         -1.16116188e+01};
   private static final double[] PIYG_BLUE_COEFFICIENTS = new double[] {0.33713237, 0.8968716, 6.34398907, -12.82119389, 0.03361242, 5.35789659};

   private static final double[] VIRIDIS_RED_COEFFICIENTS = new double[] {0.2774879, -0.11255063, 2.10607102, -14.14404302, 23.72334937, -10.90841729};
   private static final double[] VIRIDIS_GREEN_COEFFICIENTS = new double[] {0.00247317, 1.49535097, -1.31431592, 1.28943779, -0.223256, -0.35398068};
   private static final double[] VIRIDIS_BLUE_COEFFICIENTS = new double[] {0.31371651, 2.2855777, -9.80527933, 22.74666121, -25.84324129, 10.38676565};

   /**
    * Creates a list of ellipsoids visuals corresponding to all the rigid bodies in the system given the root rigid body. The update() can be called for each
    * element of this list in a loop to update the visuals.
    *
    * @param rootBody - The root body of the robot. This is expected to be the pelvis or first physical body, not the elevator.
    * @param registry - The registry for the ellipsoids.
    */
   public static ArrayList<YoInertiaEllipsoid> createYoInertiaEllipsoids(RigidBodyReadOnly rootBody, YoRegistry registry)
   {
      ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids = new ArrayList<>();
      // This rootBody is expected to be the pelvis or first physical body, not the elevator.
      List<? extends RigidBodyReadOnly> bodies = rootBody.subtreeList();

      for (int i = 0; i < bodies.size(); i++)
      {
         if (bodies.get(i).isRootBody())
            continue;

         YoInertiaEllipsoid yoInertiaEllipsoid = new YoInertiaEllipsoid(bodies.get(i), registry);
         yoInertiaEllipsoids.add(yoInertiaEllipsoid);
      }
      return yoInertiaEllipsoids;
   }

   /**
    * Creates a graphic group to be returned by getSCS2YoGraphics().
    *
    * @param yoInertiaEllipsoids - The list of inertia ellipsoids to be converted to definitions and added to the group.
    */
   public static YoGraphicDefinition getInertiaEllipsoidGroup(ArrayList<YoInertiaEllipsoid> yoInertiaEllipsoids)
   {
      YoGraphicGroupDefinition ellipsoidGroup = new YoGraphicGroupDefinition("Inertia Ellipsoids");

      for (YoInertiaEllipsoid yoInertiaEllipsoid : yoInertiaEllipsoids)
      {
         YoGraphicEllipsoid3DDefinition ellipsoidDefinition = yoInertiaEllipsoid.getEllipsoidDefinition();

         if (ellipsoidDefinition == null)
            continue;

         ellipsoidGroup.addChild(ellipsoidDefinition);
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
      checkScaleInUnitWindow(scale);

      int red = evaluateRGBPolynomial(COOLWARM_RED_COEFFICIENTS, scale);
      int green = evaluateRGBPolynomial(COOLWARM_GREEN_COEFFICIENTS, scale);
      int blue = evaluateRGBPolynomial(COOLWARM_BLUE_COEFFICIENTS, scale);

      return new int[] {red, green, blue};
   }

   public static void getRGBForCoolwarmColorMap(double scale, int[] rgbToPack)
   {
      checkScaleInUnitWindow(scale);

      rgbToPack[0] = evaluateRGBPolynomial(COOLWARM_RED_COEFFICIENTS, scale);
      rgbToPack[1] = evaluateRGBPolynomial(COOLWARM_GREEN_COEFFICIENTS, scale);
      rgbToPack[2] = evaluateRGBPolynomial(COOLWARM_BLUE_COEFFICIENTS, scale);
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
   public static int[] getRGBForPiYGColorMap(double scale)
   {
      checkScaleInUnitWindow(scale);

      int red = evaluateRGBPolynomial(PIYG_RED_COEFFICIENTS, scale);
      int green = evaluateRGBPolynomial(PIYG_GREEN_COEFFICIENTS, scale);
      int blue = evaluateRGBPolynomial(PIYG_BLUE_COEFFICIENTS, scale);

      return new int[] {red, green, blue};
   }

   public static void getRGBForPiYGColorMap(double scale, int[] rgbToPack)
   {
      checkScaleInUnitWindow(scale);

      rgbToPack[0] = evaluateRGBPolynomial(PIYG_RED_COEFFICIENTS, scale);
      rgbToPack[1] = evaluateRGBPolynomial(PIYG_GREEN_COEFFICIENTS, scale);
      rgbToPack[2] = evaluateRGBPolynomial(PIYG_BLUE_COEFFICIENTS, scale);
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
   public static int[] getRGBForViridisColorMap(double scale)
   {
      checkScaleInUnitWindow(scale);

      int red = evaluateRGBPolynomial(VIRIDIS_RED_COEFFICIENTS, scale);
      int green = evaluateRGBPolynomial(VIRIDIS_GREEN_COEFFICIENTS, scale);
      int blue = evaluateRGBPolynomial(VIRIDIS_BLUE_COEFFICIENTS, scale);

      return new int[] {red, green, blue};
   }

   public static void getRGBForViridisColorMap(double scale, int[] rgbToPack)
   {
      checkScaleInUnitWindow(scale);

      rgbToPack[0] = evaluateRGBPolynomial(VIRIDIS_RED_COEFFICIENTS, scale);
      rgbToPack[1] = evaluateRGBPolynomial(VIRIDIS_GREEN_COEFFICIENTS, scale);
      rgbToPack[2] = evaluateRGBPolynomial(VIRIDIS_BLUE_COEFFICIENTS, scale);
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

   private static void checkScaleInUnitWindow(double scale)
   {
      if (scale < 0 || scale > 1)
      {
         throw new IllegalArgumentException("Value must be between 0 and 1");
      }
   }
}
