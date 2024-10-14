package us.ihmc.robotics.math.trajectories.yoVariables;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.abstracts.AbstractFramePolynomial3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoFramePolynomial3D extends AbstractFramePolynomial3D
{
   public YoFramePolynomial3D(String name, int maximumNumberOfCoefficients, ReferenceFrame referenceFrame, YoRegistry registry)
   {
      super(new YoPolynomial3D(name, maximumNumberOfCoefficients, registry), referenceFrame);
   }

   public YoFramePolynomial3D(YoPolynomial xPolynomial, YoPolynomial yPolynomial, YoPolynomial zPolynomial, ReferenceFrame referenceFrame)
   {
      super(new YoPolynomial3D(xPolynomial, yPolynomial, zPolynomial), referenceFrame);
   }

   public YoFramePolynomial3D(YoPolynomial[] yoPolynomials, ReferenceFrame referenceFrame)
   {
      super(new YoPolynomial3D(yoPolynomials[0], yoPolynomials[1], yoPolynomials[2]), referenceFrame);
   }

   public YoFramePolynomial3D(List<YoPolynomial> yoPolynomials, ReferenceFrame referenceFrame)
   {
      super(new YoPolynomial3D(yoPolynomials.get(0), yoPolynomials.get(1), yoPolynomials.get(2)), referenceFrame);
   }

   public static YoFramePolynomial3D[] createYoFramePolynomial3DArray(YoPolynomial[] xPolynomial, YoPolynomial[] yPolynomial, YoPolynomial[] zPolynomial,
                                                                      ReferenceFrame referenceFrame)
   {
      if (xPolynomial.length != yPolynomial.length || xPolynomial.length != zPolynomial.length)
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      YoFramePolynomial3D[] yoPolynomial3Ds = new YoFramePolynomial3D[xPolynomial.length];

      for (int i = 0; i < xPolynomial.length; i++)
      {
         yoPolynomial3Ds[i] = new YoFramePolynomial3D(xPolynomial[i], yPolynomial[i], zPolynomial[i], referenceFrame);
      }
      return yoPolynomial3Ds;
   }

   public static YoFramePolynomial3D[] createYoFramePolynomial3DArray(List<YoPolynomial> xPolynomial, List<YoPolynomial> yPolynomial,
                                                                      List<YoPolynomial> zPolynomial, ReferenceFrame referenceFrame)
   {
      if (xPolynomial.size() != yPolynomial.size() || xPolynomial.size() != zPolynomial.size())
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      YoFramePolynomial3D[] yoPolynomial3Ds = new YoFramePolynomial3D[xPolynomial.size()];

      for (int i = 0; i < xPolynomial.size(); i++)
      {
         yoPolynomial3Ds[i] = new YoFramePolynomial3D(xPolynomial.get(i), yPolynomial.get(i), zPolynomial.get(i), referenceFrame);
      }

      return yoPolynomial3Ds;
   }

   public static List<YoFramePolynomial3D> createYoFramePolynomial3DList(YoPolynomial[] xPolynomial, YoPolynomial[] yPolynomial, YoPolynomial[] zPolynomial,
                                                                         ReferenceFrame referenceFrame)
   {
      if (xPolynomial.length != yPolynomial.length || xPolynomial.length != zPolynomial.length)
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      List<YoFramePolynomial3D> yoPolynomial3Ds = new ArrayList<>(xPolynomial.length);

      for (int i = 0; i < xPolynomial.length; i++)
      {
         yoPolynomial3Ds.add(new YoFramePolynomial3D(xPolynomial[i], yPolynomial[i], zPolynomial[i], referenceFrame));
      }
      return yoPolynomial3Ds;
   }

   public static List<YoFramePolynomial3D> createYoFramePolynomial3DList(List<YoPolynomial> xPolynomial, List<YoPolynomial> yPolynomial,
                                                                         List<YoPolynomial> zPolynomial, ReferenceFrame referenceFrame)
   {
      if (xPolynomial.size() != yPolynomial.size() || xPolynomial.size() != zPolynomial.size())
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      List<YoFramePolynomial3D> yoPolynomial3Ds = new ArrayList<>(xPolynomial.size());

      for (int i = 0; i < xPolynomial.size(); i++)
      {
         yoPolynomial3Ds.add(new YoFramePolynomial3D(xPolynomial.get(i), yPolynomial.get(i), zPolynomial.get(i), referenceFrame));
      }
      return yoPolynomial3Ds;
   }
}
