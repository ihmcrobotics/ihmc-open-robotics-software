package us.ihmc.robotics.math.trajectories.core;

import us.ihmc.commons.trajectories.core.Polynomial;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.abstracts.AbstractFramePolynomial3D;

import java.util.ArrayList;
import java.util.List;

public class FramePolynomial3D extends AbstractFramePolynomial3D
{
   public FramePolynomial3D(int maximumNumberOfCoefficients, ReferenceFrame referenceFrame)
   {
      super(new Polynomial3D(maximumNumberOfCoefficients), referenceFrame);
   }

   public FramePolynomial3D(Polynomial xPolynomial, Polynomial yPolynomial, Polynomial zPolynomial, ReferenceFrame referenceFrame)
   {
      super(new Polynomial3D(xPolynomial, yPolynomial, zPolynomial), referenceFrame);
   }

   public FramePolynomial3D(Polynomial[] polynomials, ReferenceFrame referenceFrame)
   {
      super(new Polynomial3D(polynomials[0], polynomials[1], polynomials[2]), referenceFrame);
   }

   public FramePolynomial3D(List<Polynomial> polynomials, ReferenceFrame referenceFrame)
   {
      super(new Polynomial3D(polynomials.get(0), polynomials.get(1), polynomials.get(2)), referenceFrame);
   }

   public static FramePolynomial3D[] createFramePolynomial3DArray(Polynomial[] xPolynomial, Polynomial[] yPolynomial, Polynomial[] zPolynomial,
                                                                    ReferenceFrame referenceFrame)
   {
      if (xPolynomial.length != yPolynomial.length || xPolynomial.length != zPolynomial.length)
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      FramePolynomial3D[] yoPolynomial3Ds = new FramePolynomial3D[xPolynomial.length];

      for (int i = 0; i < xPolynomial.length; i++)
      {
         yoPolynomial3Ds[i] = new FramePolynomial3D(xPolynomial[i], yPolynomial[i], zPolynomial[i], referenceFrame);
      }
      return yoPolynomial3Ds;
   }

   public static FramePolynomial3D[] createFramePolynomial3DArray(List<Polynomial> xPolynomial, List<Polynomial> yPolynomial,
                                                                    List<Polynomial> zPolynomial, ReferenceFrame referenceFrame)
   {
      if (xPolynomial.size() != yPolynomial.size() || xPolynomial.size() != zPolynomial.size())
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      FramePolynomial3D[] yoPolynomial3Ds = new FramePolynomial3D[xPolynomial.size()];

      for (int i = 0; i < xPolynomial.size(); i++)
      {
         yoPolynomial3Ds[i] = new FramePolynomial3D(xPolynomial.get(i), yPolynomial.get(i), zPolynomial.get(i), referenceFrame);
      }

      return yoPolynomial3Ds;
   }

   public static List<FramePolynomial3D> createFramePolynomial3DList(Polynomial[] xPolynomial, Polynomial[] yPolynomial, Polynomial[] zPolynomial,
                                                                       ReferenceFrame referenceFrame)
   {
      if (xPolynomial.length != yPolynomial.length || xPolynomial.length != zPolynomial.length)
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      List<FramePolynomial3D> yoPolynomial3Ds = new ArrayList<>(xPolynomial.length);

      for (int i = 0; i < xPolynomial.length; i++)
      {
         yoPolynomial3Ds.add(new FramePolynomial3D(xPolynomial[i], yPolynomial[i], zPolynomial[i], referenceFrame));
      }
      return yoPolynomial3Ds;
   }

   public static List<FramePolynomial3D> createFramePolynomial3DList(List<Polynomial> xPolynomial, List<Polynomial> yPolynomial,
                                                                       List<Polynomial> zPolynomial, ReferenceFrame referenceFrame)
   {
      if (xPolynomial.size() != yPolynomial.size() || xPolynomial.size() != zPolynomial.size())
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      List<FramePolynomial3D> yoPolynomial3Ds = new ArrayList<>(xPolynomial.size());

      for (int i = 0; i < xPolynomial.size(); i++)
      {
         yoPolynomial3Ds.add(new FramePolynomial3D(xPolynomial.get(i), yPolynomial.get(i), zPolynomial.get(i), referenceFrame));
      }
      return yoPolynomial3Ds;
   }
}
