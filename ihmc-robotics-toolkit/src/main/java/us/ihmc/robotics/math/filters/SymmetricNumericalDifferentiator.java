package us.ihmc.robotics.math.filters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;

public class SymmetricNumericalDifferentiator
{
   public static double[] differentiate(double[] xs, double[] f)
   {
      double[] df = new double[f.length];
      for (int i = 1; i < f.length - 1; i++)
      {
         double lower = f[i - 1];
         double upper = f[i + 1];
         double diff = (upper - lower) / (xs[i] - xs[i - 1] + xs[i + 1] - xs[i]);
         df[i] = diff;
      }

      // Add endpoints by padding out elements 1 and n-1
      df[0] = df[1];
      df[df.length - 1] = df[df.length - 2];

      return df;
   }

   public static double[] differentiate(double dt, double[] f)
   {
      double[] xs = new double[f.length];
      for (int i = 0; i < xs.length; i++)
      {
         xs[i] = i * dt;
      }
      return differentiate(xs, f);
   }

   public static List<FrameVector3D> differentiate(double dt, ReferenceFrame frame, List<? extends FrameTuple3DBasics> xs)
   {
      List<FrameVector3D> vs = new ArrayList<>(xs.size());
      for (int i = 1; i < xs.size() - 1; i++)
      {
         FrameTuple3DBasics before = xs.get(i - 1);
         FrameTuple3DBasics after = xs.get(i + 1);

         FrameVector3D v = difference(dt, frame, before, after);
         vs.add(v);
      }

      // Add endpoints by padding out elements 1 and n-1
      vs.add(0, new FrameVector3D(vs.get(0)));
      vs.add(new FrameVector3D(vs.get(vs.size() - 1)));

      return vs;
   }

   private static FrameVector3D difference(double dt, ReferenceFrame frame, FrameTuple3DBasics before, FrameTuple3DBasics after)
   {
      after.checkReferenceFrameMatch(before);

      FrameVector3D v = new FrameVector3D(after.getReferenceFrame());
      v.sub(after, before);
      v.scale(1 / (2.0 * dt));
      v.changeFrame(frame);

      return v;
   }

}
