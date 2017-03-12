package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import us.ihmc.robotics.geometry.FramePoint2d;

import java.util.ArrayList;

public class CoMIntegrationTools
{
   private static final FramePoint2d tmpPoint2d = new FramePoint2d();
   private static final FramePoint2d altTmpPoint2d = new FramePoint2d();

   static public void computeCoMPositionUsingConstantCMP(double initialTime, double finalTime, double omega0, FramePoint2d constantCMP, FramePoint2d initialICP,
         FramePoint2d initialCoM, FramePoint2d finalCoMToPack)
   {
      double timeDelta = finalTime - initialTime;

      finalCoMToPack.set(initialICP);
      finalCoMToPack.sub(constantCMP);
      finalCoMToPack.scale(Math.exp(omega0 * timeDelta));

      tmpPoint2d.set(initialCoM);
      tmpPoint2d.sub(initialICP);
      tmpPoint2d.scale(Math.exp(-omega0 * timeDelta));

      finalCoMToPack.sub(tmpPoint2d);

      finalCoMToPack.add(constantCMP);
   }

   static public void computeCoMPositionUsingCubicICP(double initialTime, double finalTime, double omega0, ArrayList<FramePoint2d> coefficients,
         FramePoint2d initialCoM, FramePoint2d finalCoMToPack)
   {
      FramePoint2d c0 = coefficients.get(0);
      FramePoint2d c1 = coefficients.get(1);
      FramePoint2d c2 = coefficients.get(2);
      FramePoint2d c3 = coefficients.get(3);

      double timeDelta = finalTime - initialTime;

      finalCoMToPack.add(c1);
      tmpPoint2d.set(c2);
      tmpPoint2d.scale(-2.0 / omega0);
      finalCoMToPack.add(tmpPoint2d);
      tmpPoint2d.set(c3);
      tmpPoint2d.scale(6 / Math.pow(omega0, 2.0));
      finalCoMToPack.add(tmpPoint2d);
      finalCoMToPack.scale(timeDelta);

      tmpPoint2d.set(c3);
      tmpPoint2d.scale(Math.pow(timeDelta, 3.0));
      finalCoMToPack.add(tmpPoint2d);

      tmpPoint2d.set(c3);
      tmpPoint2d.scale(-3.0 / omega0);
      tmpPoint2d.add(c2);
      tmpPoint2d.scale(Math.pow(timeDelta, 2.0));
      finalCoMToPack.add(tmpPoint2d);

      altTmpPoint2d.set(c0);
      tmpPoint2d.set(c1);
      tmpPoint2d.scale(-1.0 / omega0);
      altTmpPoint2d.add(tmpPoint2d);
      tmpPoint2d.set(c2);
      tmpPoint2d.scale(2.0 / Math.pow(omega0, 2.0));
      altTmpPoint2d.add(tmpPoint2d);
      tmpPoint2d.set(c3);
      tmpPoint2d.scale(6.0 / Math.pow(omega0, 3.0));
      altTmpPoint2d.add(tmpPoint2d);
      finalCoMToPack.add(altTmpPoint2d);

      tmpPoint2d.set(initialCoM);
      tmpPoint2d.sub(altTmpPoint2d);
      tmpPoint2d.scale(Math.exp(-omega0 * timeDelta));
      finalCoMToPack.add(tmpPoint2d);
   }
}
