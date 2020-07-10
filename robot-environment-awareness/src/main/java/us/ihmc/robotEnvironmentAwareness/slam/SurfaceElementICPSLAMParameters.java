package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.Scanner;

public class SurfaceElementICPSLAMParameters
{
   public SurfaceElementICPSLAMParameters()
   {
      setDefaultParameters();
   }

   public SurfaceElementICPSLAMParameters(SurfaceElementICPSLAMParameters other)
   {
      set(other);
   }

   public void set(SurfaceElementICPSLAMParameters other)
   {
   }

   public void setDefaultParameters()
   {
   }

   @Override
   public String toString()
   {
      return "octreeResolution";
   }

   public static SurfaceElementICPSLAMParameters parse(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      SurfaceElementICPSLAMParameters parameters = new SurfaceElementICPSLAMParameters();

      return parameters;
   }
}
