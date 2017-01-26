package us.ihmc.simulationconstructionset.util.environments;

public enum Fiducial
{
   FIDUCIAL50,
   FIDUCIAL100,
   FIDUCIAL150,
   FIDUCIAL200,
   FIDUCIAL250,
   FIDUCIAL300,
   FIDUCIAL350,
   FIDUCIAL400,
   FIDUCIAL450;

   public static final Fiducial[] values = values();

   public String getPathString()
   {
      return "fiducials/png/" + name().toLowerCase() + ".png";
   }
}
