package us.ihmc.sensorProcessing.encoder.processors;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ZeroEncoderProcessor implements EncoderProcessor
{
   private static final long serialVersionUID = 7520403547942205332L;
   private final  String name;
   
   public ZeroEncoderProcessor(String name)
   {
      this.name = name;
   }
   
   public double getQ()
   {
      return 0.0;
   }

   public double getQd()
   {
      return 0.0;
   }

   public void update()
   {
   }

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

}
