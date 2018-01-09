package us.ihmc.simulationconstructionset.util;

import java.util.ArrayList;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationconstructionset.GroundContactModel;

public class ModularGroundContactModel implements GroundContactModel
{
   private static final long serialVersionUID = 6953909329765322020L;

   private GroundProfile3D groundProfile3D = null;
   
   private final ArrayList<GroundContactModel> groundContactModels = new ArrayList<>();

   @Override
   public void doGroundContact()
   {
      for(int i = 0; i < groundContactModels.size(); i++)
      {
         groundContactModels.get(i).doGroundContact();
      }
   }

   @Override
   public void setGroundProfile3D(GroundProfile3D profile)
   {
      this.groundProfile3D = profile;
      for(int i = 0; i < groundContactModels.size(); i++)
      {
         groundContactModels.get(i).setGroundProfile3D(groundProfile3D);
      }
   }

   @Override
   public GroundProfile3D getGroundProfile3D()
   {
      return groundProfile3D;
   }

   
   public void addGroundContactModel(GroundContactModel groundContactModel)
   {
      groundContactModel.setGroundProfile3D(groundProfile3D);
      groundContactModels.add(groundContactModel);
   }
}
