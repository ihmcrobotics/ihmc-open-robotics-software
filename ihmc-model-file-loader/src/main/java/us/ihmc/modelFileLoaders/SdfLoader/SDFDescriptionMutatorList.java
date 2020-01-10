package us.ihmc.modelFileLoaders.SdfLoader;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;

public class SDFDescriptionMutatorList implements SDFDescriptionMutator
{
   private final List<SDFDescriptionMutator> mutators = new ArrayList<>();

   public SDFDescriptionMutatorList()
   {
   }

   public SDFDescriptionMutatorList(SDFDescriptionMutator... mutators)
   {
      this.mutators.addAll(Arrays.asList(mutators));
   }

   public SDFDescriptionMutatorList(Collection<? extends SDFDescriptionMutator> mutators)
   {
      this.mutators.addAll(mutators);
   }

   public void addMutator(SDFDescriptionMutator mutator)
   {
      mutators.add(mutator);
   }

   public void removeMutator(SDFDescriptionMutator mutator)
   {
      mutators.remove(mutator);
   }

   @Override
   public void mutateContactSensorForModel(GeneralizedSDFRobotModel model, SDFContactSensor contactSensor)
   {
      mutators.forEach(mutator -> mutator.mutateContactSensorForModel(model, contactSensor));
   }

   @Override
   public void mutateForceSensorForModel(GeneralizedSDFRobotModel model, SDFForceSensor forceSensor)
   {
      mutators.forEach(mutator -> mutator.mutateForceSensorForModel(model, forceSensor));
   }

   @Override
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {
      mutators.forEach(mutator -> mutator.mutateJointForModel(model, jointHolder));
   }

   @Override
   public void mutateLinkForModel(GeneralizedSDFRobotModel model, SDFLinkHolder linkHolder)
   {
      mutators.forEach(mutator -> mutator.mutateLinkForModel(model, linkHolder));
   }

   @Override
   public void mutateModelWithAdditions(GeneralizedSDFRobotModel model)
   {
      mutators.forEach(mutator -> mutator.mutateModelWithAdditions(model));
   }

   @Override
   public void mutateSensorForModel(GeneralizedSDFRobotModel model, SDFSensor sensor)
   {
      mutators.forEach(mutator -> mutator.mutateSensorForModel(model, sensor));
   }
}
