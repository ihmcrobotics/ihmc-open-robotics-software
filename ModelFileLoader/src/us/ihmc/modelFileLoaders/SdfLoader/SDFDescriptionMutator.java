package us.ihmc.modelFileLoaders.SdfLoader;

import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;

/**
 * This interface defines an API for tweaking the properties of elements in .SDF files
 * at runtime. Functionally, it is equivalent to performing a hand-edit on an SDF file,
 * but does not require us to maintain a manual workflow for hand-editing when we regenerate
 * .sdf files that originate as Xacro templates.
 *
 * This interface does not currently define a way for making additions to a model; it simply
 * allows for modifications to the stock elements in the .xml on disk.
 *
 * As an .SDF can contain multiple models (vehicles, robots, obstacles, etc.) all of the mutation
 * methods take in a model name argument, passing on which model the joint or link belongs to.
 * This information should be checked by implementing classes to be future-proof for loading of
 * multi-model simulation descriptions.
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public interface SDFDescriptionMutator
{
   /**
    * Tweak the properties of a joint belonging to the model given in the input argument.
    * @param model The model to which this joint belongs
    * @param jointHolder The joint to be modified
    */
   void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder);

   /**
    * Tweak the properties of a link belonging to the model given in the input argument
    * @param model The model to which this link belongs
    * @param linkHolder The link to be modified
    */
   void mutateLinkForModel(GeneralizedSDFRobotModel model, SDFLinkHolder linkHolder);

   /**
    * Tweak the properties of the sensor belonging to the model given in the input argument.
    *
    * Note that this does not apply to Force or Contact sensors; they have their own methods.
    * This will be fixed in the future.
    *  @param model The model to which this sensor belongs
    * @param sensor The sensor to be modified
    */
   void mutateSensorForModel(GeneralizedSDFRobotModel model, SDFSensor sensor);

   /**
    * Tweak the properties of a force sensor belonging to the model given in the input argument.
    *  @param model The model to which this sensor belongs
    * @param forceSensor The sensor to be modified
    */
   void mutateForceSensorForModel(GeneralizedSDFRobotModel model, SDFForceSensor forceSensor);

   /**
    * Tweak the properties of a contact sensor belonging to the model given in the input argument.
    *  @param model The model to which this sensor belongs
    * @param contactSensor The sensor to be modified
    */
   void mutateContactSensorForModel(GeneralizedSDFRobotModel model, SDFContactSensor contactSensor);

   /**
    * Perform a generic mutation on the given model. Can be used to add sensors/elements not described
    * in the .SDF file
    *
    * @param model The model you wish to mutate
    */
   void mutateModelWithAdditions(GeneralizedSDFRobotModel model);
}
